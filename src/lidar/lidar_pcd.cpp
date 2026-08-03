#include "lidar.h"

#include "slam_log_reporter.h"

#include "cctype"
#include "cstdlib"
#include "cstring"
#include "fstream"
#include "sstream"
#include "string"
#include "vector"

namespace sensor_model {

namespace {

/*
 * Minimal LZF decompressor, compatible with the liblzf implementation bundled
 * by PCL. It is used to decode "binary_compressed" PCD point data.
 *
 * Stream format:
 *   - Control byte ctrl:
 *       * ctrl < 32        : literal run of (ctrl + 1) bytes.
 *       * otherwise        : back reference.
 *   - Back reference layout:
 *       len      = ctrl >> 5
 *       if len == 7: len += next byte            (length extension)
 *       distance = ((ctrl & 0x1f) << 8) + next byte + 1
 *       match    = len + 2 bytes copied from (output - distance)
 *
 * Returns the number of bytes written to `out`, or 0 on error.
 */
uint32_t LzfDecompress(const uint8_t *in, uint32_t in_len, uint8_t *out, uint32_t out_len) {
    const uint8_t *ip = in;
    const uint8_t *const in_end = in + in_len;
    uint8_t *op = out;
    const uint8_t *const out_end = out + out_len;

    while (ip < in_end) {
        const uint32_t ctrl = *ip++;

        if (ctrl < (1u << 5)) {
            // Literal run.
            const uint32_t run = ctrl + 1;
            if (ip + run > in_end || op + run > out_end) {
                return 0;
            }
            std::memcpy(op, ip, run);
            op += run;
            ip += run;
        } else {
            // Back reference.
            uint32_t len = ctrl >> 5;
            if (len == 7) {
                if (ip >= in_end) {
                    return 0;
                }
                len += *ip++;
            }
            if (ip >= in_end) {
                return 0;
            }
            const uint32_t distance = ((ctrl & 0x1fu) << 8) + *ip++ + 1;
            if (distance > static_cast<uint32_t>(op - out)) {
                return 0;
            }

            len += 2;  // Minimum match length is 3.
            if (op + len > out_end) {
                return 0;
            }

            const uint8_t *ref = op - distance;
            if (ref + len <= op) {
                // Non-overlapping match, can be copied in one go.
                std::memcpy(op, ref, len);
                op += len;
            } else {
                // Overlapping match, copy byte by byte.
                while (len--) {
                    *op++ = *ref++;
                }
            }
        }
    }

    return static_cast<uint32_t>(op - out);
}

/* Field information parsed from the PCD header. */
struct PcdField {
    std::string name;
    uint32_t size = 1;  // Bytes per element.
    char type = 'F';    // 'I' (int), 'U' (uint), 'F' (float).
    uint32_t count = 1; // Number of elements.
};

/* Metadata of a PCD header. */
struct PcdHeader {
    std::vector<PcdField> fields;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t points = 0;
    std::string data_type;  // "ascii", "binary" or "binary_compressed".
};

bool ParseUnsigned(const std::string &text, uint32_t &value) {
    if (text.empty()) {
        return false;
    }
    char *end = nullptr;
    const unsigned long v = std::strtoul(text.c_str(), &end, 10);
    if (end == text.c_str()) {
        return false;
    }
    value = static_cast<uint32_t>(v);
    return true;
}

/* Read one line (without the terminating '\n', and any trailing '\r') from a byte buffer. */
bool ReadLine(const std::vector<uint8_t> &buffer, size_t &pos, std::string &line) {
    line.clear();
    while (pos < buffer.size() && buffer[pos] != '\n') {
        line.push_back(static_cast<char>(buffer[pos]));
        ++pos;
    }
    if (pos >= buffer.size() && line.empty()) {
        return false;
    }
    if (pos < buffer.size()) {
        ++pos;  // Consume '\n'.
    }
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    return true;
}

/* Parse the PCD header. On success `data_offset` points at the first byte after the DATA line. */
bool ParsePcdHeader(const std::vector<uint8_t> &buffer, size_t &data_offset, PcdHeader &header) {
    size_t pos = 0;
    while (pos < buffer.size()) {
        std::string line;
        if (!ReadLine(buffer, pos, line)) {
            break;
        }
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        std::string key;
        iss >> key;

        if (key == "FIELDS") {
            std::string name;
            while (iss >> name) {
                PcdField field;
                field.name = name;
                header.fields.emplace_back(field);
            }
        } else if (key == "SIZE") {
            uint32_t value = 0;
            for (size_t i = 0; i < header.fields.size() && iss >> value; ++i) {
                header.fields[i].size = value;
            }
        } else if (key == "TYPE") {
            char type = 0;
            for (size_t i = 0; i < header.fields.size() && iss >> type; ++i) {
                header.fields[i].type = type;
            }
        } else if (key == "COUNT") {
            uint32_t value = 0;
            for (size_t i = 0; i < header.fields.size() && iss >> value; ++i) {
                header.fields[i].count = value;
            }
        } else if (key == "WIDTH") {
            std::string value;
            if (iss >> value) {
                ParseUnsigned(value, header.width);
            }
        } else if (key == "HEIGHT") {
            std::string value;
            if (iss >> value) {
                ParseUnsigned(value, header.height);
            }
        } else if (key == "POINTS") {
            std::string value;
            if (iss >> value) {
                ParseUnsigned(value, header.points);
            }
        } else if (key == "DATA") {
            iss >> header.data_type;
            data_offset = pos;  // First byte right after the "DATA ..." line.
            return true;
        }
        // VERSION and VIEWPOINT lines are ignored.
    }
    return false;
}

/* Read one ascii token from a byte buffer, skipping any whitespace. */
bool ReadAsciiToken(const std::vector<uint8_t> &buffer, size_t &pos, std::string &token) {
    while (pos < buffer.size() && std::isspace(buffer[pos])) {
        ++pos;
    }
    if (pos >= buffer.size()) {
        return false;
    }
    token.clear();
    while (pos < buffer.size() && !std::isspace(buffer[pos])) {
        token.push_back(static_cast<char>(buffer[pos]));
        ++pos;
    }
    return true;
}

/* Parse an ascii field value into a float. */
float ParseAsciiFieldValue(const std::string &token, const PcdField &field) {
    if (field.type == 'I') {
        return static_cast<float>(std::strtoll(token.c_str(), nullptr, 10));
    }
    if (field.type == 'U') {
        return static_cast<float>(std::strtoull(token.c_str(), nullptr, 10));
    }
    return static_cast<float>(std::strtod(token.c_str(), nullptr));
}

/* Read one field value from its raw binary representation (little endian). */
float ReadBinaryFieldValue(const uint8_t *ptr, const PcdField &field) {
    if (field.type == 'F') {
        if (field.size == 4) {
            float value = 0.0f;
            std::memcpy(&value, ptr, 4);
            return value;
        }
        if (field.size == 8) {
            double value = 0.0;
            std::memcpy(&value, ptr, 8);
            return static_cast<float>(value);
        }
    } else if (field.type == 'I') {
        if (field.size == 1) {
            int8_t value = 0;
            std::memcpy(&value, ptr, 1);
            return static_cast<float>(value);
        }
        if (field.size == 2) {
            int16_t value = 0;
            std::memcpy(&value, ptr, 2);
            return static_cast<float>(value);
        }
        if (field.size == 4) {
            int32_t value = 0;
            std::memcpy(&value, ptr, 4);
            return static_cast<float>(value);
        }
        if (field.size == 8) {
            int64_t value = 0;
            std::memcpy(&value, ptr, 8);
            return static_cast<float>(value);
        }
    } else if (field.type == 'U') {
        if (field.size == 1) {
            uint8_t value = 0;
            std::memcpy(&value, ptr, 1);
            return static_cast<float>(value);
        }
        if (field.size == 2) {
            uint16_t value = 0;
            std::memcpy(&value, ptr, 2);
            return static_cast<float>(value);
        }
        if (field.size == 4) {
            uint32_t value = 0;
            std::memcpy(&value, ptr, 4);
            return static_cast<float>(value);
        }
        if (field.size == 8) {
            uint64_t value = 0;
            std::memcpy(&value, ptr, 8);
            return static_cast<float>(value);
        }
    }
    return 0.0f;
}

}  // namespace

bool Lidar::ConvertPcdFileToPoints(const std::string &pcd_file, std::vector<Vec3> &points) {
    points.clear();

    // Load the whole file into memory so that all three data layouts
    // (ascii / binary / binary_compressed) can share one byte buffer.
    std::ifstream file(pcd_file, std::ios::binary);
    if (!file.is_open()) {
        ReportError("Failed to open pcd file: " << pcd_file);
        return false;
    }
    std::vector<uint8_t> buffer;
    file.seekg(0, std::ios::end);
    const std::streamoff file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    if (file_size > 0) {
        buffer.resize(static_cast<size_t>(file_size));
        file.read(reinterpret_cast<char *>(buffer.data()), file_size);
    }
    if (buffer.empty()) {
        ReportError("Empty pcd file: " << pcd_file);
        return false;
    }

    size_t data_offset = 0;
    PcdHeader header;
    if (!ParsePcdHeader(buffer, data_offset, header)) {
        ReportError("Failed to parse pcd header of file: " << pcd_file);
        return false;
    }

    // Number of points. The POINTS field may be omitted in older files, in
    // which case it equals width * height.
    if (header.points == 0) {
        header.points = header.width * header.height;
    }
    if (header.points == 0 || header.fields.empty()) {
        ReportError("Invalid point count or empty fields in pcd file: " << pcd_file);
        return false;
    }

    // Locate the x / y / z fields.
    int32_t x_index = -1;
    int32_t y_index = -1;
    int32_t z_index = -1;
    for (size_t i = 0; i < header.fields.size(); ++i) {
        if (header.fields[i].name == "x") {
            x_index = static_cast<int32_t>(i);
        } else if (header.fields[i].name == "y") {
            y_index = static_cast<int32_t>(i);
        } else if (header.fields[i].name == "z") {
            z_index = static_cast<int32_t>(i);
        }
    }
    if (x_index < 0 || y_index < 0 || z_index < 0) {
        ReportError("Pcd file does not contain x/y/z fields: " << pcd_file);
        return false;
    }

    // Byte offset of every field inside one point record.
    std::vector<uint32_t> field_offsets(header.fields.size(), 0);
    uint32_t point_step = 0;
    for (size_t i = 0; i < header.fields.size(); ++i) {
        field_offsets[i] = point_step;
        point_step += header.fields[i].size * header.fields[i].count;
    }

    points.reserve(header.points);

    if (header.data_type == "ascii") {
        // Points are space separated text, one point per line in the standard
        // layout. Parse the remainder as a flat token stream instead so that
        // arbitrary whitespace / line breaks are tolerated.
        size_t pos = data_offset;
        std::string token;
        std::vector<std::string> point_tokens(header.fields.size());
        for (uint32_t i = 0; i < header.points; ++i) {
            for (size_t j = 0; j < header.fields.size(); ++j) {
                if (!ReadAsciiToken(buffer, pos, token)) {
                    ReportError("Not enough ascii data in pcd file: " << pcd_file);
                    return false;
                }
                point_tokens[j] = token;
            }
            const float x = ParseAsciiFieldValue(point_tokens[x_index], header.fields[x_index]);
            const float y = ParseAsciiFieldValue(point_tokens[y_index], header.fields[y_index]);
            const float z = ParseAsciiFieldValue(point_tokens[z_index], header.fields[z_index]);
            points.emplace_back(x, y, z);
        }
    } else if (header.data_type == "binary" || header.data_type == "binary_compressed") {
        // Data layout after the header line:
        //   binary:            [raw point records]
        //   binary_compressed: [compressed_size u32][uncompressed_size u32][lzf stream]
        const uint8_t *base = buffer.data() + data_offset;
        uint64_t available_bytes = buffer.size() - data_offset;
        if (header.data_type == "binary_compressed") {
            if (data_offset + 8 > buffer.size()) {
                ReportError("Truncated compressed data in pcd file: " << pcd_file);
                return false;
            }
            uint32_t compressed_size = 0;
            uint32_t uncompressed_size = 0;
            std::memcpy(&compressed_size, buffer.data() + data_offset, 4);
            std::memcpy(&uncompressed_size, buffer.data() + data_offset + 4, 4);
            if (data_offset + 8 + compressed_size > buffer.size()) {
                ReportError("Truncated compressed data in pcd file: " << pcd_file);
                return false;
            }
            std::vector<uint8_t> uncompressed(uncompressed_size);
            const uint32_t written = LzfDecompress(buffer.data() + data_offset + 8, compressed_size,
                                                   uncompressed.data(), uncompressed_size);
            if (written != uncompressed_size) {
                ReportError("Failed to decompress pcd file: " << pcd_file);
                return false;
            }
            base = uncompressed.data();
            available_bytes = uncompressed.size();
        }

        const uint64_t expected_bytes = static_cast<uint64_t>(header.points) * point_step;
        if (available_bytes < expected_bytes) {
            ReportError("Point data is shorter than expected in pcd file: " << pcd_file);
            return false;
        }

        for (uint32_t i = 0; i < header.points; ++i) {
            const uint8_t *record = base + static_cast<uint64_t>(i) * point_step;
            const float x = ReadBinaryFieldValue(record + field_offsets[x_index], header.fields[x_index]);
            const float y = ReadBinaryFieldValue(record + field_offsets[y_index], header.fields[y_index]);
            const float z = ReadBinaryFieldValue(record + field_offsets[z_index], header.fields[z_index]);
            points.emplace_back(x, y, z);
        }
    } else {
        ReportError("Unsupported pcd data type: " << header.data_type);
        return false;
    }

    return true;
}

}  // namespace sensor_model
