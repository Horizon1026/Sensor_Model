# Sensor_Model
The model of sensors for slam, for example, imu, camera, and so on.

# Components
- [x] Camera model
- [x] Imu model
- [x] Wheel odom model
- [ ] GPS model
- [ ] Lidar model

# Dependence
- Slam_Utility
- Feature_Detector
- Visualizor2D (only for test)

# Compile and Run
- 第三方仓库的话需要自行 apt-get install 安装
- 拉取 Dependence 中的源码，在当前 repo 中创建 build 文件夹，执行标准 cmake 过程即可
```bash
mkdir build
cmake ..
make -j
```
- 编译成功的可执行文件就在 build 中，具体有哪些可执行文件可参考 run.sh 中的列举。可以直接运行 run.sh 来依次执行所有可执行文件

```bash
sh run.sh
```

# Tips
- 欢迎一起交流学习，不同意商用；
