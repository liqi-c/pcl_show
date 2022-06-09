### 文件目录：
```
.
├── CMakeLists.txt
├── include
│   ├── camera_driver
│   │   ├── camera_driver_api.h
│   │   ├── camera_driver.h
│   │   └── camera_driver_interface.h
│   ├── depth_image_to_pointcloud
│   │   ├── depth_image_to_pointcloud_api.h
│   │   ├── depth_image_to_pointcloud.h
│   │   └── depth_image_to_pointcloud_interface.h
│   ├── isca_tof_camera.h
│   ├── tof_camera.h
│   └── tof_camera_interface.h
└── src
    ├── camera_driver
    │   ├── camera_driver_api.cpp
    │   └── camera_driver.cpp
    ├── depth_image_to_pointcloud
    │   ├── depth_image_to_pointcloud_api.cpp
    │   └── depth_image_to_pointcloud.cpp
    ├── isca_tof_camera.cpp
    ├── main.cpp
    └── tof_camera.cpp

```

### 编译前说明：

需要正确设置CMakeLists.txt里面交叉编译工具以及OPENNI链接库的位置:
```
set(CMAKE_C_COMPILER "/usr/local/ARM-toolchain/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/usr/local/ARM-toolchain/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++")
set(OPENNI2_INCLUDE "$ENV{HOME}/AXonOpenNI-Linux-Arm-2.3/Include")
set(OPENNI2_REDIST "$ENV{HOME}/AXonOpenNI-Linux-Arm-2.3/Redist")
```


### 编译说明：
```
mkdir build
cd build
cmake ..
make
```

### 修改说明
v3.2 20220602
更新部分参数

V3.1 20220520
注释相机驱动部分

v3.0 20220519
加入舜宇点云转换为isca::PointCloud示例代码；
加入输入原始点云输出处理后的点云的接口；

v2.1 20220421
增加点云的显示

v2.0 20220402
加入降采样的参数；
采用for代替条件滤波器与直通滤波器，以提高运算速度；
开放整个编译使用的工程；

v1.0 20220401
只开放点云处理的源码；