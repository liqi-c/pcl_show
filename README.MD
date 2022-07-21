# PCL 显示工具(pcd) 
## 文件目录：
```
.
├── bin
│   ├── pcl_show
│   └── pcl_show_bin
├── build
├── CMakeLists.txt
├── README.MD
├── src
│   ├── main.cpp
│   ├── pcl_show.cpp
│   └── pcl_show.hpp
└── Test.sh
```
## 依赖说明：
#### 编译运行环境 
    Ubuntu 20.04 
```
$ cat /proc/version
Linux version 5.13.0-51-generic (buildd@lcy02-amd64-046) (gcc (Ubuntu 9.4.0-1ubuntu1~20.04.1) 9.4.0, GNU ld (GNU Binutils for Ubuntu) 2.34) #58~20.04.1-Ubuntu SMP Tue Jun 14 11:29:12 UTC 2022

~/pcl_show$ lsb_release -a
No LSB modules are available.
Distributor ID: Ubuntu
Description:    Ubuntu 20.04.3 LTS
Release:        20.04
Codename:       focal
```
#### 安装依赖
    官方系统源安装可能会失败，建议切换到清华源. 
```
    sudo apt-get install libusb-1.0-0-dev
    sudo apt-get install libeigen3-dev
    sudo apt-get install libflann-dev 
    sudo apt install libvtk6-dev 
    sudo apt-get install libflann1.9
    sudo apt-get install  libpcap-dev 
    sudo apt-get install libboost-all-dev
    sudo apt-get install qt5-default 
```
#### 编译PCL
  **源码下载** ：https://github.com/PointCloudLibrary/pcl/releases  
  **官方编译指导**: https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html 
```
    cd pcl-pcl-1.12.0 
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j2
    sudo make -j2 install
```
## 编译工程说明：
```
mkdir build
cd build
cmake ..
make
```
## 测试说明 
**示例工程支持4种输入：** 
1. 输入单个pcd文件 .
2. 同时输入两个pcd文件，两个文件会并排显示. 
3. 输入单个文件目录，程序会自动遍历该目录并显示所有的pcd文件. 
4. 输入两个文件目录，程序会自动编译两个目录，并并排显示两个文件. 
```
~$ ./pcl_show_bin 
usage: 
     1 . ./pcl_show_bin </filepath/*.pcd>                   -- show signle pcd file. 
     2 . ./pcl_show_bin </filepath/*.pcd> </filepath/*.pcd> -- show double pcd file. 
     3 . ./pcl_show_bin <pcd_file_dir>                      -- show continue signle pcd file. 
     4 . ./pcl_show_bin <pcd_file_dir>    <pcd_file_dir>    -- show continue double pcd file. 
```