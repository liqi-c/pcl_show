#!/bin/bash 

mkdir build
cd build
cmake ..
make -j4

##./bin/test  /home/liq/CODE/ori_pcl/pcl_0002_2022-07-12_10-04-26_756.pcd
##./bin/test  /home/liq/CODE/ori_pcl/pcl_0002_2022-07-12_10-04-26_756.pcd /home/liq/CODE/ori_pcl/pcl_0002_2022-07-12_10-04-26_756.pcd

##./bin/test /home/liq/CODE/ori_pcl/
##./bin/test  /home/liq/CODE/ori_pcl/  /home/liq/CODE/hdl_pcl/
