# ctlo
CTLO: Continuous-Time LiDAR Odometry

**CTLO** (Continuous-Time LiDAR Odometry) is an accurate and robust LiDAR odometry (LIO). 

- [Video-Bilibili](https://www.bilibili.com/video/BV1gu411w77z/?spm_id_from=333.999.0.0&vd_source=438f630fe29bd5049b24c7f05b1bcaa3)
  
<div align="center">
    <img src="./point-lio.gif" width=80% />
</div>


## NOTE: Click on the image to view the corresponding video.



### Some test results are show below:

#### MID-360, multi-layer 

<div align="center">
    <a href="https://www.bilibili.com/video/BV1gu411w77z/?spm_id_from=333.999.0.0&vd_source=438f630fe29bd5049b24c7f05b1bcaa3" target="_blank">
    <img src="./m-stair.gif" width=80% />
</div>

#### AVIA, long corridor

<div align="center">
    <a href="https://www.bilibili.com/video/BV1gu411w77z/?spm_id_from=333.999.0.0&vd_source=438f630fe29bd5049b24c7f05b1bcaa3" target="_blank">
    <img src="./vm++.gif" width=80% />
</div>




## 1. Prerequisites

### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 18.04**

For **Ubuntu 18.04 or higher**, the **default** PCL and Eigen is enough for ct_lio to work normally.

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).


## 2. Build

Clone the repository and catkin_make:

**NOTE**:**[This is import]** before catkin_make, make sure your dependency is right(you can change in ./cmake/packages.cmake)

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/chengwei0427/CTLO.git
    cd CTLO
    cd ../..
    catkin_make
    source devel/setup.bash
```

- If you want to use a custom build of PCL, add the following line to ~/.bashrc
```export PCL_ROOT={CUSTOM_PCL_PATH}```
  
## 3. Directly run

**Noted:**

**A**. Please make sure the Lidar has 'time'

## 4. Rosbag Example
Mid-360 testdata can be downloaded from [Baidu Pan(password:a6u3)](https://pan.baidu.com/s/1Jg0lOT_FQt-jE0o4d0pOlg)

## Related Works
1. [ct_icp](https://github.com/jedeschaud/ct_icp):  Continuous-Time LiDAR Odometry .
2. [ct_lio](https://github.com/chengwei0427/ct_lio): Continuous-Time Lidar-Inertial Odometry. 
