[TOC] 



-

获取设备唯一ID
机器人
 udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' -m 3
    SUBSYSTEMS=="usb-serial"
    ATTRS{idProduct}=="6001"
    ATTRS{idVendor}=="0403"

激光雷达  
 udevadm info -a -n /dev/ttyUSB1 | grep -E 'idVendor|idProduct|serial' -m 3
    SUBSYSTEMS=="usb-serial"
    ATTRS{idProduct}=="ea60"
    ATTRS{idVendor}=="10c4"

配置规则文件

执行：

sudo gedit /etc/udev/rules.d/99-robot-usb.rules

填入以下内容：

需要给每个机器人都配置一遍

# TurtleBot Kobuki base (FTDI USB)
KERNEL=="ttyUSB*", SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="kobuki", MODE:="0666"


# RPLIDAR (Silicon Labs CP210x)
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="rplidar", MODE:="0666"

2️⃣ 重新加载规则并触发


sudo udevadm control --reload-rules
sudo udevadm trigger

然后拔掉再插上两个设备，检查：

ls -l /dev/kobuki /dev/rplidar


这样我一个usb口连turtlebot机器人，一个连rplidar，如何自动识别匹配usb号
让系统自动识别并固定每个设备（即使拔插顺序改变，也不会错乱）。


# 全部安装指令

安装gedit

sudo apt install gedit

0, 安装anydesk, 配置远程桌面无人值守访问

禁用Wayland，anydesk不支持wayland

sudo gedit /etc/gdm3/custom.conf

取消注释 WaylandEnable=False

sudo systemctl restart gdm3

安装 ecl_build

sudo apt install ros-jazzy-ecl-build

安装 sophus

sudo apt install ros-jazzy-sophus

或者

git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake ..
make -j
sudo make install

这样的话，ecl_build，ecl_core/ecl_linear_algebra(下面三个cmakelist都要加)，ecl_geometry, ecl_statistics

cmakelist添加以下代码使用C++ 17
## 使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


* 安装科学上网

https://hiddify.zip/ 下载hiddify

chmod +x hiddify-linux-x64.AppImage

解决“指定的像素格式没有可用的设置”
./Hiddify-Linux-x64.AppImage --appimage-extract



安装colcon，ros2编译工具
sudo apt install colcon


1, 安装kobuki相关的包
ecl_core,ecl_lite,kobuki_core,kobuki_ros,kobuki_ros_interfaces

git clone https://github.com/stonier/ecl_lite.git

git clone https://github.com/stonier/ecl_core.git

git clone https://github.com/kobuki-base/kobuki_ros.git

git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git

git clone https://github.com/kobuki-base/kobuki_core.git


ROS2 Jazzy 使用的 GCC/Clang 默认把 所有警告当作错误 (-Werror)，

所以即便只是警告，也会直接导致构建失败。
在编译时禁止将警告当作错误
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-Wno-error=overloaded-virtual"


2, 安装 ROS 2 版 RPLIDAR 驱动包
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

安装 ROS 2 版 RPLIDAR 驱动包
git clone -b ros2 git@github.com:Slamtec/rplidar_ros.git

单独编译某个包
colcon build --packages-select rplidar_ros

安装ros2建图工具包

sudo apt install ros-jazzy-slam-toolbox

安装tf_transformations

sudo apt install ros-jazzy-tf-transformations

安装yaml相关依赖

sudo apt install libyaml-cpp-dev

# 最新关键操作指令

## 1,配置USB连接规则

查看所有连接的usb设备

lsusb


## 2, 启动激光雷达
ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_port:=/dev/rplidar serial_baudrate:=115200

## 3, 启动连接turtlebot2

配置 kobuki_ros/kobuki_node/config/kobuki_node_params.yaml 中 device_port 为/dev/kobuki

然后更新参数 colcon build --packages-select kobuki_node

ros2 launch kobuki_node kobuki_node-launch.py serial_port:=/dev/kobuki serial_baudrate:=115200

需要root 和 dialout 用户组可以访问

你的普通用户不在 dialout 组，所以会报 permission denied

添加用户到dialout组

sudo usermod -aG dialout $USER


完全退出你的用户账号（GUI 或 SSH）。

重新登录。

再执行：

groups

你会看到 dialout 已经在列表里。

## 4, 键盘控制移动
ros2 run kobuki_keyop kobuki_keyop_node --ros-args -r cmd_vel:=/commands/velocity


## 5, 启动建图
改变yaml参数文件地址
ros2 launch lamapf_and_gazebo   turtlebot2_online_async_launch.py

## 6, 发布雷达到机器人底盘的静态transform(激光雷达方向朝正后，而不是朝正前)

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 1 0 base_footprint laser

## 7, tf树可视化

ros2 run tf2_tools view_frames 

应该是map->odom->base_footprint->laser

## 8, 保存地图

安装地图服务器


sudo apt install ros-jazzy-nav2-map-server \
                 ros-jazzy-nav2-behavior-tree \
                 ros-jazzy-nav2-planner \
                 ros-jazzy-nav2-amcl \
                 ros-jazzy-nav2-lifecycle-manager \
                 ros-jazzy-nav2-map-server 


ros2 run nav2_map_server map_saver_cli -f ~/my_map

## 9, 启动定位


改变yaml参数文件地址到当前文件地址

启动rviz
输入rviz2

ros2 launch lamapf_and_gazebo turtlebot2_amcl_localization.launch.py use_sim_time:=false

在turtlebot2_amcl_localization.launch.py和initial_pose_publisher.py中设置地图名称

初始位置在amcl_localization.yaml中设置

第一次启动launch如果rviz2收不到地图，那就关了定位再开一次

初始位置不对则输入rviz2设置初始位置

整合各个启动项：

加载激光雷达，连接turtlebot，发布静态transform，启动定位，启动rviz

ros2 launch lamapf_and_gazebo localization_full.launch.py

## 10, 启动局部控制器测试，并导出日志到指定文件

ros2 launch lamapf_and_gazebo local_controller_test.launch.py >  my_node.log 2>&1

加载激光雷达，连接turtlebot，发布静态transform，启动定位，启动rviz

## 11, 启动中央控制器，并导出日志到指定文件

ros2 launch lamapf_and_gazebo center_controller_test.launch.py >  my_node.log 2>&1

增加名称空间后的速度话题变了，因此需要更改速度发布话题：
新的键盘控制指令

ros2 run kobuki_keyop kobuki_keyop_node --ros-args -r cmd_vel:=/robot0/commands/velocity

# 大规模安装

## 1, 通过upan安装ubuntu 24.04

安装anydesk, 配置远程桌面无人值守访问

禁用Wayland，anydesk不支持wayland

sudo gedit /etc/gdm3/custom.conf

取消注释 WaylandEnable=False

然后重启gdm3
sudo systemctl restart gdm3

## 2, 安装ros2
sudo apt install software-properties-common

sudo add-apt-repository universe

sudo apt update

sudo apt install curl gnupg lsb-release -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update  # 更新包管理器的本地软件包索引

sudo apt upgrade -y # 根据 apt update 更新的软件包列表，查找当前安装包是否有新版本，并升级到最新版本

sudo apt install ros-jazzy-desktop -y  # 安装 ROS 2 Jazzy 的桌面版本，包含可视化工具 RViz 等

sudo apt install ros-jazzy-ros-base -y

最常见版本：

    ros-jazzy-desktop-full：完整安装，包括所有开发工具和图形界面。
    ros-jazzy-desktop：较为简化的桌面版本，适合大多数桌面开发。
    ros-jazzy-ros-base：基本的安装版本，只包含核心功能，适合基础开发。


. 配置环境变量

每次打开终端时，必须设置 ROS2 的环境变量。可以通过以下命令手动设置：

source /opt/ros/jazzy/setup.bash

为了避免每次都要手动设置，可以将此命令添加到 ~/.bashrc 文件中：

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

(找不到ros-jazzy试试下面的方法)

# 方案1：清华源+跳过SSL验证（最可能成功）
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /etc/apt/apt.conf.d/99-* 2>/dev/null

echo 'Acquire::https::Verify-Peer "false";' | sudo tee /etc/apt/apt.conf.d/99-no-ssl
echo 'Acquire::https::Verify-Host "false";' | sudo tee -a /etc/apt/apt.conf.d/99-no-ssl

echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

# 查看是否成功
apt-cache search ros-jazzy 2>/dev/null | head -5

# 方案1：清华源+跳过SSL验证（最可能成功）
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /etc/apt/apt.conf.d/99-* 2>/dev/null

echo 'Acquire::https::Verify-Peer "false";' | sudo tee /etc/apt/apt.conf.d/99-no-ssl
echo 'Acquire::https::Verify-Host "false";' | sudo tee -a /etc/apt/apt.conf.d/99-no-ssl

echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

## bashrc

source /opt/ros/jazzy/setup.bash
source /home/wangweilab/ros2_ws/install/setup.bash

# 查看是否成功
apt-cache search ros-jazzy 2>/dev/null | head -5

## 3，安装opencv
sudo apt update
sudo apt install libopencv-dev

## 4, 安装ros_gz_interfaces
sudo apt-get install ros-jazzy-ros-gz-interfaces

## 5, 安装tf2
sudo apt-get install ros-jazzy-tf2

## 6, 安装std-msgs, sensor msgs
sudo apt-get install ros-jazzy-std-msgs
sudo apt-get install ros-jazzy-sensor-msgs


## 7, 安装eigen
sudo apt-get install libeigen3-dev

## SuiteSparse
sudo apt-get install libsuitesparse-dev

## 8, 安装libxml2
sudo apt-get install libxml2-dev


## * 安装科学上网

https://hiddify.zip/ 下载hiddify

chmod +x hiddify-linux-x64.AppImage

## 安装fuse2
sudo apt install libfuse2


## 安装tinyxml

sudo apt-get install libtinyxml-dev

## 安装CLI11库

sudo apt-get install libcli11-dev

## 安装git

sudo apt install git

## ssh初始化

ssh-keygen -t rsa -C "1521232476@qq.com"

然后复制pub key添加的到github的ssh中，使得可以clone代码


## 安装 python3
sudo apt install python3-pip

sudo apt install python3-wheel

## 安装 glut
sudo apt-get install freeglut3-dev

## 安装 g2o 

sudo apt-get install libg2o-dev

// 如果需要额外的依赖
sudo apt-get install libqglviewer-dev-qt5
sudo apt-get install libsuitesparse-dev

sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3

git clone git@github.com:RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ..
make -j4
sudo make install


## 安装epoxy
sudo apt install -y libepoxy-dev

## 安装Pangolin

git clone git@github.com:stevenlovegrove/Pangolin.git

sudo apt-get install libglew-dev
sudo apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev
cd Pangolin
mkdir build
cd build
cmake ..
make -j2
sudo make install

## 安装 Qt5 Charts
sudo apt-get install libqt5charts5-dev

## 9, 安装LayeredMAPF
git clone git@github.com:JoeYao-bit/LayeredMAPF.git

git clone --depth=1 git@github.com:JoeYao-bit/LayeredMAPF.git
(只要最新版本，节约时间)

freeNav-base的cmakelist中注释libkahypar



build通过并make install

## 更新子仓库
git submodule update --init

## 安装octomap

sudo apt-get install liboctomap-dev

或

git clone git@github.com:OctoMap/octomap.git
cd octomap/octomap
mkdir build
cd build
cmake ..
make
sudo make install

参考LayeredMAPF的readme安装依赖

## 安装 argparse

sudo apt install libargparse-dev

或

git clone git@github.com:p-ranav/argparse.git
cd argparse
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

更新头文件缓存（如果需要）
sudo ldconfig

## 安装epoxy
sudo apt install -y libepoxy-dev

## 安装diagnostic-updater
sudo apt install ros-jazzy-diagnostic-updater

代码安装 

git clone git@github.com:ros/diagnostics.git

放到ros2_ws/src目录下

将ecl等依赖包解压到ros2_ws/src目录下

安装lamapf_and_gazebo

git clone git@github.com:JoeYao-bit/lamapf_and_gazebo.git

安装lamapf_and_gazebo_msgs

git clone git@github.com:JoeYao-bit/lamapf_and_gazebo_msgs.git

