[TOC] 


# 最新关键操作指令

## 1,配置USB连接规则

查看所有连接的usb设备
lsusb


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

sudo nano /etc/udev/rules.d/99-robot-usb.rules

填入以下内容：

需要给每个机器人都配置一遍

# TurtleBot Kobuki base (FTDI USB)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="kobuki"

# RPLIDAR (Silicon Labs CP210x)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"


2️⃣ 重新加载规则并触发


sudo udevadm control --reload-rules
sudo udevadm trigger

然后拔掉再插上两个设备，检查：

ls -l /dev/kobuki /dev/rplidar


这样我一个usb口连turtlebot机器人，一个连rplidar，如何自动识别匹配usb号
让系统自动识别并固定每个设备（即使拔插顺序改变，也不会错乱）。

## 2, 启动激光雷达
ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_port:=/dev/rplidar serial_baudrate:=115200

## 3, 启动连接turtlebot2
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

## 6, 发布雷达到机器人底盘的静态transform
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint laser

## 7, tf树可视化
ros2 run tf2_tools view_frames 
应该是map->odom->base_footprint->laser

## 8, 保存地图

安装地图服务器

sudo apt install ros-jazzy-nav2-map-server

ros2 run nav2_map_server map_saver_cli -f ~/my_map

## 9, 启动定位
改变yaml参数文件地址
ros2 launch lamapf_and_gazebo turtlebot2_amcl_localization.launch.py use_sim_time:=false

## 10, 发布初始位置
改init_pose_publisher.py中的文件路径
python3 /home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/script/initial_pose_publisher.py

## 11, 初始位置不对则输入rviz2设置初始位置


# 全部安装指令

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