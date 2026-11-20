[TOC] 

本项目基于ros2实现多机规划以及调度功能。

# 编译

### 基础依赖包下载、编译与安装,包括数据结构，可视化组件等等
```
git clone git@github.com:JoeYao-bit/freeNav-base.git
```
```
cd freeNav-base
```
```
mkdir build
```
```
cd build
```
```
cmake ..
```
```
make 
```
```
sudo make install
```
### 下载源代码至工作空间源代码目录

例如
```
cd ros2_ws/src/
```
下载自定义消息包
```
git clone git@github.com:JoeYao-bit/lamapf_and_gazebo_msgs.git
```
下载多机规划及调度包
```
git clone git@github.com:JoeYao-bit/lamapf_and_gazebo.git
```
切换到专为小直开发的简化分支
```
git checkout xiaozhi261120
```
### ros2包编译

进入工作空间src目录，例如ros2_ws/src/

```
colcon build
```

更新launch后需要重新运行 colcon build 以更新可执行文件



# 测试
启动可视化单元测试
包括路径规划、中央控制器调度、以及模拟动作执行
```
ros2 run lamapf_and_gazebo test_lamapf_planner_node
```