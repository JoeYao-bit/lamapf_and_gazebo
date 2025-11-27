[TOC] 

更新launch后需要重新运行 colcon build 以更新位于install的launch复制文件，单独更新src节点内的launch无效

运行下述代码启动gazebo以及相关world文件

```
ros2 launch large_agent_mapf gazebo_launch.py 
```

LA-MAPF 和 freeNav-base的头文件安装在/usr/local/include目录，分别来自freeNav和LayeredMAPF

启动planner： 

```
ros2 launch large_agent_mapf lamapf_planner.py
```

libcanvas_ex.a liblamapf_alg_ex.a 分别是freeNav和LayeredMAPF编译得到的静态库，安装在/usr/local/lib

11-11：同样的输入参数和代码，运行速度比在Clion慢了好几倍，不知道为什么

colcon build --cmake-args -DCMAKE_CXX_FLAGS="-O3"

通过设置编译时O3优化，运行速度大大提高了

启动rviz： ros2 run rviz2 rviz2

启动fake agent：
```
ros2 run large_agent_mapf fake_agents_node
```

使用ROS2 命令行生成一个机器人（运行成功）
ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' '{name: "sdf_ball", xml: "<?xml version=\"1.0\" ?><sdf version=\"1.5\"><model name=\"will_be_ignored\"><static>true</static><link name=\"link\"><visual name=\"visual\"><geometry><sphere><radius>1.0</radius></sphere></geometry></visual></link></model></sdf>"}'

通过代码构造SDF，并存放在/tmp目录下

[gazebo-1] Error [Param.cc:449] Invalid argument. Unable to set value [{radius} ] for key[radius].
[gazebo-1] Error [parser_urdf.cc:3267] Unable to call parseURDF on robot model
[gazebo-1] Error [parser.cc:488] parse as old deprecated model file failed.
[gazebo-1] Error Code 8 Msg: Error reading element <radius>
[gazebo-1] Error Code 8 Msg: Error reading element <cylinder>
[gazebo-1] Error Code 8 Msg: Error reading element <geometry>
[gazebo-1] Error Code 8 Msg: Error reading element <collision>
[gazebo-1] Error Code 8 Msg: Error reading element <link>
[gazebo-1] Error Code 8 Msg: Error reading element <model>
[gazebo-1] Error Code 8 Msg: Error reading element <sdf>


<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="cylinder_robot">
    <link name="base_link">
      <pose>0 0 0.5 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>2.300000</radius>
            <length>1.000000</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>

    // 替换模型名称、半径和高度 需要多次进行，每一个都设置一遍

gazebo world文件地址：/home/yaozhuo/software/gazebo/warehouse.world


    <?xml version ?>  xml的版本
    <sdf version>  sdf的版本，和config里<sdf>的版本要一样呀
    <model name> 模型的名字
    <pose> 在世界中的位置 x y z pitch yaw roll
    <static> 选择模型是否固定
    <link>链接 包含模型的一个主体的物理属性，尽量减少模型中链接数量以提高性能和稳定
    <collision>: 用于碰撞检查，一个link可以有多个碰撞元素
    <geometry> 物体
    <box> | <sphere> | <cylinder>形状名字
    <size> x y z长度 | <radius>半径 | <radius> & <length>
    <surface> 平面
    <friction>设置地面摩擦力
    <ode> <mu> <slip>
    <visual>: 可视化
    <geometry> 几何形状
    <box>形状名字
    <size> x y z长度
    <inertial>: 惯性元素，描述了link的动态特性，例如质量和转动惯量矩阵
    <mass> 质量
    <inertia> ！！！注意这两单词不一样呀
    <sensor>: 从world收集数据用于plugin
    <light>: 光源
    <joint>关节 关节连接两个link，用于旋转轴和关节限制等
    <plugin>插件  用于控制模型

删除collision可以忽略物理模型

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius1}</radius>
            <length>{height1}</length>
          </cylinder>
        </geometry>
      </collision>

这是一个自问自答的问题记录。

在使用ros2 control CLI时遇到的RTPS报错，提示无法连接服务。

$ ros2 control list_hardware_interfaces
2022-03-27 12:15:30.282 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7412: open_and_lock_file failed -> Function open_port_internal
Could not contact service /controller_manager/list_hardware_interfaces

造成该问题的原因是找不到和这个服务通信的数据类型。

可以采用下面的命令安装下，重新运行即可。其中foxy可以替换为你自己的ros2版本。
sudo apt install ros-foxy-controller-manager*

/----------------------------------------------/

libgazebo_ros_state（set entity state服务需要），不是系统插件，需要在.world中注册


<sdf version='1.7'>
  <world name='default'>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
        <argument>model_states:=model_states</argument>
      </ros>
      <update_rate>1.0</update_rate>
    </plugin>
...

参考链接： https://robotics.stackexchange.com/questions/96506/how-to-use-gazebo-plugins-found-in-gazebo-ros-ros2-foxy-gazebo11

修改之后仍然报错，如下，但可以通过set_entity_state服务设置机器人位姿了
[gazebo-1] [Err] [gazebo_shared.cc:46] System is attempting to load a plugin, but detected an incorrect plugin type. Plugin filename[libgazebo_ros_state.so].

/usr/local/inlcude，/usr/local/lib 本地安装的头文件和库地址

/usr/inlcude，/usr/lib 下载安装的头文件和库地址

TODO： 创建一个服务，如果已经规划完路径，则从头开始演示执行过程

/-----------自定义服务------------/

https://blog.csdn.net/oXiaoLingTong/article/details/120594951

 ros2中对自定义消息、服务的名字有要求，首字母必须大写，中介不能有下划线等。

 生成的头文件名字小写间隔加'_'

 PathExecution.src -> path_execution.hpp

 colcon build 才生成hpp
 colcon build --cmake-args -DCMAKE_CXX_FLAGS="-O3"
好像不生成hpp

/--------------------单次发布指令，触发回调函数---------------------/
ros2 topic pub --once /init_exe std_msgs/msg/String "{data: haha}"

/------------------------------------------------------/

新增模型， Utility Cart (较大) / Turtlebot / Warehouse Robot / Kuka Youbot,来自http:/models.gazebosim.org, 
open-rmf/DeliveryRobotwithconveyor, 来自http:/models.gazebosim.org, 


gazebo如何删除添加的model path
在～/.gazebo/gui.ini文件中删除后，重新打开gazebo即可。

sdf文件中的名字和模型在gazebo的名字需要保持一致


反复开启关闭gazebo后，出现gazebo显示正在运行，被占用的情况
２　解决方法：
killall gzserver
killall gzclient


3D gazebo模型网址

https://data.nvision2.eecs.yorku.ca/3DGEMS/


gazebo中红色x轴，蓝色z轴，绿色y轴
通过gazebo的Camera->Orthographic可以获得平行投影俯视图，作为投影依据

map_large_office.png <=> office_env_large.world 

GUI->Grid 设置栅格大小，默认20，一格一米


报错
2025-04-12 05:24:50.913 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7425: open_and_lock_file failed -> Function open_port_internal
https://blog.csdn.net/qq_27865227/article/details/127340287
可以采用下面的命令安装下，重新运行即可。其中foxy可以替换为你自己的ros2版本。
sudo apt install ros-foxy-controller-manager*
造成该问题的原因是找不到和这个服务通信的数据类型。

构造虚拟instance,单独运行以下代码即可

```
ros2 run large_agent_mapf generate_instance_node
```

暂停仿真后保存：

    在保存世界前，点击 Gazebo 界面上的 暂停按钮（⏸️），确保模型处于静止状态。

清除动态数据：

    在保存前运行以下命令（通过 Gazebo 的终端或 ROS 服务）：
    bash

gz physics -c  # 清除所有力和速度

world文件中添加<collide>false</collide>取消所有碰撞检查

    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <collide>false</collide>
    </physics>


world（例如office_env_large.world）中添加模型后，需要将world文件中的第一次出现单位名称时的global pose 替换第二次出现时的global pose
不然显示的pose有问题    

由world生成地图技巧：
1，在gazebo中切换视角到俯视图，ortho视角，取消grid可见性
2，透明化地板，截图
3，按分辨率调整地图尺寸，得到栅格地图

04-21,王老师意见

规划完之后所有轨迹可见？
然后部分可见，避免过于杂乱？

一个窗口3D显示，一个2D显示？只显示未完成轨迹
根据各个单位颜色显示运动状态，运动中、等待、故障？


# 250718, 由ubuntu 22.04转到24.04

在 Gazebo Harmonic / Ionic（即 gz-sim9/gz-sim10） 这一代 Gazebo 中，.world 文件已经不再使用或推荐，取而代之的是标准的 .sdf 文件。

## Launch Gazebo from ROS 2
https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/


## Spawn, set pose and delete entities using ROS 2 #705 

https://github.com/gazebosim/ros_gz/pull/705


# Launch Gazebo: （添加实体、设置实体位置、删除实体，通过服务）

`
gz sim code/ros2_ws/src/ros_gz/ros_gz_sim_demos/worlds/default.sdf
`

Run the ROS-Gazebo bridge for entity creation:

`
    ros2 run ros_gz_bridge parameter_bridge /world/default/create@ros_gz_interfaces/srv/SpawnEntity
`

`
    ros2 run ros_gz_bridge parameter_bridge /world/default/create@ros_gz_interfaces/srv/SetEntityPose
`

`
    ros2 run ros_gz_bridge parameter_bridge /world/default/create@ros_gz_interfaces/srv/DeleteEntity
`

Spawn an entity:

`
  ros2 run ros_gz_sim spawn_entity --name box --sdf_filename $(ros2 pkg prefix ros_gz_sim_demos)/share/ros_gz_sim_demos/models/cardboard_box/model.sdf
`
Similarly, you can test set_entity_pose and delete_entity with their respective bridge services.

测试成功
# Launch Gazebo from ROS 2

https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/

The package ros_gz_sim contains two launch files named gz_server.launch.py and gz_sim.launch.py. You can use them to start Gazebo server or Gazebo (server and GUI) respectively.
 
 `
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
 `
Or you can just start the server:
`
ros2 launch ros_gz_sim gz_server.launch.py world_sdf_file:=empty.sdf
`

# Launching with ros_gz_bridge(不涉及添加实体，设置实体位置，删除实体等操作)

An example launch file for XML can be viewed here An example launch file for Python can be viewed here

Example command for directly using these launch files from the terminal:

`
ros2 launch ros_gz_sim ros_gz_sim.launch.py world_sdf_file:=empty.sdf bridge_name:=ros_gz_bridge config_file:=<path_to_your_YAML_file> use_composition:=True create_own_container:=True
`

In the above launch files you may notice that the create_own_container argument for ros_gz_bridge is hardcoded to False. This has been done to prevent two duplicate containers from getting created (one for gz_server and another one for ros_gz_bridge), and instead make ros_gz_bridge use the container created by gz_server. More info about this can be viewed here

建立ros2和gazebo之间的桥接联系

https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md

# 另一种办法启动gazebo并添加实体的方法
启动gazebo
`
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
`

添加实体

`
ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=empty file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5
`

# 250718

启动gazebo，以及添加实体、删除实体、设置实体位姿的服务

`
ros2 launch lamapf_and_gazebo ros_gz_launch.launch.py
`

添加实体

`
ros2 run ros_gz_sim spawn_entity --name box --sdf_filename $(ros2 pkg prefix ros_gz_sim_demos)/share/ros_gz_sim_demos/models/cardboard_box/model.sdf
`

# https://gazebosim.org/api/sim/7/model_command.html

To try out this command we need first a running simulation. Let's load the diff_drive example world. In a terminal, run:

gz sim diff_drive.sdf

Once Gazebo is up, we can use the gz model command to get information of the simulation. Open a new terminal and enter:

gz model --list

And available models should be printed:

Available models:
    - ground_plane
    - vehicle_blue
    - vehicle_green

Once you get the name of the model you want to see, you may run the following commands to get its properties.

gz model -m <model_name> to get the complete information of the model. e.g.

gz model -m vehicle_blue

如何添加指定sdf文件（原world文件）

如果.world 文件已经是完整的 SDF 格式（以 <sdf> 为根元素），可以直接重命名


如何配置gazebo harmonic查找文件的路径

https://2048.csdn.net/682c4ba6606a8318e858358a.html?dp_token=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpZCI6NTQ5MDAxOSwiZXhwIjoxNzUzNDY4MzQwLCJpYXQiOjE3NTI4NjM1NDAsInVzZXJuYW1lIjoid2VpeGluXzM5NDkwNTM1In0.wqacTiG9_g7jKR2quf6-T-E3eTU9oqzLB0nKBVTd3Nc#:~:text=%E6%8A%8A%E4%B8%8B%E8%BD%BD%E7%9A%84%E6%A8%A1%E5%9E%8B%E6%96%87%E4%BB%B6%E5%A4%B9%E5%A4%8D%E5%88%B6%E5%88%B0gz_models%E4%B8%8B%EF%BC%9B%E7%84%B6%E5%90%8E%E5%9C%A8%E7%BB%88%E7%AB%AF%E4%B8%AD%E8%BE%93%E5%85%A5%20%28%E7%9C%81%E7%95%A5%E5%8F%B7%E4%B8%BAgz_models%E7%9A%84%E7%BB%9D%E5%AF%B9%E8%B7%AF%E5%BE%84%EF%BC%89%20%E6%88%96%E8%80%85%E8%BF%9B%E5%85%A5%EF%BD%9E%2F.bashrc%2C%E8%BE%93%E5%85%A5%E4%B8%8A%E8%BF%B0%E8%B7%AF%E5%BE%84%EF%BC%8C%E4%BB%A5%E5%90%8E%E5%B0%B1%E4%B8%8D%E7%94%A8%E9%87%8D%E5%A4%8D%E8%BE%93%E5%85%A5%EF%BC%9B%20%E6%8E%A5%E4%B8%8B%E6%9D%A5%EF%BC%8C%E8%BF%9B%E5%85%A5Gazebo,3.%E6%8F%92%E4%BB%B6%E5%AF%BC%E5%85%A5%E6%9C%AC%E5%9C%B0%E6%A8%A1%E5%9E%8B%EF%BC%9A%E4%BB%8EGazebo%E5%8F%B3%E4%B8%8A%E8%A7%92%E7%9A%84%E6%90%9C%E7%B4%A2%E4%B8%AD%EF%BC%8C%E6%89%93%E5%BC%80Resource%20Spawner%E6%8F%92%E4%BB%B6%EF%BC%8C%E5%8F%B3%E6%A0%8F%E4%B8%8B%E6%8B%89%E6%98%BE%E7%A4%BA%EF%BC%88%E6%B3%A8%E6%84%8F%E8%BF%99%E9%87%8C%E6%9C%89%E6%97%B6%E4%BE%AF%E6%BB%91%E5%8A%A8%E6%9D%A1%E6%B2%A1%E6%98%BE%E7%A4%BA%E7%9A%84%E8%AF%9D%EF%BC%8C%E6%9C%80%E5%A4%A7%E5%8C%96%E9%A1%B5%E9%9D%A2%E5%B0%B1%E8%83%BD%E7%9C%8B%E5%88%B0%20%EF%BC%89%E6%AD%A4%E6%97%B6%E5%B0%B1%E8%83%BD%E5%9C%A8Local%20Resorece%E4%B8%AD%E7%9C%8B%E5%88%B0%E4%BD%A0%E5%AF%BC%E5%85%A5%E7%9A%84%E6%A8%A1%E5%9E%8B%E5%BA%93%EF%BC%9B


首先在终端中新建你的Models库：

mkdir gz_models
cd gz_models

把下载的模型文件夹复制到gz_models下；然后在终端中输入(省略号为gz_models的绝对路径）
 

export GZ_SIM_RESOURCE_PATH=~/……/gz_models/

 或者进入～/.bashrc,输入上述路径，以后就不用重复输入；

 接下来，进入Gazebo

gz sim

3.插件导入本地模型：从Gazebo右上角的搜索中，打开Resource Spawner插件，右栏下拉显示（注意这里有时侯滑动条没显示的话，最大化页面就能看到 ）此时就能在Local Resorece中看到你导入的模型库；


![alt text](image.png)

gazebo 右上角搜索 view angle，控制视角 


gazebo模型网站
https://app.gazebosim.org/fuel/worlds


https://github.com/gazebosim/gz-sim/tree/main/examples/worlds


构造虚拟instance,单独运行以下代码即可

```
ros2 run lamapf_and_gazebo generate_instance_node
```

启动gazebo，以及添加实体、删除实体、设置实体位姿的服务

```
ros2 launch lamapf_and_gazebo ros_gz_launch.launch.py
```

启动MAPF，并在gazebo中可视化

```
ros2 run lamapf_and_gazebo test_lamapf_planner_node
```

启动gazebo可视化会失败，单独opencv窗口可以，或由于多节点并行和更新位姿耗时冲突导致

空格开始移动，按空格暂停移动

测试加载实体

```
ros2 run lamapf_and_gazebo spawn_entity
```



空地图添加模型成功，但有sdf world添加失败，为什么？

. **ROS 2服务名称冲突**：
    在加载不同世界时，服务名称可能发生变化（特别是当世界名称不同时）。
    **解决方案**：
    - 检查服务名称：在加载有模型的世界后，运行`ros2 service list`，确认`/spawn_entity`服务是否存在。
    - 注意：服务名称可能包含世界名称，例如`/world/your_world_name/spawn_entity`。在调用服务时，确保使用正确的服务名称。

SDF文件中的世界名称，统一设置为default

  <world name='default'>

新添加的包需要重开终端，重新source bash setup才能找到

ctrl+shift+T拓展的终端不会重新source

## 生成地图

gazebo sim 选择俯视图，正交视角
截图后生成规划用地图时考虑栅格对应的分辨率，并顺时针转90度，以对其坐标系
涂白（255,255,255）区域为可通行区域

启动gazebo，以及添加实体、删除实体、设置实体位姿的服务

```
ros2 launch lamapf_and_gazebo ros_gz_launch.launch.py
```

start central controller test
```
ros2 run lamapf_and_gazebo test_central_controller
```
在 ROS2 里，终端打印 (RCLCPP_INFO / RCLCPP_ERROR / RCLCPP_WARN 等) 默认是写到 stdout/stderr，所以有几种方式可以把这些日志保存到文件里：

直接用终端重定向（最简单）

ros2 run my_pkg my_node > my_node.log 2>&1

如果你在用 ros2 launch：

ros2 launch my_pkg my_launch.py > launch.log 2>&1

TODO:

增加单位路径上的碰撞检测
有碰撞风险则等待至无碰撞或超时报错，超时后更新地图重启规划。

将每个机器人作为一个node计算控制指令 (实现)

通过ros的跨平台多节点通信机制实现不同机器人与控制中心的通信（实现）

通过服务实现总控和机器人间通信（部分实现，缺少实机部分的联网配置以及单机启动）

规划：
基于MPC的直线运动控制(实现)；
局部碰撞预测(根据laser scan预测沿预定路线前进是否有碰撞风险)；
停机后中控重新恢复（已添加中断，尚未添加恢复）；
跨平台IP配置

单机器人简单建图与定位。


在gazebo里获取模拟的激光雷达数据，模拟的里程计数据
进而模拟单机器人定位与碰撞检测
进而模拟多机器人导航


老师，我向您汇报一下我最近工作进展和未来计划：主要任务是多机器人导航系统的落地。计划分两步走，先在gazebo上搭建场景模拟仿真，跑通完整流程后实物落地。

仿真部分已经完成了场景搭建，多机器人路径规划（Large Agent CBS+我的多机器人路径规划分解+我的高效碰撞检测）以及导航调度（基于动作依赖图action dependency graph），机器人运动控制（基于模型预测控制MPC）；

尚未完成的有单机器人传感器数据模拟（基于gazebo和ros2通信机制）、定位（计划采用amcl）、建图（计划采用gmapping或cartographer）；

实物部分尚未完成的有机器人传感器与数据获取（基于实验室现有turtlebot机器人+微型主机），多机器人间的通信（ros2自带通过连接同一wifi实现机器人间的无线通信）。

之前投稿的四篇论文仍在审核，一篇二审（JAAMAS）三篇一审（IJRR，IEEE T-ASE, Journal of Intelligent & Robotic Systems）。

如果你在 Gazebo Harmonic 里加载了一个 GpuLidar 传感器，它会发布类似：

/world/default/model/your_robot/link/lidar_link/sensor/lidar/scan


你要想在 ROS2 下看到 /scan（类型 sensor_msgs/msg/LaserScan），就需要 bridge：

ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/your_robot/link/lidar_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan

SDF 规范不允许 world 下面直接放 link。
<link> 必须放在 <model> 里面。


！！！！！
gazebo开局默认暂停，需要点击左下角播放键，才有传感器数据流

gazebo官方文档，演示如何添加传感器生成数据流

添加模型时，必须在world里添加插件，否则无法生成数据流

    <!-- 系统插件 -->
    <plugin
        filename="gz-sim-physics-system"
        name="gz::sim::systems::Physics">
    </plugin>
    <plugin
        filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    
    <plugin 
        filename="gz-sim-imu-system" 
        name="gz::sim::systems::Imu">
    </plugin>

    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <plugin
        filename="gz-sim-user-commands-system"
        name="gz::sim::systems::UserCommands">
    </plugin>


查看是否发布消息
gz topic -l

打印消息内容
gz topic -e -t /world/default/model/box/link/link/sensor/imu_sensor/imu

如果传感器成功工作，会打印出传感器消息

参考simple_world.sdf

加载world时消息桥接。
def generate_launch_description():
    # world 文件路径
    world_file = os.path.join(
        os.getenv("HOME"), "code/ros2_ws/src/lamapf_and_gazebo/world/simple_world.sdf"
    )

    # Gazebo 仿真
    gz_sim = Node(
        package="ros_gz_sim",
        executable="gz_sim",
        arguments=["-r", world_file],
        output="screen"
    )

    # ROS–Gazebo bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # imu
            "/world/default/model/imu_box/link/base_link/sensor/imu_sensor/imu"
            "@sensor_msgs/msg/Imu[gz.msgs.IMU",

            # lidar
            "/world/default/model/lidar_robot/link/base_link/sensor/lidar/scan"
            "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            # odometry（如果有）
            "/model/lidar_robot/odometry"
            "@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        output="screen"
    )

手动输入命令实现gazebo中模拟laserscan话题桥接

ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/robot_with_lidar/link/base_link/sensor/lidar_sensor/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan

将其通过

    laser_bridge_bridge = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
  '/world/default/model/robot_with_lidar/link/base_link/sensor/lidar_sensor/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'], output='screen')

  在ros_gz_launch.launch.py中启动，实现了在ros2读取到gazebo中的模拟激光雷达数据

在ros2中对应的消息类型是sensor_msgs/msg/LaserScan，名字与在gz中的相同

在 SDF/SDFormat 的 官方 schema中，<topic> 只对部分传感器类型（如 camera、imu）开放

但是 激光雷达（lidar / gpu_lidar / ray sensor） 目前 不支持 <topic> 标签。

lidar scan 无法通过topic配置名称


ros2 定位用nav2-amcl
启动
ros2 run nav2_amcl amcl


The Robotics Back-End：ROS2 + SLAM Toolbox 生成地图

https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation

这篇教程使用 TurtleBot3 Gazebo 模拟器 演示了以下完整流程：

环境准备（包含 DDS 切换到 Cyclone DDS）

安装 Nav2 和 SLAM Toolbox

启动仿真机器人（ROS2 Gazebo + /scan 发布）

启动 Nav2 和 SLAM Toolbox，进行地图构建

使用 RViz2 可视化地图

保存地图文件（.pgm 和 .yaml）

只要在 ros2_ws/src/ 的层级结构中（无论直接放还是放在子目录），colcon 都能正常编译和使用，区别只是目录管理上的清晰度。


通过

ros2 launch turtlebot3_gazebo empty_world.launch.py

启动带有一个turtle bot的gazebo仿真环境,没有障碍物


ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


启动带有一个turtle bot的gazebo仿真环境,有障碍物


ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

启动带有一个turtle bot的gazebo仿真环境, 模拟室内场景



ros2 run turtlebot3_teleop teleop_keyboard

键盘控制gazebo中机器人移动

残留之前物体，关闭gazebo重开也无法消除时，重启电脑


# gazebo+ros2+turtlebot建图仿真流程

1,启动gazebo world，加载环境和机器人

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2,启动建图节点
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

3, 键盘wasdx控制机器人移动
ros2 run turtlebot3_teleop teleop_keyboard

4,保存地图及参数yaml文件
ros2 run nav2_map_server map_saver_cli -f ~/map


# gazebo+ros2+turtlebot导航仿真流程
1, 启动gazebo world，加载环境和机器人

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2, 启动导航节点，加载地图

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml

3, rviz中点击2D Pose Estimate，设置初始位置

点击nav goal，设置目标位置




如何在终端模拟发送自定义ErrorState

想发布到 /AgentErrorState 话题

ros2 topic pub /AgentErrorState lamapf_and_gazebo_msgs/msg/ErrorState "{agent_id: 3, error_state: -1}"

运行后会默认 持续发布（10Hz）。如果只想发一条，可以加 --once：

ros2 topic pub --once /AgentErrorState lamapf_and_gazebo_msgs/msg/ErrorState "{agent_id: 3, error_state: -1}"

在 ROS2 里，终端打印 (RCLCPP_INFO / RCLCPP_ERROR / RCLCPP_WARN 等) 默认是写到 stdout/stderr，所以有几种方式可以把这些日志保存到文件里：

直接用终端重定向（最简单）

ros2 run my_pkg my_node > my_node.log 2>&1

如果你在用 ros2 launch：

ros2 launch my_pkg my_launch.py > launch.log 2>&1

vscode要选择所有包含 "hello" 的行：

    双击第一个 "hello"

    按 Ctrl+Shift+L 选择所有 "hello"

    按 Ctrl+L 选择整行

    按 Ctrl+C 复制


增加了断点恢复功能



接收到机器人异常后，
测试中模拟发送: ros2 topic pub --once /AgentErrorState lamapf_and_gazebo_msgs/msg/ErrorState "{agent_id: 3, error_state: -1}"

按机器人最后的可用位姿重新规划路径
启动 controller:
ros2 run lamapf_and_gazebo test_central_controller > my_node.log 2>&1 

规划完成后继续原任务


开发碰撞预测

分为旋转预测和前进预测。
如果旋转过程中的几何形状或者前进预测中的几何形状，与激光点云有交集，则判断为存在碰撞风险
逐个检查每个点是否在几何形状范围内。
local controller应该等待直到可用，或者等待超过一定时间，重启任务

MAPF计划执行的两种不确定性：
1, 时间不确定性
通信延时和速度差异，导致各机器人到达预定目标的时间先后顺序不确定，如不考虑可能导致任务失败
采用action dependency graph，机器人必须等计划中必须更早的机器人到达目标位姿后再继续任务
否则等待，
确保严格按照时间顺序执行计划

2, 位姿不确定性
机器人在移动过程中，因为定位存在误差，可能随机抖动，导致机器人不能一直严格按照计划执行
检测到偏离原计划路径过远时，按照当前位置重新规划，以完成任务目标



25-09-22

1, 根据turtlebot，是否能供电，以及确定微型主机选购方案

  TurtleBot2 的供电情况：

  TurtleBot2 的底座通常是 iRobot Create 2，内部带有 14.4V（官方标称）镍氢电池。

  官方设计时就考虑了给上层计算单元（笔记本/NUC/mini PC）供电。

  Create 2 底座上有一个 DB25 扩展口（或通过电源接口板）可以引出电池电压（约 12–16V 随电池状态变化）。

  一般会通过一个 DC-DC 降压模块 把电池电压转换为 12V 或 5V，供给微型主机、Kinect 等设备。

  常见配置：

  很多 TurtleBot2 使用 Intel NUC、笔记本、Jetson、NUC-like 工控机作为主机，功耗 15–30W 左右，底座电池是可以带动的。

  如果是功耗更大的 x86 主机（>50W），就需要注意电池续航会大幅缩短。

  注意点：

  电压匹配：确认你的微型主机需要 12V 还是 19V（很多 NUC/笔记本是 19V）。如果是 19V，需要一个升压模块。

  电流足够：Create 2 电池容量有限，电流输出没问题，但续航会受影响。

  保护电路：建议加保险丝或电源管理模块，避免过放电或电压波动损坏主机。

  官方配件：TurtleBot2 的电源板（Power Board）本身就支持给外部设备提供 12V/5V，比较安全。

  ![alt text](image-1.png)

  👉 总结：
  TurtleBot2 是可以直接给微型主机供电的，只要通过合适的 DC-DC 电源转换模块，大部分 NUC、Jetson Nano/Orin、工控 mini PC 都能稳定使用。

  图中内容 & 标注（基于文档）

这个“Power Panel”部分通常包括几个输出电压接口，不同用途 / 不同负载要求的输出。例如：

| 接口标识 |	电压	| 电流（大致／最大负载）|	用途 |
|-------- | ------|-------------------- | -----| 
| 12V @ 5A |	用于功率较大的外设，比如机械臂（arm）或其它大功率模块 |	较大电流输出 |	高功耗外设 |
| 12V @ 1.5A |	通常给 Kinect 或类似中等功耗传感器	| 中等电流 |	传感器或外设 |
| 5V @ 1A |	小外设，如一些低功率控制板、通用板子等 |	比较小 |	控制／低功耗模块 |
| 19V @ 2A |	为笔记本／微型主机类设备提供电源（如果使用的是笔记本或需要 19V 输入的主机） | 较高要求时使用 | 微型主机／笔记本供电 |

2, 配置单机器人定位、建图和运动控制

3, 配置多机器人MAPF测试



251006

单独机器人配置
1, 安装ubuntu 24.04
2, 安装ros2
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


3，安装opencv
sudo apt update
sudo apt install libopencv-dev



4, 安装ros_gz_interfaces
sudo apt-get install ros-jazzy-ros-gz-interfaces

5, 安装tf2
sudo apt-get install ros-jazzy-tf2

6, 安装std-msgs, sensor msgs
sudo apt-get install ros-jazzy-std-msgs
sudo apt-get install ros-jazzy-sensor-msgs


7, 安装eigen
sudo apt-get install libeigen3-dev

### SuiteSparse
sudo apt-get install libsuitesparse-dev

8, 安装libxml2
sudo apt-get install libxml2-dev


* 安装科学上网

https://hiddify.zip/ 下载hiddify

chmod +x hiddify-linux-x64.AppImage

安装fuse2
sudo apt install libfuse2

9, 安装LayeredMAPF
git clone git@github.com:JoeYao-bit/LayeredMAPF.git

git clone --depth=1 git@github.com:JoeYao-bit/LayeredMAPF.git
(只要最新版本，节约时间)

更新子仓库
git submodule update --init


安装epoxy
sudo apt install -y libepoxy-dev


kaHypar可以删掉
# ubuntu系统跨平台互联

1, 连接到同一wifi网络下，关闭vpn

2, 通过ip addr show查看在wifi下的ip，关闭开启wifi后新增加的ip中
通常是    inet 10.34.152.24/16 brd 10.34.255.255 scope global dynamic noprefixroute wlo1
       valid_lft 4836sec preferred_lft 4836sec
这样的格式。
其中inet 10.34.152.24是想要的ip地址。

通过ping 10.34.152.24测试在另一平台上是否可以和本机通信。

3, 配置 ROS 2 环境变量

ROS 2 的通信基于 DDS (Data Distribution Service)，不依赖 ROS Master（不像 ROS 1）。
但要让两个主机在同一个 DDS 域中通信，需要统一几个关键参数。

（1）确保两台机器的 ROS_DOMAIN_ID 相同

在两台电脑的 ~/.bashrc 中添加相同的：

export ROS_DOMAIN_ID=0

如果在同一个局域网运行多个 ROS 系统，可以设置为其他数字（例如 10、20），但两台需要一致。


在每台电脑的 ~/.bashrc 中添加：

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

ROS_LOCALHOST_ONLY=0：允许通过局域网通信（否则只在本机上）

rmw_fastrtps_cpp 是默认的 DDS 实现，也可以换成 rmw_cyclonedds_cpp（通信效果更稳定）

4️⃣ 测试 listener/talker

A 机：

ros2 run demo_nodes_cpp talker

B 机：

ros2 run demo_nodes_cpp listener



在家庭wifi和另一台笔记本上，只需设置好
export ROS_DOMAIN_ID=0

export ROS_LOCALHOST_ONLY=0

即可通过listener和talker测试，
估计是学校wifi特殊导致的问题

连接Tech Sup Provide WangW Lab可以实现通信

微型主机电源供电，

微型主机安装配套turtlebot的建图和定位包


1, 安装kobuki相关的包
ecl_core,ecl_lite,kobuki_core,kobuki_ros,kobuki_ros_interfaces

git clone https://github.com/stonier/ecl_lite.git

git clone https://github.com/stonier/ecl_core.git

git clone https://github.com/kobuki-base/kobuki_ros.git

git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git

git clone https://github.com/kobuki-base/kobuki_core.git

安装 ecl_build

sudo apt install ros-jazzy-ecl-build

安装 sophus

sudo apt install ros-jazzy-sophus

编译

colcon build --cmake-args -DCMAKE_CXX_FLAGS="-Wno-error=overloaded-virtual"


启动与turtlebot的连接
 ros2 launch kobuki_node-launch.py 

编译器认为基类的 operator= 被隐藏了，所以发出警告。

为什么编译失败？

ROS2 Jazzy 使用的 GCC/Clang 默认把 所有警告当作错误 (-Werror)，

所以即便只是警告，也会直接导致构建失败。




方法 1：在编译时禁止将警告当作错误

你可以在 workspace 的 CMakeLists.txt 或 colcon build 时加入：

需要root 和 dialout 用户组可以访问

你的普通用户不在 dialout 组，所以会报 permission denied

添加用户到dialout组

sudo usermod -aG dialout $USER


完全退出你的用户账号（GUI 或 SSH）。

重新登录。

再执行：

groups

你会看到 dialout 已经在列表里。


在kobuki_ros/kobuki_node下
启动对turtlebot的连接：ros2 launch kobuki_node-launch.py

再运行ros2 run kobuki_keyop kobuki_keyop_node
实现键盘控制移动

你的 kobuki_ros_node 配置文件（YAML）中确实没有显式设置 波特率；

但 stty -F /dev/ttyUSB0 显示当前串口速率为 115200 baud；

这实际上就是 Kobuki 的默认波特率。

所以 ✅ 你的波特率已经是正确的，不需要额外配置。

运行 

ros2 run kobuki_keyop kobuki_keyop_node --ros-args -r cmd_vel:=/commands/velocity

启动键盘控制运动

kobuki_ros_node 实际上 订阅的是 /commands/velocity，
而 kobuki_keyop 默认发布的是 /cmd_vel。

因此需要remap cmdvel话题到/commands/velocity话题，即可实现键盘运动控制
（神人官方，话题都不匹配）
里程计发布在/odom话题

连接激光雷达
目前激光雷达，单独通过usb线和机器连接，不经过turtlebot本身

安装 ROS 2 版 RPLIDAR 驱动包
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git



单独编译某个包
colcon build --packages-select rplidar_ros

启动连接激光雷达，并发布到话题/scan
ros2 launch rplidar_ros rplidar_a2m8_launch.py


我一个usb口连turtlebot机器人，一个连rplidar，如何自动识别匹配usb号
让系统自动识别并固定每个设备（即使拔插顺序改变，也不会错乱）。

我们可以用 udev 规则（推荐方式） 为每个设备创建独立的固定名称：
例如：

机器人
➜  ros2_ws udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' -m 3
    SUBSYSTEMS=="usb-serial"
    ATTRS{idProduct}=="6001"
    ATTRS{idVendor}=="0403"

激光雷达  
➜  ros2_ws udevadm info -a -n /dev/ttyUSB1 | grep -E 'idVendor|idProduct|serial' -m 3
    SUBSYSTEMS=="usb-serial"
    ATTRS{idProduct}=="ea60"
    ATTRS{idVendor}=="10c4"
➜  ros2_ws 
  

# RPLIDAR
ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="A602WXYZ", SYMLINK+="rplidar"

# TurtleBot Base (Kobuki)
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="FTXYZ123", SYMLINK+="kobuki"


我们可以用这些信息创建一个稳定的 udev 规则，让每次插拔都自动生成固定端口名：
/dev/kobuki 和 /dev/rplidar。


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

🧭 四、ROS2 中使用

启动激光雷达

ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_port:=/dev/rplidar serial_baudrate:=115200

启动机器人

ros2 launch kobuki_node kobuki_node-launch.py serial_port:=/dev/kobuki serial_baudrate:=115200

测试通过

安装ros2建图工具包

sudo apt install ros-jazzy-slam-toolbox

tf只有base foot print 到odom的tf，需要laser到base foot print的tf

发布静态tf

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint laser

激光雷达的数据话题名 /scan ≠ 激光雷达的 坐标系 frame_id = "laser"


启动建图指令

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false


安装tf2

 sudo apt install ros-jazzy-tf2

 sudo apt install ros-jazzy-tf2-msgs

 sudo apt install ros-jazzy-tf2-geometry-msgs 

 sudo apt install ros-jazzy-tf2-ros 


map_update_interval: 5.0

这个参数控制 SLAM Toolbox 发布 /map_updates 的频率（秒为单位）。

默认 5 秒意味着地图更新很慢，如果你只是微动或者传感器帧率低，/map_updates 可能在长时间内没有变化，看起来像“只显示第一帧地图”。

throttle_scans: 1 表示每帧 scan 都处理，这可以保持更新频繁。

minimum_travel_distance 和 minimum_travel_heading 控制 SLAM 是否更新地图，如果你走得太慢或者旋转角度太小，也可能导致 /map_updates 没变化。


使用自己参数，原参数更新频率太低，最小更新距离太大

原参数启动
ros2 launch slam_toolbox online_async_launch.py 

自定义参数启动
ros2 launch lamapf_and_gazebo   turtlebot2_online_async_launch.py


安装地图服务器

sudo apt install ros-jazzy-nav2-map-server


保存地图到指定位置和名字，得到.pgm文件和yaml文件

ros2 run nav2_map_server map_saver_cli -f ~/my_map


安装amcl定位

sudo apt install ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager


启动amcl定位 

ros2 launch lamapf_and_gazebo turtlebot2_amcl_localization.launch.py use_sim_time:=false

 AMCL 粒子滤波器仍然需要 明确的初始位姿消息 /initialpose。

手动运行脚本/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/script/initial_pose_publisher.py
发布制定地图yaml文件的位姿

即可实现定位

amcl_localization.yaml参数中
    update_min_d: 0.01
    update_min_a: 0.01

限制了更新位姿的最小阀值，静止不动或过小则不发布新的位姿


如果地图建图原点位姿不是初始化定位的位姿，后续在rviz2中通过设置pose estimate手动重新设置初始位姿

为确保地图能在rviz2中可视化，而且地图仅amcl启动时加载一次，
先打开rviz2，选择地图为map，再启动定位



# 最新关键操作指令

## 1,配置USB连接规则

查看所有连接的usb设备
lsusb




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
KERNEL=="ttyUSB*", SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="kobuki", MODE:="0666"


# RPLIDAR (Silicon Labs CP210x)
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTR{idVendor}=="10c4", ATTR{idProduct}=="ea60", SYMLINK+="rplidar"

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

配置 kobuki_ros/kobuki_node/config/kobuki_node_params.yaml 中 device_port 为/dev/kobuki
然后更新参数 colcon build --packages-select kobuki_node
ros2 launch kobuki_node kobuki_node-launch.py serial_baudrate:=115200

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

sudo apt install ros-jazzy-nav2-map-server

ros2 run nav2_map_server map_saver_cli -f ~/my_map

在turtlebot2_amcl_localization.launch.py和initial_pose_publisher.py中设置地图名称

## 9, 启动定位
改变yaml参数文件地址
ros2 launch lamapf_and_gazebo turtlebot2_amcl_localization.launch.py use_sim_time:=false

## 10, 发布初始位置（更新amcl_localization.yaml，在启动时设置初始位置后已经可以废弃）
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

安装yaml-cpp和eigen，方便坐标变换

sudo apt install libyaml-cpp-dev libeigen3-dev

真机实验
1,各个机器人的定位和局部控制节点有自己的唯一编号，共用用一张地图
2,局部控制器接收唯一指定定位节点的结果
3,定位节点接收指定编号的里程计和激光雷达信息，以及tf信息？


tf树可以通过指定名称空间的方式隔离
不同机器人的同类话题比如odom可以通过指定名称空间的方式避免话题名重复
命名空间不会阻止跨空间通信；只要两个节点话题路径完全匹配（字符串相同），它们就能通信；
ROS 2 不会自动在不同命名空间间桥接或阻断通信。

构造如图所示的tf树

map)
 ├── robot1/odom → robot1/base_link → ...
 ├── robot2/odom → robot2/base_link → ...
 └── robot3/odom → robot3/base_link → ...


kobuki_ros odom tf话题名字在 kobuki_node_params.yaml设置

    odom_frame: odom
    base_frame: base_footprint

rplidar 的 frame_id 在rplidar_a2m8_launch.py中配置

    frame_id = LaunchConfiguration('frame_id', default='laser')
 
但laser到base_footprint的tf由我通过命令发布
在这里需要改为指定机器人的laser名称

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint laser

假设各个机器人已经有准确定位信息，
如何选择各个机器人目标点？

1，固定地图，预设各个机器人的起点终点，即固定场景
地图名称、各个机器人起末位置，均为固定参数，写入指定文件
先制作地图，在选择每个机器人期末位姿，写入文件
确保各个机器人均处于指定位置之后
再加载文件对应的起末位姿，进行全局规划和局部运动控制

需要根据地图yaml进行文件生成坐标系转换函数

先获取各个机器人起末点的世界坐标，再将其转换为栅格地图坐标

导入规划系统

所有机器人采用turtlebot

重写专门局部控制节点， 完成

2,根据当前各个机器人位姿动态随机选择（算法随机或人工临时设置）

先试试第一个


测试加载yaml地图参数, 完成

ros2 run lamapf_and_gazebo test_yaml_converter

现有框架允许单一机器人在MAPF体系下工作

后续计划

机身电源供电
建图（更大的室内地图）
真实定位替换虚拟定位
单机器人框架下导航

## 万用表测量微型主机电源电压

插好表笔

黑表笔插在 COM 孔

红表笔插在 VΩmA 孔

选择量程档位

把旋钮转到带有符号 “V—” 或 “DCV” 的区域

选一个比你预期电压高的档位

例如测 12V，用 “20V” 档最合适。
如果不确定电压大小，可先用 “200V” 档，测到数值后再切低档。

接触待测电源

红表笔接 电源正极（+）

黑表笔接 电源负极（−）或地

读取数值

屏幕上显示的就是电压值（单位 V）

若显示 负号 “−”，说明你接反了（红黑笔接反），重新调换即可。

## turtlebot 输出12V 5A电源正负极

上、下两个孔分别是 正极 (+) 和 负极 (−)

一般惯例：红色正极、黑色负极

剥线钳用1.0的口径剥线

## 远程桌面
实现微型主机和屏幕的无线通信，能通过桌面“看到”微型主机的屏幕。
便于调试
<!-- 
已弃用！
微型主机：
通过 ip addr show查看在同一wifi下的ip地址，例如192.168.4.23
点开系统。点开桌面共享，设置允许桌面共享和远程控制，以及用户名和登录密码

笔记本电脑：
安装remmina：sudo apt install remmina -y
打开Remmina，输入微型主机的ip地址，输入微型主机设置的用户名，密码，域留空，登陆即可

验证通过，但需要每次查看ip地址，而且ip地址每次开机可能会变。

尝试通过rustdesk实现开机自启动，根据id和密码即可远程控制

如何查看系统架构是x86 64还是arm 64? 输入uname -m
威嵌沃微型主机和笔记本都是x86 64架构

第一台微机的rust id是 342 459 772
永久登录密码：Wangwei12345678 -->

安装todesk，连接hidm欺骗器，假装虚拟主机连接了屏幕，
即可通过todesk远程控制
为了降低通信延迟，可以调低分辨率至1280*720

11.17 anydesk不好使，换anydesk
远程控制密码:wasd16807
## 如何设计实验？
找个开阔点的场地，比如地下实验室，建图
设置起点，终点
开始MAPF


## 微型主机的固定在turtlebot 上
微型主机底部有四个M3的定位螺纹孔，
通过turtlebot自带的定位孔即可定位
M3，长度12mm以上为佳

11.17测试发现串口根据设备id匹配似乎失效。
需要手动调整串口连接微型主机的位置
成功连接后激光雷达和键盘控制仍然可用

## 工作流程设计
1, 启动激光雷达和turtlebot

2, 启动定位，设置初始位置，确保定位准确

3, 设置各个机器人路径终点（起点为当前位置）写入参数文件

4, 从参数文件启动中央控制器，给各个局部控制器下发局部运动目标


接下来先测试局部控制器，再实现上述中央控制器


模拟发布给局部控制器的指令
--once表示只发布一次


ros2 topic pub --once /goal lamapf_and_gazebo_msgs/msg/UpdateGoal "{start_x: 1.0, start_y: 2.0, start_yaw: 0.0, target_x: 5.0, target_y: 6.0, target_yaw: 1.57, agent_id: 1, wait: true}"


串口识别失败，连接turtlebot 出问题了

已解决，流程已经更新

为什么启动amcl一开始收不到地图并显示？

1️⃣ TF 不对导致地图显示失败

RViz2 渲染地图依赖 map → odom → base_link 的 TF 树

如果启动时 TF 不连通或者丢失，RViz2 即使收到了 /map 消息，也无法显示

设置初始位置之前，amcl的tf树里没有map，rviz收到地图也不显示
设置初始位置后，收到地图就能正常显示了


想办法直接从参数文件中启动时设置初始位置


通过配置amcl_localization.yaml已经完成
    set_initial_pose: True
    initial_pose:
      x: 1.0
      y: 2.0
      yaw: 1.57


TODO：
配置amcl使得其周期发布位姿

save_pose_rate: 10.0
tf_broadcast_frequency: 20.0

失败，

save_pose_rate 控制的是“将机器人位姿保存到磁盘的频率”

不会触发周期性发布 /amcl_pose

/amcl_pose 只会在以下情况发布：

激光数据到来 → 位姿估计更新

位姿变化超过 update_min_d / update_min_a

设置初始位置 /initialpose

⚠ 即使 save_pose_rate 设置成 1000.0，也不会定时发布 /amcl_pose

查看两坐标系之间的实时 TF 变换：

ros2 run tf2_ros tf2_echo <target_frame> <source_frame>

ros2 run tf2_ros tf2_echo map base_footprint

/amcl_pose无法实时发布

但tf中可以实时查看map base_footprint之间的tf，即定位结果

安装tf2_ros,tf2_geometry_msgs

sudo apt install ros-jazzy-tf2-ros 
sudo apt install ros-jazzy-tf2-geometry-msgs

充电时不一定需要底盘电源开关打开，电池本身是可充电的独立单元，充电器直接给电池供电即可

由于/amcl_pose不会持续发送，考虑在local planner中使用tf作为定位信息获取来源

通过localization_full.launch.py直接启动
完成以下五件事

加载激光雷达，连接turtlebot，发布静态transform，启动定位，启动rviz

ros2 launch lamapf_and_gazebo localization_full.launch.py

启动定位后需要在rviz上设置初始位置，
在运行键盘控制使得amcl慢慢修复初始位置的微小误差

但键盘控制会和局部控制器抢控制权，因此调整完定位初始误差后就关掉

跑起来了，但速度有点快，结尾姿态调整时还振荡，左右摆头

通过调整速度参数应该能避免振荡

降低了速度上限后，振荡现象好像消失了, 继续观察

11-25
局部控制器测试容易走直线就停不下来


通过launch启动局部控制器的测试

ros2 launch lamapf_and_gazebo local_controller_test.launch.py >  my_node.log 2>&1
