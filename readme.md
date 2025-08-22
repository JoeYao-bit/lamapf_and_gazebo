 
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
ros2 run lamapf_and_gazebo lamapf_planner_node
```

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

## 生成地图

gazebo sim 选择俯视图，正交视角
截图后生成规划用地图时考虑栅格对应的分辨率，并顺时针转90度，以对其坐标系
涂白（255,255,255）区域为可通行区域

TODO:

增加单位路径上的碰撞检测
有碰撞风险则等待至无碰撞或超时报错，超时后更新地图重启规划。

将每个机器人作为一个node计算控制指令

通过ros的跨平台多节点通信机制实现不同机器人与控制中心的通信

通过服务实现总控和机器人间通信