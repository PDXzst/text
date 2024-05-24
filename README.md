# 使用说明 Made by HITsz zst
## Add two indoor worlds and one outdoor world
按照说明解压`models.zip`和`worlds.zip`后在`launch`文件中，修改  
```
 <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/indoor2.world"/>
```

<div align="center">
  
|worlds|details|
|:---:|:---:|
|indoor1.world|室内环境indoor1|
|indoor2.world|室内环境indoor2|
|indoor3.world|室内环境indoor3|
|indoor4.world|室内环境indoor4|
|outdoor.world|室外环境outdoor(outdoor1-mini)|
|outdoor1.world|室外环境outdoor1|

![](/photos/indoor1截图.png)  
indoor1截图  
![](/photos/indoor2截图.png)  
indoor2截图  
![](/photos/indoor2截图.png)  
indoor3截图  
![](/photos/indoor2截图.png)  
indoor4截图  
![](/photos/outdoor截图.png)  
outdoor截图  
![](/photos/outdoor1截图.png)  
outdoor1截图  

</div>


### model.zip
请解压至`models`环境变量对应路径  
在PX4环境中对应：`~/PX4-Autopilot/Tools/sitl_gazebo/models`  
### worlds.zip
请解压至`worlds`环境变量对应路径  
在PX4环境中对应：`~/PX4-Autopilot/Tools/sitl_gazebo/worlds`  
## 添加groundtruth的方法
在无人机:`iris.sdf`文件末尾加入  
```
<plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>50.0</updateRate>
      <bodyName>base_link</bodyName>
      <topicName>ground_truth/state</topicName>
      <gaussianNoise>0.01</gaussianNoise>
      <frameName>world</frameName>
      <xyzOffsets>0 0 0</xyzOffsets>
      <rpyOffsets>0 0 0</rpyOffsets>
</plugin>
```
可参考给出的`iris.sdf`文件  

在无人车`scout_v2.xacro`末尾加上上述`demo`注意有`<gazebo> </gazebo>`  

可以通过`<topicName>ground_truth/state</topicName>`改变`topic`  

## 功能包keyboard
请添加到ros工作空间  
位置控制无人机  
```
rosrun keyboard control_node
rosrun keyboard keyboard_pub
```
打开两个终端依次运行上述程序，在`keyboard_ pub`中输入控制按键，`control_node`无需进一步操作  
参考自`https://www.guyuehome.com/43667`  
本人添加了`u(左旋)``o(右旋)`键位旋转的功能  
无人机控制指令控制的话题：`/mavros/setpoint_position/local`  
录制bag：  
```
rosbag record /mavros/setpoint_position/local
```
注意：由于px4无人机通过offboard进入板载运行，需要先起飞才能控制。录制bag的启动方式后仍然需要先运行`rosrun keyboard control_node`再手动`ctrl c`  

3.10更新  
```
rosrun keyboard control_node_robot
rosrun keyboard keyboard_pub_robot
```
可以相对机器人坐标系进行控制  
## 无人车控制

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```
如果上述运行失败，请尝试安装如下依赖(注意：根据安装的ros版本不同`melodic`处可能为`noetic`可以在终端输入`rosversion -d`查看)：  
```
sudo apt-get install ros-melodic-ros-control
sudo apt-get install ros-melodic-ros-controllers
sudo apt-get install ros-melodic-gazebo-ros
sudo apt-get install ros-melodic-gazebo-ros-control
sudo apt-get install ros-melodic-joint-state-publisher-gui 
sudo apt-get install ros-melodic-teleop-twist-keyboard 
```
控制的话题：`/cmd_vel`  
录制bag：  
```
rosbag record /cmd_vel
```
注意：无人车只能通过控制速度控制其运动，录制bag运动时可能不会每次运动轨迹都一样  

## 功能包offboard_pkg
请添加到ros工作空间  
速度控制无人机  
参考自`https://blog.csdn.net/HuangChen666/article/details/129642230`  

## 功能包state_path_rviz
用于订阅`nav_msgs::Odometry`在`rviz`中可视化，适用于机器人的`groundture`  
`odom2path`在`rviz`中为`/Path`曲线  
`odom2pose`可在`rviz`中生成`Marker`型的`(椭)球`和`线`，两者一起输出可形成`串珠型曲线`  
使用方法：  
1、更改`launch`中对应的`groundture话题`  
2、启动  
```
roslaunch state_path_rviz odom2path.launch
```
```
roslaunch state_path_rviz odom2pose.launch
```
两个程序互不冲突，可以一起运行，看看哪种曲线比较好  
另外在`odom2pose`中，我目前设置的是图形只保留5s，可以在`.py`中自行设置保留时间，也可以DIY球和线的大小颜色等，我加了注释很容易修改  
## 功能包imu_tf
```
roslaunch imu_tf imu_tf_single.launch
```

## 功能包gtvis
将`groundtrue`从csv文件中可视化到rviz平台  
将数据放入`csv_data`文件夹中，注意更改`launch`文件的文件路径的  
师兄你给我的那三个数据直接按照你发给我的名称放进去就行了  
由于RVIZ不能按照数据的`stamp`对应输出数据，所以我做了如下纠正：  
因为csv文件数据的频率为120Hz，所以我设置了数据发布频率为120Hz来实现同步  
## uav.bag car.bag
测试时录制的`bag`文件  

## 注意
更新地图的话需要将`models`文件夹也一起更新(更新方法:替代原文件)  

## 更新日志
- [X] 更改`models/cpr_office` `models/cpr_office_construction`.更改`worlds/indoor1.world` `indoor2.world`.2024-2-23  
- [X] 更新`indoor4.world``indoor1.world`.成功添加无人机位置控制的转向功能.2024-2-24  
- [X] 2024-4-11 添加imu_tf功能包
- [X] 2024-4-13 日更改`models/cpr_office`和`worlds/indoor1.world`需要使用请重新粘贴  
- [X] 2024-4-15 日更改`models/cpr_office`和`worlds/indoor1.world`,新增大量墙壁壁画，需要使用请重新粘贴  

