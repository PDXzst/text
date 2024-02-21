# 使用说明
## two indoor worlds and one outdoor worlds
按照如下两个步骤操作，然后在`launch`文件中，修改  
```
 <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/indoor2.world"/>
```

<div align="center">
  
|worlds|details|
|:---:|:---:|
|indoor1.world|室内环境indoor1|
|indoor2.world|室内环境indoor2|
|outdoor1.world|室外环境outdoor1|

![](/photos/indoor1截图.png)  
indoor1截图  
![](/photos/indoor2截图.png)  
indoor2截图  
![](/photos/outdoor1截图.png)  
outdoor1截图  

</div>


### model.zip
请将文件夹解压至models环境变量对应路径  
在PX4环境中应该对应：`~/PX4-Autopilot/Tools/sitl_gazebo/models`  
### worlds.zip
请将文件夹解压至worlds环境变量对应路径  
在PX4环境中应该对应：`/home/robot/PX4-Autopilot/Tools/sitl_gazebo/worlds`  \
## 添加groundtruth的方法
在无人机/车对应的`.sdf`文件末尾加入  
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
如图：  

<div align="center">
  
![](/photos/groundture.png)

</div>

我在文件`crazyflie.sdf`末尾添加以后就能够通过`rostopic echo /ground_truth/state`获得groundtruth  

可以通过`<topicName>ground_truth/state</topicName>`改变`topic`  
