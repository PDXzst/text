# 使用说明
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
|outdoor1.world|室外环境outdoor1|

![](/photos/indoor1截图.png)  
indoor1截图  
![](/photos/indoor2截图.png)  
indoor2截图  
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
参考自`https://www.guyuehome.com/43667`  
本人添加了`u``o`键位旋转的功能，还存在bug，望周知  

## 功能包offboard_pkg
请添加到ros工作空间  
速度控制无人机  
参考自`https://blog.csdn.net/HuangChen666/article/details/129642230`  

## 地图更新
- [X] 更改`models/cpr_office` `models/cpr_office_construction`  
- [X] 更改`worlds/indoor1.world` `indoor2.world`  

