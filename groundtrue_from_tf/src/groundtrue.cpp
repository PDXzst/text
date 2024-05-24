//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include <nav_msgs/Odometry.h>
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"tf_sub");
    ros::NodeHandle nh;
    // 3.创建 TF 订阅节点
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("ground_state", 10);
    while (ros::ok())
    {
    // 4.生成一个坐标点(相对于子级坐标系)
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "body";
        point_laser.header.stamp = ros::Time();
        point_laser.point.x = 0.01;
        point_laser.point.y = -0.03;
        point_laser.point.z = 0;
        geometry_msgs::PointStamped point_base;        
    // 5.转换坐标点(相对于父级坐标系)
        //新建一个坐标点，用于接收转换结果  
        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {
            point_base = buffer.transform(point_laser,"camera_init");
            ROS_INFO("坐标点相对于 world 的坐标为:(%.2f,%.2f,%.2f)",point_base.point.x,point_base.point.y,point_base.point.z);

        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常:%s",e.what());
        }
        nav_msgs::Odometry groundtrue;
        groundtrue.header.stamp = ros::Time::now();
        groundtrue.header.frame_id = "odom";
        groundtrue.pose.pose.position.x = point_base.point.x;
        groundtrue.pose.pose.position.y = point_base.point.y;
        groundtrue.pose.pose.position.z = point_base.point.z;
        odom_pub.publish(groundtrue);
        ros::spinOnce();
    }


    return 0;
}
