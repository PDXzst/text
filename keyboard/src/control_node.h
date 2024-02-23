#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/WaypointList.h>
#include <rosgraph_msgs/Clock.h>
#include <mavros_msgs/WaypointReached.h>

using namespace std;
class PX4_Control
{
public:
    PX4_Control();
    // void IMU_Data();
    void State_Callback(const mavros_msgs::State::ConstPtr &msg);
    void Local_Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void WayPoint_Callback(const mavros_msgs::WaypointList::ConstPtr &msg);
    void WaypointReached_Pose_Callback(const mavros_msgs::WaypointReached::ConstPtr &msg);

    ros::NodeHandle px4_control_node;
    mavros_msgs::State Current_State;
    geometry_msgs::PoseStamped Local_Pose;
    mavros_msgs::WaypointList WaypointList;
    mavros_msgs::WaypointReached WaypointReached;

    ros::Subscriber State_Sub;
    ros::Subscriber Local_Pose_Sub;    
    ros::Subscriber WayPoint_Sub;  
    ros::Subscriber WaypointReached_Sub;  


    ros::Publisher Target_Pos_Pub;

    ros::ServiceClient Arming_Client;
    ros::ServiceClient Set_Mode_Client;

};

PX4_Control::PX4_Control()
{
    State_Sub = px4_control_node.subscribe("mavros/state", 10, &PX4_Control::State_Callback, this);
    Local_Pose_Sub = px4_control_node.subscribe("mavros/local_position/pose", 10, &PX4_Control::Local_Pose_Callback, this);
    WayPoint_Sub = px4_control_node.subscribe("/mavros/mission/waypoints", 10, &PX4_Control::WayPoint_Callback, this);
    WaypointReached_Sub = px4_control_node.subscribe("/mavros/mission/reached", 10, &PX4_Control::WaypointReached_Pose_Callback, this);


    Target_Pos_Pub = px4_control_node.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);


    Arming_Client = px4_control_node.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    Set_Mode_Client = px4_control_node.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void PX4_Control::State_Callback(const mavros_msgs::State::ConstPtr &msg)
{
    Current_State = *msg;
}
void PX4_Control::Local_Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Local_Pose = *msg;
}

    void PX4_Control::WayPoint_Callback(const mavros_msgs::WaypointList::ConstPtr &msg)
    {
WaypointList = *msg;
    }
    void PX4_Control::WaypointReached_Pose_Callback(const mavros_msgs::WaypointReached::ConstPtr &msg)
    {
WaypointReached= *msg;
    }
    
