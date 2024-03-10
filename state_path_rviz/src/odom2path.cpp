#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
ros::Publisher path_pub;    // 发布path的节点
ros::Subscriber odom_sub;   // 接收odometry的节点
ros::Publisher error_pub; 

nav_msgs::Path path;        // 发布的路径
bool init_flag = true;      // path初始化标记位
int num = 0;
int flag = 0;
double error = 0;
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    std_msgs::Float64 error_msgs;
    if(flag<100)
    {
        flag ++;
    }
    if(init_flag){
        path.header = odom->header;
        path.header.frame_id = "odom";
        init_flag = false;
    }
    if(flag>=100){
        num ++;
        error += odom->pose.pose.position.x*odom->pose.pose.position.x+odom->pose.pose.position.y*odom->pose.pose.position.y+odom->pose.pose.position.z*odom->pose.pose.position.z;
        error_msgs.data = error/num;
        error_pub.publish(error_msgs);
    }
    

    geometry_msgs::PoseStamped pose_stamped_this;
    pose_stamped_this.header = odom->header;
    pose_stamped_this.pose = odom->pose.pose;

    path.poses.push_back(pose_stamped_this);
    path_pub.publish(path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_to_path");
    // 创建节点句柄，并设置命名空间
    ros::NodeHandle nh;

    // 定义参数变量
    std::string odom_topic;
    std::string path_topic;
    std::string error_topic;
    ros::param::get("~odom_topic", odom_topic);
    ros::param::get("~path_topic", path_topic);
    ros::param::get("~error_topic", error_topic);
    
    // 订阅odometry消息
    odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 100, odomCallback);
    // 发布path消息
    path_pub = nh.advertise<nav_msgs::Path>(path_topic, 100, true);

    error_pub = nh.advertise<std_msgs::Float64>(error_topic, 100, true);
    ros::spin();
    return 0;
}
