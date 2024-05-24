#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_to_odometry");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odometry_topic", 10);

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("camera_init", "body", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "camera_init";
        odom.child_frame_id = "body";

        // 设置位置
        odom.pose.pose.position.x = transform.getOrigin().x();
        odom.pose.pose.position.y = transform.getOrigin().y();
        odom.pose.pose.position.z = transform.getOrigin().z();

        // 设置姿态
        odom.pose.pose.orientation.x = transform.getRotation().x();
        odom.pose.pose.orientation.y = transform.getRotation().y();
        odom.pose.pose.orientation.z = transform.getRotation().z();
        odom.pose.pose.orientation.w = transform.getRotation().w();

        odom_pub.publish(odom);

        rate.sleep();
    }

    return 0;
}