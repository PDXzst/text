#include "imu_tf.h"
namespace imu_tf{
void Imu_Tf::groundture_cb(const nav_msgs::Odometry::ConstPtr& groundture_data_p){
    if(!last_odom_received){//初始化检测
        last_odom_received = true;
        last_odom = *groundture_data_p;
        return;
    }
    //计算在世界座标系下的加速度
    double dt = (groundture_data_p->header.stamp - last_odom.header.stamp).toSec();
    double dv_x = groundture_data_p->twist.twist.linear.x - last_odom.twist.twist.linear.x;
    double dv_y = groundture_data_p->twist.twist.linear.y - last_odom.twist.twist.linear.y;
    double dv_z = groundture_data_p->twist.twist.linear.z - last_odom.twist.twist.linear.z;

    double acc_x = dv_x / dt;
    double acc_y = dv_y / dt;
    double acc_z = dv_z / dt;
    last_odom = *groundture_data_p;
    tf::Quaternion acc_world(acc_x,acc_y,acc_z,0);

    //robot四元数
    world_tf.setX(groundture_data_p->pose.pose.orientation.x);
    world_tf.setY(groundture_data_p->pose.pose.orientation.y);
    world_tf.setZ(groundture_data_p->pose.pose.orientation.z);
    world_tf.setW(groundture_data_p->pose.pose.orientation.w);
    //在robot座标系下的加速度
    geometry_msgs::Vector3 acc_robot = transform_into_robot(world_tf,acc_world);
    //发布msgs
    sensor_msgs::Imu imu_msgs;
    imu_msgs.angular_velocity = groundture_data_p->twist.twist.angular;
    imu_msgs.header.stamp = groundture_data_p->header.stamp;
    imu_msgs.linear_acceleration = acc_robot;
    //--添加高斯噪声--//
    imu_msgs.angular_velocity.x+=GuassianKernel(mean,stddev);
    imu_msgs.angular_velocity.y+=GuassianKernel(mean,stddev);
    imu_msgs.angular_velocity.z+=GuassianKernel(mean,stddev);
    imu_msgs.linear_acceleration.x+=GuassianKernel(mean,stddev);
    imu_msgs.linear_acceleration.y+=GuassianKernel(mean,stddev);
    imu_msgs.linear_acceleration.z+=GuassianKernel(mean,stddev);

    pub.publish(imu_msgs);
    //grountrue的速度在robot座标系下的值，用于上述imu数据测试用
    // nav_msgs::Odometry true_vel;
    // tf::Quaternion vel_world(groundture_data_p->twist.twist.linear.x,groundture_data_p->twist.twist.linear.y,groundture_data_p->twist.twist.linear.z,0);
    // true_vel.twist.twist.linear = transform_into_robot(world_tf,vel_world);
    // groundtrue_vel_pub.publish(true_vel);
}
//利用四元数将世界座标系转化为robot座标系
geometry_msgs::Vector3 Imu_Tf::transform_into_robot(tf::Quaternion world_tf_,tf::Quaternion point_tf){
    tf::Quaternion tansformed_point = world_tf_.inverse()*point_tf*world_tf_;
    geometry_msgs::Vector3 point;
    point.x = tansformed_point.getX();
    point.y = tansformed_point.getY();
    point.z = tansformed_point.getZ();
    return point;
}
//高斯噪声
double Imu_Tf::GuassianKernel(double mu, double sigma)
{
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}


Imu_Tf::Imu_Tf(ros::NodeHandle &_nh)
{
    std::string groundtrue_topic;
    std::string imu_topic;
    ros::param::get("~groundtrue_topic", groundtrue_topic);
    ros::param::get("~imu_topic", imu_topic);
    ros::param::get("~mean",mean);
    ros::param::get("~stddev",stddev);
    nh = _nh;
    groundture_sub=nh.subscribe<nav_msgs::Odometry>(groundtrue_topic,100,&Imu_Tf::groundture_cb,this);
    pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 100, true);
    //groundtrue_vel_pub = nh.advertise<nav_msgs::Odometry>("true_vel", 100, true);
}
}