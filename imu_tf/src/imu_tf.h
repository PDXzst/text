#ifndef IMU_TF_H
#define IMU_TF_H
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
namespace imu_tf{
class Imu_Tf{
public:
    Imu_Tf(ros::NodeHandle &_nh);
private:
    ros::Publisher pub;
    ros::Publisher groundtrue_vel_pub;
    ros::NodeHandle nh; 
    tf::Quaternion world_tf;
    ros::Subscriber groundture_sub;
    void groundture_cb(const nav_msgs::Odometry::ConstPtr& groundture_data_p);
    geometry_msgs::Vector3 transform_into_robot(tf::Quaternion world_quaternion,tf::Quaternion point_tf);
    bool last_odom_received = false;
    unsigned int seed = 0;
    ros::Time last_time;
    nav_msgs::Odometry last_odom;
    double GuassianKernel(double mu, double sigma);

    double mean;
    double stddev;
}; 

}


#endif