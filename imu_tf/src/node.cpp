#include "imu_tf.h"
#include <ros/ros.h>


using namespace imu_tf;

int main(int argc,char** argv){
    ros::init(argc,argv,"imu_tf_node");
    ros::NodeHandle nh;
    Imu_Tf filer(nh);
    ros::spin();
}