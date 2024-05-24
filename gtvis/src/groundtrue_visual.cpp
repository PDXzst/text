#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_publisher");
    ros::NodeHandle n;
    std::string csv_file;
    ros::param::get("~csv_file", csv_file);
    std::string odom_topic;
    ros::param::get("~odom_topic", odom_topic);
    ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>(odom_topic, 10000000);
    ros::Time now =ros::Time::now();
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << "Error opening file " << csv_file << std::endl;
    }
    ros::Rate loop_rate(120.0048002);
    std::string line;
    int currentRow = 0;
    while (std::getline(file, line)) {
        if (currentRow >= 7) {
            bool flat = true;
            std::istringstream ss(line);
            std::vector<double> row;
            std::string cell;
            int currentCol = 0;
            while (std::getline(ss, cell, ',')) {
                if (currentCol <= 8) {
                    try {
                        row.push_back(std::stod(cell));
                    } catch (const std::exception& e) {
                        //std::cerr << "Error converting cell to double: " << e.what() << std::endl;
                        //std::cout << "row:" <<currentRow << "col" << currentCol << std::endl;
                        flat = false;
                    }
                }
                else if (currentCol > 8){
                    break;
                }
                currentCol++;
            }
            if(flat){
                nav_msgs::Odometry odometry;
            odometry.header.stamp = now + ros::Duration(row[1]);
            odometry.header.frame_id = "odom";
            //odometry.child_frame_id = odom_topic;
            odometry.pose.pose.orientation.x = row[2];
            odometry.pose.pose.orientation.y = row[3];
            odometry.pose.pose.orientation.z = row[4];
            odometry.pose.pose.orientation.w = row[5];
            odometry.pose.pose.position.x =    row[6]/1000;
            odometry.pose.pose.position.y =    row[7]/1000;
            odometry.pose.pose.position.z =    row[8]/1000;
            
            pub_odom.publish(odometry);
            }
            
        }
        currentRow++;
        ros::spinOnce();
        loop_rate.sleep();
    }


    
    
    ros::spin();
}
