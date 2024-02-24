#include "control_node.h"
#include <istream>
#include <termio.h>
#include <cstring>
#include <std_msgs/Char.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)  
{  
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());  
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());  
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());  
  
    Eigen::Quaterniond q = rollAngle*yawAngle*pitchAngle;  
    //cout << “Euler2Quaternion result is:” <<endl;  
    //cout << ”x = ” << q.x() <<endl;  
    //cout << “y = ” << q.y() <<endl;  
    //cout << ”z = ” << q.z() <<endl;  
    //cout << ”w = ” << q.w() <<endl<<endl;  
    return q;  
}  

Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    //cout << "Quaterniond2Euler result is:" <<endl;
    //cout << "x = "<< euler[2] << endl ;
    //cout << "y = "<< euler[1] << endl ;
    //cout << "z = "<< euler[0] << endl << endl;
    euler[0] = atan2(sin(euler[0]), cos(euler[0]));
    return euler;
}


int test_cnt = 0;

// 获取键盘输入字符
char scanKeyboard()
{
    char input;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    input = cin.get();

    tcsetattr(0, TCSANOW, &stored_settings);
    return input;
}

// 发布目标位置
void publishTargetPosition(const geometry_msgs::PoseStamped &pose, ros::Publisher &targetPosPub, ros::Rate &loopRate)
{
    targetPosPub.publish(pose);
    // ros::spinOnce();
    // loopRate.sleep();
}

// 检查是否满足起飞条件
bool checkTakeoffConditions(const PX4_Control &px4_control)
{
    return px4_control.Current_State.armed;
}

void keyboardCallback(const std_msgs::Char::ConstPtr &msg, geometry_msgs::PoseStamped &pose, const PX4_Control &px4_control, float target_altitude, ros::Publisher &targetPosPub, ros::Rate &loopRate, bool &control_pose_flag, bool &exit_flag)
{
    Eigen::Vector3d xyz = Quaterniond2Euler(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    
    if (checkTakeoffConditions(px4_control))
    {
        if (control_pose_flag && px4_control.Local_Pose.pose.position.z < target_altitude * 0.9)
        {
            // 起飞后保持初始位置
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = target_altitude;
        }
        else
        {
            control_pose_flag = false;
            // 根据键盘消息执行相应操作
            char input = msg->data;
            switch (input)
            {
            case 'w':
            case 'W':
                // 执行向上的操作
                pose.pose.position.z += 0.1;
                std::cout << pose.pose.position.z << std::endl;
                break;
            case 's':
            case 'S':
                // 执行向下的操作
                if (pose.pose.position.z > 0.1)
                {
                    pose.pose.position.z -= 0.1;
                }
                break;
            case 'l':
            case 'L':
                // 执行向右平移的操作
                pose.pose.position.y -= 0.05;
                break;
            case 'i':
            case 'I':
                // 执行向前平移的操作
                pose.pose.position.x += 0.05;
                break;
            case 'j':
            case 'J':
                // 执行向左平移的操作
                pose.pose.position.y += 0.05;
                break;
            case 'k':
            case 'K':
                // 执行向后平移的操作
                pose.pose.position.x -= 0.05;
                break;
            case 'u':
            case 'U':
                // 执行逆时针旋转
                xyz[0] += 0.1;
                
                break;
            case 'o':
            case 'O':
                // 执行顺时针旋转
                xyz[0] -= 0.1;
                break;
            case 'q':
            case 'Q':
                // 执行退出的操作
                exit_flag = true;
                break;
            default:
                break;
            }
        }
    }
    
    Eigen::Quaterniond xyzw = euler2Quaternion(xyz[0],xyz[1],xyz[2]);
    
    pose.pose.orientation.x = xyzw.x();
    pose.pose.orientation.y = xyzw.y();
    pose.pose.orientation.z = xyzw.z();
    pose.pose.orientation.w = xyzw.w();
    
}

int main(int argc, char **argv)
{
    float target_altitude = 1.2;
    float radius = 200;
    ros::init(argc, argv, "px4_lidar");
    PX4_Control px4_control;

    ros::Rate loop_rate(100);

    while (ros::ok() && px4_control.Current_State.connected)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = target_altitude;
    // 发送一些初始的目标位置
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        publishTargetPosition(pose, px4_control.Target_Pos_Pub, loop_rate);
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok() && !px4_control.Current_State.armed)
    {
        if (px4_control.Current_State.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (px4_control.Set_Mode_Client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else if (!px4_control.Current_State.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (px4_control.Arming_Client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        px4_control.Target_Pos_Pub.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    bool control_pose_flag = true;
    bool exit_flag = false;
    ros::NodeHandle keyboard_sub_node;
    ros::Subscriber keyboard_sub = keyboard_sub_node.subscribe<std_msgs::Char>("keyboard_input", 1,
        [&](const std_msgs::Char::ConstPtr& msg) {
            keyboardCallback(msg, pose, px4_control, target_altitude, px4_control.Target_Pos_Pub, loop_rate, control_pose_flag, exit_flag);
        });
    while (ros::ok())
    {
        if (exit_flag)
        {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep(); 
        px4_control.Target_Pos_Pub.publish(pose);
    }

    return 0;
}
