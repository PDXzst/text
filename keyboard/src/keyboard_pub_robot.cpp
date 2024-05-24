#include <istream>
#include <iostream>
#include <termio.h>
#include <cstring>
#include <ros/ros.h>
#include <std_msgs/Char.h>

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

    input = std::cin.get();

    tcsetattr(0, TCSANOW, &stored_settings);
    return input;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_publisher");
    ros::NodeHandle nh;

    ros::Publisher keyboard_pub = nh.advertise<std_msgs::Char>("keyboard_input_robot", 1);

    while (ros::ok())
    {
        // 获取键盘输入
        char input = scanKeyboard();

        // 创建键盘消息
        std_msgs::Char msg;
        msg.data = input;

        // 发布键盘消息
        keyboard_pub.publish(msg);

        // 循环等待回调函数执行
        ros::spinOnce();
        if (input == 'q')
            break;
    }

    return 0;
}

