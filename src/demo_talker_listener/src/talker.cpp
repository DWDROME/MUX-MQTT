#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker"); // 初始化ROS节点
    ros::NodeHandle nh; // 创建ROS节点句柄

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("/ping/ros", 1000); // 创建Publisher，发布到"chatter"话题
    ros::Rate loop_rate(2); 

    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello ROS! " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str()); // 在终端输出消息
        chatter_pub.publish(msg); // 发布消息

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}

