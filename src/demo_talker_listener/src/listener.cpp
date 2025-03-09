#include "ros/ros.h"
#include "std_msgs/String.h"

// 回调函数，处理接收到的消息
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener"); // 初始化ROS节点
    ros::NodeHandle nh; // 创建ROS节点句柄

    ros::Subscriber sub = nh.subscribe("/pong/ros", 1000, chatterCallback); // 订阅"chatter"话题
    ros::spin(); // 循环等待回调函数执行
    return 0;
}
