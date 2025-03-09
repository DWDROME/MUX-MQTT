#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iq160");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::String>("/ping/ros", 1);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "Hello MQTT";
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
