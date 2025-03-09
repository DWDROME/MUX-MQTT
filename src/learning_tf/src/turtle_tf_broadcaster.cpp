#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br;  //static 关键字 确保只创建一次
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );  //平面，z为0

  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);   //转换为四元数，适应tf
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
  /*StampedTransform 代表带时间戳的坐标变换：
    transform：变换信息（位置 + 旋转）。
    ros::Time::now()：当前时间戳。
    "world"：父坐标系，乌龟的运动是相对于 world。
    turtle_name：子坐标系，乌龟的名称作为 tf 变换的目标。*/
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};   //阻止第二只海龟产生
  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};