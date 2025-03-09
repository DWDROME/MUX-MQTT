#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
import struct  # 新增导入
from turtlesim.msg import Pose

class MQTTToROS:
    def __init__(self):
        rospy.init_node('mqtt_to_ros_bridge')

        # 读取 YAML 配置参数
        self.mqtt_broker = rospy.get_param('mqtt/broker', "192.168.18.80")
        self.mqtt_port = rospy.get_param('mqtt/port', 1883)
        self.mqtt_pose = rospy.get_param('mqtt/mqtt_pose', "pingpong/turtle1/pose")
        self.topic_pose = rospy.get_param('mqtt/topic_pose', "/turtle1/pose")

        # ROS 发布者
        self.pose_pub = rospy.Publisher(self.topic_pose, Pose, queue_size=10)

        # MQTT 客户端
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.mqtt_callback
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
        self.mqtt_client.subscribe(self.mqtt_pose)
        self.mqtt_client.loop_start()

        rospy.loginfo(f"MQTT→ROS 桥接已启动: {self.mqtt_broker}:{self.mqtt_port}, 订阅: {self.mqtt_pose} → 发布到 ROS: {self.topic_pose}")

    def mqtt_callback(self, client, userdata, msg):  
        try:
            pose = Pose()
            pose.deserialize(msg.payload)
            self.pose_pub.publish(pose)
            # rospy.loginfo("接收并转发到 ROS: x=%.2f, y=%.2f", pose.x, pose.y)
        except TypeError as e:
            rospy.logerr("数据类型错误: %s", e)
        except struct.error as e:
            rospy.logerr("二进制数据不完整: %s", e)
        except Exception as e:
            rospy.logerr("未知错误: %s", e)

    def shutdown(self):
        self.mqtt_client.loop_stop()
        rospy.loginfo("MQTT→ROS 桥接已关闭")

if __name__ == '__main__':
    bridge = MQTTToROS()
    rospy.on_shutdown(bridge.shutdown)
    rospy.spin()
