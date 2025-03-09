#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
import io  # 必须导入 io 模块
from turtlesim.msg import Pose

class ROSToMQTT:
    def __init__(self):
        rospy.init_node('ros_to_mqtt_bridge')

        # 读取 YAML 配置文件中的参数（必须在 init_node 之后）
        self.mqtt_broker = rospy.get_param('mqtt/broker', "192.168.18.80")
        self.mqtt_port = rospy.get_param('mqtt/port', 1883)
        self.topic_pose = rospy.get_param('mqtt/topic_pose', "/turtle1/pose")
        self.mqtt_pose = rospy.get_param('mqtt/mqtt_pose', "pingpong/turtle1/pose")
        
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
        self.mqtt_client.loop_start()

        rospy.Subscriber(self.topic_pose, Pose, self.ros_pose_callback)
        rospy.loginfo(f"ROS→MQTT 桥接已启动: {self.mqtt_broker}:{self.mqtt_port}, 发布: {self.mqtt_pose}")

    def ros_pose_callback(self, msg):
        try:
            # 创建缓冲区对象并序列化消息
            buffer = io.BytesIO()
            msg.serialize(buffer)  # 将缓冲区作为参数传递
            data = buffer.getvalue()

            # 发布到MQTT
            self.mqtt_client.publish(self.mqtt_pose, data)
            # rospy.loginfo("发送到MQTT: x=%.2f, y=%.2f", msg.x, msg.y)
        except Exception as e:
            rospy.logerr("序列化失败: %s", e)

    def shutdown(self):
        self.mqtt_client.loop_stop()
        rospy.loginfo("ROS→MQTT 桥接已关闭")

if __name__ == '__main__':
    bridge = ROSToMQTT()
    rospy.on_shutdown(bridge.shutdown)
    rospy.spin()
