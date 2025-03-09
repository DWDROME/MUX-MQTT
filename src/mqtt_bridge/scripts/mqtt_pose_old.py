#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
import io
from turtlesim.msg import Pose

# MQTT Broker 地址和端口，请根据实际情况修改
MQTT_BROKER = "192.168.18.80"
MQTT_PORT = 1883

# MQTT 主题
MQTT_TOPIC_SUB = "pingpong/turtle1/pose"  # 用于订阅，从MQTT Broker接收turtlesim/Pose消息
MQTT_TOPIC_PUB = "pingpong/turtle1/pose"  # 用于发布，将ROS中的turtlesim/Pose消息发送到MQTT Broker

# 全局MQTT客户端对象
mqtt_client = None

# MQTT 回调函数
# 接收到MQTT消息后，直接使用二进制数据反序列化为ROS消息，并发布到ROS主题 /turtle1/pose
def on_message(client, userdata, msg):
    try:
        pose = Pose()
        # 直接使用ROS消息的反序列化方法
        pose.deserialize(msg.payload)
        pub.publish(pose)
        # rospy.loginfo("Published Pose from MQTT: %s", pose)
    except Exception as e:
        rospy.logerr("Failed to deserialize MQTT message: %s", e)

# ROS回调函数
# 当接收到ROS主题 /turtle1/pose 消息时，将其序列化为二进制数据后发布到MQTT Broker
def ros_pose_callback(msg):
    try:
        buffer = io.BytesIO()
        msg.serialize(buffer)
        mqtt_client.publish(MQTT_TOPIC_PUB, buffer.getvalue())
        # rospy.loginfo("Published Pose to MQTT: %s", msg)
    except Exception as e:
        rospy.logerr("Failed to publish ROS message to MQTT: %s", e)

if __name__ == '__main__':
    rospy.init_node('mqtt_ros_pose_bridge', anonymous=True)
    
    # ROS发布器：用于发布从MQTT接收到的Pose消息到ROS主题 /turtle1/pose
    pub = rospy.Publisher('/turtle1/pose', Pose, queue_size=10)
    # ROS订阅器：用于监听ROS主题 /turtle1/pose，将消息转发到MQTT Broker
    rospy.Subscriber('/turtle1/pose', Pose, ros_pose_callback)
    
    # 初始化MQTT客户端
    mqtt_client = mqtt.Client()
    mqtt_client.on_message = on_message
    # 连接到MQTT Broker
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    # 订阅指定的MQTT主题
    mqtt_client.subscribe(MQTT_TOPIC_SUB)
    # 启动MQTT客户端的循环线程
    mqtt_client.loop_start()
    
    # ROS循环等待回调
    rospy.spin()
    
    mqtt_client.loop_stop()
