#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
from std_msgs.msg import String

# 在主机B上接收MQTT Broker发送的control_mode数据（std_msgs/String），并发布到ROS的控制模式话题

class MQTTToControlMode:
    def __init__(self):
        rospy.init_node('mqtt_to_control_mode_bridge', anonymous=True)

        # 读取 YAML 配置参数
        self.mqtt_broker = rospy.get_param('mqtt/broker', "192.168.18.80")
        self.mqtt_port = rospy.get_param('mqtt/port', 1883)
        self.mqtt_control_mode = rospy.get_param('mqtt/mqtt_control_mode', "pingpong/control_mode")
        self.topic_control_mode = rospy.get_param('mqtt/topic_control_mode', "/control_mode")

        # ROS 发布者
        self.pub = rospy.Publisher(self.topic_control_mode, String, queue_size=10)

        # MQTT 客户端
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
        self.mqtt_client.subscribe(self.mqtt_control_mode)
        self.mqtt_client.loop_start()

        rospy.loginfo(f"MQTT→ROS 桥接已启动: {self.mqtt_broker}:{self.mqtt_port}, 订阅 {self.mqtt_control_mode} → 发布到 ROS: {self.topic_control_mode}")

    def on_message(self, client, userdata, msg):
        try:
            # 解码MQTT消息内容为字符串
            control_mode = msg.payload.decode()
            # 构造ROS消息
            ros_msg = String(data=control_mode)
            # 发布到ROS的控制模式话题
            self.pub.publish(ros_msg)
            rospy.loginfo("已发布控制模式: %s", ros_msg.data)
        except Exception as e:
            rospy.logerr("处理MQTT消息失败: %s", e)

    def shutdown(self):
        self.mqtt_client.loop_stop()
        rospy.loginfo("MQTT→ROS 桥接已关闭")

if __name__ == '__main__':
    bridge = MQTTToControlMode()
    rospy.on_shutdown(bridge.shutdown)
    rospy.spin()
