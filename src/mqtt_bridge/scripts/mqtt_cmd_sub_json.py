#!/usr/bin/env python3
import json
import paho.mqtt.client as mqtt
import rospy
from geometry_msgs.msg import Twist

# 在主机B上接收json信息，发布cmd_vel到主机A的小海龟上 

# MQTT 回调函数
# def on_message(client, userdata, msg):
#     try:
#         data = json.loads(msg.payload.decode())
#         twist = Twist()
#         twist.linear.x = data["linear"]["x"]
#         twist.linear.y = data["linear"]["y"]
#         twist.linear.z = data["linear"]["z"]
#         twist.angular.x = data["angular"]["x"]
#         twist.angular.y = data["angular"]["y"]
#         twist.angular.z = data["angular"]["z"]
#         pub.publish(twist)
#         rospy.loginfo("Published Twist: %s", twist)
#     except Exception as e:
#         rospy.logerr("Failed to parse MQTT message: %s", e)

class MQTTToCmdVel:
    def __init__(self):
        rospy.init_node('mqtt_to_cmd_vel_bridge', anonymous=True)

        # 读取 YAML 配置参数
        self.mqtt_broker = rospy.get_param('mqtt/broker', "192.168.18.80")
        self.mqtt_port = rospy.get_param('mqtt/port', 1883)
        self.mqtt_cmd_vel = rospy.get_param('mqtt/mqtt_cmd_vel_sub', "pingpong/cmd_vel")
        self.topic_cmd_vel = rospy.get_param('mqtt/topic_cmd_vel', "/remote_cmd_vel")

        # ROS 发布者
        self.pub = rospy.Publisher(self.topic_cmd_vel, Twist, queue_size=10)

        # MQTT 客户端
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
        self.mqtt_client.subscribe(self.mqtt_cmd_vel)
        self.mqtt_client.loop_start()

        rospy.loginfo(f"MQTT→ROS 桥接已启动: {self.mqtt_broker}:{self.mqtt_port}, 订阅 {self.mqtt_cmd_vel} → 发布到 ROS: {self.topic_cmd_vel}")

    def on_message(self, client, userdata, msg):
        try:
            # 确保解码为UTF-8字符串
            payload = msg.payload.decode("utf-8")
            rospy.logdebug("原始MQTT消息: %s", payload)

            # 解析JSON
            data = json.loads(payload)

            # 验证JSON结构
            if "linear" not in data or "angular" not in data:
                raise ValueError("JSON缺少linear或angular字段")

            # 提取数据并填充Twist消息
            twist = Twist()

            linear = data["linear"]
            angular = data["angular"]

            twist.linear.x = linear.get("x", 0.0)
            twist.linear.y = linear.get("y", 0.0)
            twist.linear.z = linear.get("z", 0.0)

            twist.angular.x = angular.get("x", 0.0)
            twist.angular.y = angular.get("y", 0.0)
            twist.angular.z = angular.get("z", 0.0)

            self.pub.publish(twist)
            # rospy.loginfo("已转发Twist命令: linear=%.2f, angular=%.2f", 
            #             twist.linear.x, twist.angular.z)
        except UnicodeDecodeError as e:
            rospy.logerr("解码失败: %s", e)
        except json.JSONDecodeError as e:
            rospy.logerr("JSON解析失败: %s", e)
        except KeyError as e:
            rospy.logerr("缺少必要字段: %s", e)
        except Exception as e:
            rospy.logerr("未知错误: %s", e)

    def shutdown(self):
        self.mqtt_client.loop_stop()
        rospy.loginfo("MQTT→ROS 桥接已关闭")

if __name__ == '__main__':
    bridge = MQTTToCmdVel()
    rospy.on_shutdown(bridge.shutdown)
    rospy.spin()
