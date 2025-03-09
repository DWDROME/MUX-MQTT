#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from topic_tools.srv import MuxSelect  # MUX 话题切换服务

class MuxController:
    def __init__(self):
        rospy.init_node("mux_controller")

        # 订阅 /control_mode 话题
        rospy.Subscriber("/control_mode", String, self.control_mode_callback)

        # 等待 /mux/select 服务可用
        rospy.loginfo("等待 MUX 服务 /mux/select 可用...")
        rospy.wait_for_service("/mux/select")

        # 连接到 MUX 话题切换服务
        self.mux_select = rospy.ServiceProxy("/mux/select", MuxSelect)
        rospy.loginfo("MUX 服务已连接，准备切换话题...")

    def control_mode_callback(self, msg):
        mode = msg.data.strip().lower()  # 去除空格并转换为小写
        topic_name = None

        if mode == "teleop":
            topic_name = "/teleop_cmd_vel"
        elif mode == "remote":
            topic_name = "/remote_cmd_vel"
        else:
            rospy.logwarn("未知的控制模式: %s", mode)
            return

        try:
            # 调用 MUX 话题切换服务
            response = self.mux_select(topic=topic_name)
            rospy.loginfo("MUX 话题切换成功: %s → %s", response.prev_topic, topic_name)
        except rospy.ServiceException as e:
            rospy.logerr("MUX 话题切换失败: %s", e)

if __name__ == "__main__":
    MuxController()
    rospy.spin()
