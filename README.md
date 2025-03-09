# MUX-MQTT
多路输入切换器和mqtt桥。mqtt使用基本二进制序列化，性能有限

==支持ROS noetic==

  # 发布 control_mode 切换指令
rostopic pub /control_mode std_msgs/String "data: teleop" 或者 rosservice call /cmd_vel_mux/select "topic: '/teleop_cmd_vel'"
rostopic pub /teleop_cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" -r 10

# 切换至远程模式
rostopic pub /control_mode std_msgs/String "data: remote"  或者 rosservice call /cmd_vel_mux/select "topic: '/remote_cmd_vel'"
rostopic pub /remote_cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" -r 10

# 触发紧急停止（优先级100，覆盖 mux 的50）
rostopic pub /emergency_cmd_vel geometry_msgs/Twist "linear: {x: 0}" -r 10

rosservice call /cmd_vel_mux/select "topic: '/teleop_cmd_vel'"

在远端发布命令时，
1. 切换输入
远端时MQTTX直接发布字符串到话题[pingpong/control_mode]，以切换模式 ==默认是teleop==
"remote"
"teleop"
近端时，可以在命令行中发布`rostopic pub /control_mode std_msgs/String "data: teleop" 或者 rosservice call /cmd_vel_mux/select "topic: '/teleop_cmd_vel'"`
以切换命令

2. 当为"remote"模式时
MQTT可以发布json指令到话题[pingpong/cmd_vel]
格式为
```json
{
  "linear": {
    "x": 0.5,
    "y": 0.0,
    "z": 0.0
  },
  "angular": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.2
  }
}
```

3. 当为"teleop"模式时
在近端时，启动控制器
```python
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=teleop_cmd_vel
```

默认完全功能的桥以及键盘控制器
```python
roslaunch mqtt_bridge start.demo.launch
```
