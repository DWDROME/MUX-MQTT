### **MUX-MQTT**
多路输入切换器和 MQTT 桥接，MQTT 使用基本二进制序列化，性能有限。

主要功能包：
- mqtt_bridge

---

==支持 ROS Noetic==
使用功能包：
- twist_mux   --速度优先度控制（锁）未优化完成
- topic_tools --control_mode控制输入方式
- teleop_twist_keyboard --键盘控制

---
目前已完成基本MUX-MQTT桥建设
可以在mqtt_bridge.yaml中修改对应的话题名称

未完成：
- 手柄控制
- 导航控制
- 强制刹车

---

## **YAML 配置**
在 `config/mqtt_config.yaml` 中定义 MQTT 相关参数：
```yaml
mqtt:
  broker: "192.168.18.80"   # MQTT 服务器地址
  port: 1883               # MQTT 端口

  mqtt_pose: "pingpong/turtle1/pose"   # MQTT 订阅的位姿话题
  topic_pose: "/turtle1/pose"          # ROS 发布的位姿话题

  mqtt_cmd_vel_sub: "pingpong/cmd_vel" # MQTT 订阅的速度控制话题
  topic_cmd_vel: "/remote_cmd_vel"     # ROS 发布的速度控制话题

  mqtt_control_mode: "pingpong/control_mode"  # MQTT 订阅的控制模式话题
  topic_control_mode: "/control_mode"         # ROS 发布的控制模式话题
```
**在 `launch` 文件中加载 YAML 配置**
```xml
<rosparam file="$(find mqtt_bridge)/config/mqtt_config.yaml" command="load"/>
```

---

## **控制模式切换**
### **切换至 `teleop` 模式**
```bash
rostopic pub /control_mode std_msgs/String "data: teleop"
rosservice call /cmd_vel_mux/select "topic: '/teleop_cmd_vel'"
rostopic pub /teleop_cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" -r 10
```

### **切换至 `remote` 模式**
```bash
rostopic pub /control_mode std_msgs/String "data: remote"
rosservice call /cmd_vel_mux/select "topic: '/remote_cmd_vel'"
rostopic pub /remote_cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" -r 10
```

### **紧急停止**
```bash
rostopic pub /emergency_cmd_vel geometry_msgs/Twist "linear: {x: 0}" -r 10
```

---

## **远程端控制**
### **1. 切换模式**
MQTTX 发布到 `pingpong/control_mode`
```
"remote"
"teleop"
```
默认 `teleop`。  
本地可用：
```bash
rostopic pub /control_mode std_msgs/String "data: teleop"
rosservice call /cmd_vel_mux/select "topic: '/teleop_cmd_vel'"
```

### **2. `remote` 模式**
MQTT 发送 JSON 指令到 `pingpong/cmd_vel`：
```json
{
  "linear": { "x": 0.5, "y": 0.0, "z": 0.0 },
  "angular": { "x": 0.0, "y": 0.0, "z": 0.2 }
}
```

### **3. `teleop` 模式**
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=teleop_cmd_vel
```

---

## **启动完整系统**
```bash
roslaunch mqtt_bridge start_demo.launch
```
