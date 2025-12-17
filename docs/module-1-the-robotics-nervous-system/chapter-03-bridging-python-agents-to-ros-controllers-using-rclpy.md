---
title: Chapter 03 Bridging Python agents to ROS controllers using rclpy
---

Bridging Python agents to ROS controllers means connecting high-level Python-based decision makers (AI agents, RL policies, planners) with low-level ROS 2 controllers (motor, joint, velocity control) using rclpy, the ROS 2 Python client library.

## **Bridging Python Agents to ROS Controllers using `rclpy`**

![Image](https://control.ros.org/humble/_images/ros2_control_overview.png?utm_source=chatgpt.com)

![Image](https://www.yahboom.net/public/upload/upload-html/1701163060/image8.gif?utm_source=chatgpt.com)

![Image](https://us1.discourse-cdn.com/flex022/uploads/ros/optimized/3X/6/a/6ad481ed4f6916f6bcf72bdd195e4b5a1986935c_2_1024x574.jpeg?utm_source=chatgpt.com)

**Bridging Python agents to ROS controllers** means connecting **high-level Python-based decision makers** (AI agents, RL policies, planners) with **low-level ROS 2 controllers** (motor, joint, velocity control) using **`rclpy`**, the ROS 2 Python client library.

---

## 🧠 Concept Overview

* **Python Agent**
  Makes decisions (velocity, joint targets, actions) using logic, ML, or AI.

* **ROS 2 Controller**
  Executes real-time control through `ros2_control` and hardware interfaces.

* **`rclpy` Bridge**
  Acts as the communication layer between the agent and controllers.

---

## 🏗️ Architecture

```
Python Agent (AI / RL / Logic)
        ↓  (rclpy)
ROS 2 Node (Command Publisher)
        ↓
Controller Manager (ros2_control)
        ↓
Robot Hardware (Motors / Joints)
```

---

## 🔁 Communication Methods

### 1. **Topics (Most Common)**

Python agent publishes commands:

* `/cmd_vel` → mobile robots
* `/joint_trajectory` → manipulators

Controllers subscribe and execute.

---

### 2. **Services**

Used for:

* Controller switching
* Resetting controllers
* Mode changes

---

### 3. **Actions**

Used for:

* Long-running tasks (navigation, grasping)
* Feedback and result handling

---

## 🧪 Example: Python Agent → Velocity Controller

### ✅ Python Agent Node (`rclpy`)

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AgentBridge(Node):
    def __init__(self):
        super().__init__('agent_bridge')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        msg = Twist()
        msg.linear.x = 0.3     # Agent decision
        msg.angular.z = 0.1
        self.publisher.publish(msg)

rclpy.init()
node = AgentBridge()
rclpy.spin(node)
```

---

## ⚙️ ROS 2 Controller Side

Typical controller:

* `diff_drive_controller`
* `joint_trajectory_controller`
* `forward_command_controller`

Subscribed topic:

```
/cmd_vel
```

---

## 🤖 Example Use Cases

* **Reinforcement Learning agents**
* **Autonomous navigation**
* **Multi-agent robot systems**
* **Simulation-to-real transfer**
* **Human-in-the-loop control**

---

## ✅ Why Use `rclpy` for Bridging?

* Easy Python integration 🐍
* Direct compatibility with ROS 2
* Distributed & scalable
* DDS-based real-time communication
* Works with simulation and hardware

---

## 📌 Summary

**`rclpy` enables Python agents to control ROS 2 controllers by publishing commands, calling services, or executing actions.**
This bridge cleanly separates **intelligence (Python agent)** from **execution (ROS controllers)**—a best-practice design for modern robotic systems 🤖
