---
title: Chapter 04 URDF (Unified Robot Description Format)
---

URDF (Unified Robot Description Format) is an XML-based file format used in ROS and ROS 2 to describe the physical structure of a robot. It defines how a robot looks, how its parts are connected, and how they move relative to each other.

### **Understanding URDF (Unified Robot Description Format)**

![Image](https://spart.readthedocs.io/en/latest/_images/SC_Example.png?utm_source=chatgpt.com)

![Image](https://abedgnu.github.io/Notes-ROS/_images/joint.png?utm_source=chatgpt.com)

![Image](https://i.ytimg.com/vi/kh2yhsKZRQ8/maxresdefault.jpg?utm_source=chatgpt.com)

**URDF (Unified Robot Description Format)** is an **XML-based file format** used in ROS and ROS 2 to **describe the physical structure of a robot**. It defines how a robot looks, how its parts are connected, and how they move relative to each other.

---

### 🧠 What URDF Represents

URDF provides a complete **kinematic and visual description** of a robot, including:

* Robot **links** (rigid bodies)
* **Joints** connecting the links
* **Visual appearance** (meshes, colors)
* **Collision geometry**
* **Physical properties** (mass and inertia)

---

### 🧩 Core Components

#### 1. **Links**

Links are the rigid parts of the robot.
Each link can have:

* `visual` → appearance in RViz
* `collision` → used for collision detection
* `inertial` → mass and inertia (for dynamics)

#### 2. **Joints**

Joints define how links move relative to each other.

Common joint types:

* `fixed`
* `revolute`
* `continuous`
* `prismatic`

Each joint specifies:

* Parent and child links
* Rotation or translation axis
* Motion limits

---

### 🏗️ Example Structure

```
robot
 ├── base_link
 │    └── joint1
 │         └── link1
 │              └── joint2
 │                   └── link2
```

---

### 🛠️ Where URDF Is Used

* **RViz** → visualization
* **Gazebo / Ignition** → simulation
* **ros2_control** → hardware & controllers
* **MoveIt** → motion planning

URDF is the **single source of truth** for robot geometry.

---

### 📐 Why Understanding URDF Is Important

* Enables correct **robot visualization**
* Essential for **motion planning**
* Required for **accurate simulation**
* Defines joint limits and constraints
* Links software control to physical hardware

---

### ✅ Summary

Understanding URDF means understanding the **robot’s physical and kinematic structure**.
It allows ROS tools to visualize, simulate, and control robots accurately, making it a **foundation of robot control and development** 🤖
