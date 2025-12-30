---
title: Chapter 04 URDF (Unified Robot Description Format)
---

## **URDF: The Blueprint of Robotic Systems** 🤖

![Image](https://spart.readthedocs.io/en/latest/_images/SC_Example.png?utm_source=chatgpt.com)

**URDF (Unified Robot Description Format)** is an **XML-based file format** used in ROS and ROS 2 to **describe the physical structure of a robot**. It defines how a robot looks, how its parts are connected, and how they move relative to each other. URDF serves as the **digital blueprint** that enables accurate visualization, simulation, and control of robotic systems.

---

## 🧠 **Core Concepts of URDF**

### **What is URDF?**

URDF is a **robot description language** that uses XML syntax to define:

* **Kinematic structure** → How robot parts connect and move
* **Visual appearance** → How the robot looks in visualization
* **Collision geometry** → How the robot interacts with the environment
* **Physical properties** → Mass, inertia, and dynamics
* **Sensors and actuators** → Where components are mounted

### **Key Benefits**

* **Single source of truth** for robot geometry
* **Cross-platform compatibility** across ROS tools
* **Simulation-ready** for Gazebo and Ignition
* **Motion planning support** for MoveIt
* **Visualization integration** with RViz

---

## 🏗️ **URDF Structure and Components**

### **1. Robot Root Element**

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- Robot definition goes here -->
</robot>
```

### **2. Links - The Building Blocks**

Links represent **rigid bodies** in the robot structure:

```xml
<link name="base_link">
    <visual>
        <geometry>
            <cylinder length="0.6" radius="0.2"/>
        </geometry>
        <material name="blue">
            <color rgba="0 0 0.8 1"/>
        </material>
    </visual>

    <collision>
        <geometry>
            <cylinder length="0.6" radius="0.2"/>
        </geometry>
    </collision>

    <inertial>
        <mass value="10"/>
        <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
</link>
```

#### **Visual Properties**
* **Geometry** → Shape (box, cylinder, sphere, mesh)
* **Material** → Color and texture
* **Origin** → Position and orientation relative to link frame

#### **Collision Properties**
* **Geometry** → Simplified shapes for collision detection
* **Origin** → Position and orientation relative to link frame

#### **Inertial Properties**
* **Mass** → Physical mass of the link
* **Inertia** → Moment of inertia tensor
* **Origin** → Center of mass location

### **3. Joints - The Connections**

Joints define how links move relative to each other:

```xml
<joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.1 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
</joint>
```

#### **Joint Types**

* **`fixed`** → No movement (welded connection)
* **`revolute`** → Single-axis rotation with limits
* **`continuous`** → Single-axis rotation without limits
* **`prismatic`** → Single-axis translation with limits
* **`floating`** → 6DOF movement (rarely used)
* **`planar`** → Planar movement (rarely used)

#### **Joint Properties**
* **Parent/Child** → Links that the joint connects
* **Origin** → Position and orientation of joint frame
* **Axis** → Rotation/translation axis
* **Limits** → Min/max values and effort/velocity limits

---

## 🔧 **URDF Elements in Detail**

### **1. Visual Elements**

```xml
<visual>
    <!-- Geometry options -->
    <geometry>
        <!-- Box -->
        <box size="1.0 2.0 3.0"/>

        <!-- Cylinder -->
        <cylinder radius="0.5" length="1.0"/>

        <!-- Sphere -->
        <sphere radius="0.3"/>

        <!-- Mesh -->
        <mesh filename="package://my_robot/meshes/link.dae" scale="1.0 1.0 1.0"/>
    </geometry>

    <!-- Material -->
    <material name="red">
        <color rgba="1 0 0 1"/>
        <!-- Or reference texture -->
        <texture filename="package://my_robot/materials/textures/red.png"/>
    </material>

    <!-- Origin (position and orientation) -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
</visual>
```

### **2. Collision Elements**

```xml
<collision>
    <!-- Often simplified geometry for performance -->
    <geometry>
        <box size="1.0 2.0 3.0"/>
        <!-- Or use the same as visual -->
        <mesh filename="package://my_robot/meshes/link_collision.stl"/>
    </geometry>

    <!-- Origin -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
</collision>
```

### **3. Inertial Elements**

```xml
<inertial>
    <!-- Mass -->
    <mass value="1.0"/>

    <!-- Inertia tensor -->
    <inertia
        ixx="0.1" ixy="0.0" ixz="0.0"
        iyy="0.2" iyz="0.0"
        izz="0.3"/>

    <!-- Center of mass -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
</inertial>
```

### **4. Joint Elements**

```xml
<joint name="example_joint" type="revolute">
    <!-- Links -->
    <parent link="parent_link"/>
    <child link="child_link"/>

    <!-- Joint pose -->
    <origin xyz="0 0 1" rpy="0 0 0"/>

    <!-- Axis of rotation/translation -->
    <axis xyz="0 0 1"/>

    <!-- Joint limits (for revolute and prismatic) -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>

    <!-- Joint dynamics -->
    <dynamics damping="0.1" friction="0.0"/>
</joint>
```

---

## 🧪 **Complete URDF Example**

Here's a complete example of a simple differential drive robot:

```xml
<?xml version="1.0"?>
<robot name="diff_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Base link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.3 0.15"/>
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1"/>
            </material>
        </visual>

        <collision>
            <geometry>
                <box size="0.5 0.3 0.15"/>
            </geometry>
        </collision>

        <inertial>
            <mass value="5.0"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                     iyy="0.2" iyz="0.0"
                     izz="0.3"/>
        </inertial>
    </link>

    <!-- Left wheel -->
    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0 0 0" rpy="1.5708 0 0"/>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>

        <collision>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0 0 0" rpy="1.5708 0 0"/>
        </collision>

        <inertial>
            <mass value="0.5"/>
            <inertia ixx="0.001" ixy="0.0" ixz="0.0"
                     iyy="0.001" iyz="0.0"
                     izz="0.002"/>
        </inertial>
    </link>

    <!-- Right wheel -->
    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0 0 0" rpy="1.5708 0 0"/>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>

        <collision>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0 0 0" rpy="1.5708 0 0"/>
        </collision>

        <inertial>
            <mass value="0.5"/>
            <inertia ixx="0.001" ixy="0.0" ixz="0.0"
                     iyy="0.001" iyz="0.0"
                     izz="0.002"/>
        </inertial>
    </link>

    <!-- Castor wheel -->
    <link name="caster_wheel">
        <visual>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>

        <collision>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
        </collision>

        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
                     iyy="0.0001" iyz="0.0"
                     izz="0.0001"/>
        </inertial>
    </link>

    <!-- Joints -->
    <joint name="left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel"/>
        <origin xyz="0 0.2 -0.025" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="0 -0.2 -0.025" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="caster_joint" type="fixed">
        <parent link="base_link"/>
        <child link="caster_wheel"/>
        <origin xyz="-0.2 0 -0.075" rpy="0 0 0"/>
    </joint>

    <!-- Transmission for ros2_control -->
    <transmission name="left_wheel_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="left_wheel_joint">
            <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        </joint>
        <actuator name="left_wheel_motor">
            <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>

    <transmission name="right_wheel_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="right_wheel_joint">
            <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        </joint>
        <actuator name="right_wheel_motor">
            <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>

</robot>
```

---

## 🔄 **URDF with Xacro**

Xacro is a macro language that makes URDF files more readable and maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

    <!-- Properties -->
    <xacro:property name="M_PI" value="3.1415926535897931" />
    <xacro:property name="wheel_radius" value="0.1" />
    <xacro:property name="wheel_width" value="0.05" />
    <xacro:property name="base_length" value="0.5" />
    <xacro:property name="base_width" value="0.3" />
    <xacro:property name="base_height" value="0.15" />

    <!-- Macro for wheel -->
    <xacro:macro name="wheel" params="prefix parent xyz rpy">
        <link name="${prefix}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
                <material name="black">
                    <color rgba="0 0 0 1"/>
                </material>
            </visual>

            <collision>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
            </collision>

            <inertial>
                <mass value="0.5"/>
                <inertia ixx="0.001" ixy="0.0" ixz="0.0"
                         iyy="0.001" iyz="0.0"
                         izz="0.002"/>
            </inertial>
        </link>

        <joint name="${prefix}_wheel_joint" type="continuous">
            <parent link="${parent}"/>
            <child link="${prefix}_wheel"/>
            <origin xyz="${xyz}" rpy="${rpy}"/>
            <axis xyz="0 1 0"/>
        </joint>
    </xacro:macro>

    <!-- Base link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1"/>
            </material>
        </visual>

        <collision>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
        </collision>

        <inertial>
            <mass value="5.0"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                     iyy="0.2" iyz="0.0"
                     izz="0.3"/>
        </inertial>
    </link>

    <!-- Wheels using macro -->
    <xacro:wheel prefix="left" parent="base_link" xyz="0 0.15 -0.025" rpy="0 0 0"/>
    <xacro:wheel prefix="right" parent="base_link" xyz="0 -0.15 -0.025" rpy="0 0 0"/>

</robot>
```

---

## 🛠️ **URDF Tools and Validation**

### **1. URDF Validation**

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Parse and display robot information
urdf_to_graphiz /path/to/robot.urdf
```

### **2. Visualization Tools**

```bash
# View robot in RViz
ros2 run rviz2 rviz2

# View robot in standalone viewer
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat robot.urdf)'
```

### **3. Common Validation Steps**

1. **XML Syntax Check** → Verify proper XML formatting
2. **Link/Joint Connectivity** → Ensure all joints connect existing links
3. **Kinematic Chain** → Verify there's a valid path from base to all links
4. **Physical Properties** → Check mass and inertia values are reasonable

---

## 🤖 **URDF in Simulation and Control**

### **1. Gazebo Integration**

```xml
<!-- Gazebo-specific extensions -->
<gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
</gazebo>

<!-- Plugin for ros2_control -->
<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find my_robot)/config/my_robot_controllers.yaml</parameters>
    </plugin>
</gazebo>
```

### **2. ros2_control Integration**

```xml
<!-- Transmission elements for ros2_control -->
<transmission name="joint1_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="joint1_motor">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

### **3. MoveIt Integration**

```xml
<!-- Safety settings for MoveIt -->
<safety_controller k_position="100.0" k_velocity="10.0"
                  soft_lower_limit="-1.57" soft_upper_limit="1.57"/>
```

---

## ⚙️ **Best Practices for URDF Creation**

### **1. Naming Conventions**

* Use **descriptive names** that reflect the component's function
* Follow **consistent naming patterns** (e.g., `left_wheel`, `right_wheel`)
* Use **lowercase with underscores** for joint and link names
* Include **semantic meaning** in names (e.g., `gripper_finger_left`)

### **2. Physical Accuracy**

* **Realistic mass values** → Use actual robot measurements
* **Accurate inertia tensors** → Calculate from CAD models when possible
* **Proper collision geometry** → Simplified but accurate shapes
* **Realistic joint limits** → Based on physical constraints

### **3. Performance Optimization**

* **Simplified collision geometry** → Use boxes/cylinders instead of complex meshes
* **Appropriate mesh resolution** → Balance detail with performance
* **Efficient joint structure** → Minimize unnecessary links
* **Xacro macros** → Reduce redundancy and improve maintainability

### **4. Organization**

* **Logical structure** → Group related components
* **Clear comments** → Document complex sections
* **Modular design** → Use Xacro for reusable components
* **Version control** → Track changes to robot descriptions

---

## 🚀 **Advanced URDF Features**

### **1. Gazebo-Specific Elements**

```xml
<!-- Sensors -->
<gazebo reference="camera_link">
    <sensor name="camera" type="camera">
        <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.1</near>
                <far>100</far>
            </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so"/>
    </sensor>
</gazebo>

<!-- Actuators -->
<gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.3</wheel_separation>
        <wheel_diameter>0.2</wheel_diameter>
    </plugin>
</gazebo>
```

### **2. Semantic Descriptions**

```xml
<!-- Robot semantic description (SRDF) -->
<!-- This is typically in a separate .srdf file -->
<robot name="my_robot">
    <!-- Groups of joints for planning -->
    <group name="arm">
        <chain base_link="base_link" tip_link="end_effector"/>
    </group>

    <!-- End effectors -->
    <end_effector name="gripper" parent_link="wrist_link" group="gripper"/>

    <!-- Virtual joints for mobile base -->
    <virtual_joint name="odom_joint" type="planar" parent_frame="odom" child_link="base_footprint"/>
</robot>
```

### **3. Transmission Types**

```xml
<!-- Position interface -->
<transmission name="position_joint_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor1">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
</transmission>

<!-- Velocity interface -->
<transmission name="velocity_joint_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint2">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor2">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </actuator>
</transmission>

<!-- Effort interface -->
<transmission name="effort_joint_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint3">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor3">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </actuator>
</transmission>
```

---

## 🧪 **URDF Debugging and Troubleshooting**

### **Common Issues**

1. **Missing Links** → Joint references non-existent link
2. **Invalid Inertias** → Negative or zero values
3. **Kinematic Loops** → Multiple paths between links
4. **Joint Limit Errors** → Limits exceed physical constraints

### **Debugging Commands**

```bash
# Validate URDF
check_urdf my_robot.urdf

# Generate kinematic graph
urdf_to_graphiz my_robot.urdf
dot -Tpng my_robot.gv -o my_robot.png

# Check joint limits
ros2 param list | grep joint
```

### **Visualization Debugging**

```bash
# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat my_robot.urdf)'

# View in RViz
ros2 run rviz2 rviz2
```

---

## 📊 **URDF in Real-World Applications**

### **1. Industrial Robots**

* **Precision manufacturing** → Accurate kinematic models
* **Collision avoidance** → Proper collision geometry
* **Path planning** → Integration with MoveIt
* **Safety systems** → Joint limits and constraints

### **2. Service Robots**

* **Navigation** → Base kinematic models
* **Manipulation** → Arm and gripper descriptions
* **Human interaction** → Collision-safe designs
* **Multi-robot systems** → Shared robot descriptions

### **3. Research Platforms**

* **Algorithm development** → Accurate simulation models
* **Hardware validation** → Simulation-to-reality transfer
* **Multi-robot experiments** → Standardized descriptions
* **Educational tools** → Clear robot structure visualization

---

## ✅ **Summary**

**URDF (Unified Robot Description Format)** is the **fundamental building block** for representing robotic systems in ROS and ROS 2. It provides a comprehensive description of:

* **Kinematic structure** → How robot parts connect and move
* **Visual appearance** → How the robot looks in visualization
* **Collision geometry** → How the robot interacts with the environment
* **Physical properties** → Mass, inertia, and dynamics
* **Integration points** → For simulation, planning, and control

Understanding URDF is **essential** for developing, simulating, and controlling robotic systems. It serves as the bridge between the physical robot and its digital representation in software tools 🤖

---

## 📚 **Further Reading**

* URDF Tutorials: http://wiki.ros.org/urdf/Tutorials
* Xacro Documentation: http://wiki.ros.org/xacro
* ROS 2 Control URDF Integration: https://control.ros.org/
* MoveIt Robot Description: https://moveit.ros.org/documentation/
* Gazebo Robot Description: http://gazebosim.org/tutorials?tut=ros_urdf
