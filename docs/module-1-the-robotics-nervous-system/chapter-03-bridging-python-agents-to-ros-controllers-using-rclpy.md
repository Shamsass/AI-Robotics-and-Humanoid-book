---
title: Chapter 03 Bridging Python agents to ROS controllers using rclpy
---

## **Bridging Python Agents to ROS Controllers: Connecting AI to Physical Execution** 🧠

![Image](https://control.ros.org/humble/_images/ros2_control_overview.png?utm_source=chatgpt.com)

**Bridging Python agents to ROS controllers** means connecting **high-level Python-based decision makers** (AI agents, RL policies, planners) with **low-level ROS 2 controllers** (motor, joint, velocity control) using **`rclpy`**, the ROS 2 Python client library. This bridge enables the integration of sophisticated artificial intelligence with real-time robotic control systems.

---

## 🧠 **Core Concepts of Agent-Controller Bridging**

### **What is Agent-Controller Bridging?**

Agent-controller bridging is the process of connecting:

* **High-level AI agents** (Python-based decision makers)
* **Low-level control systems** (ROS 2 controllers)
* **Communication layer** (`rclpy`)

This connection allows intelligent algorithms to control physical robots effectively.

### **Key Components**

* **Python Agent**: Makes decisions using AI, ML, or logic
* **ROS 2 Controller**: Executes real-time control commands
* **rclpy Bridge**: Facilitates communication between components
* **Hardware Interface**: Connects to physical robot hardware

---

## 🏗️ **Architecture Patterns**

### **1. Direct Bridge Pattern**

```
Python Agent (AI / ML / Logic)
        ↓
rclpy Node (Bridge)
        ↓
ROS 2 Controller Manager
        ↓
Hardware Interface
        ↓
Robot Hardware
```

### **2. Distributed Bridge Pattern**

```
Python Agent (Local/Machine A)
        ↓
rclpy Bridge Node
        ↓
DDS Network (ROS 2 Middleware)
        ↓
ROS 2 Controllers (Machine B)
        ↓
Robot Hardware (Machine C)
```

### **3. Multi-Agent Bridge Pattern**

```
Multiple Python Agents
        ↓
rclpy Bridge Node (Aggregator)
        ↓
Multiple ROS 2 Controllers
        ↓
Robot Hardware
```

---

## 🔧 **Implementation Techniques**

### **1. Publisher-Based Bridging**

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class AgentControllerBridge(Node):
    def __init__(self):
        super().__init__('agent_controller_bridge')

        # Publishers for different controller types
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers for sensor feedback
        self.joint_state_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)

        # Agent state
        self.current_joint_states = None
        self.agent_action = None

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def set_agent_action(self, action):
        """Method for AI agent to set control action"""
        self.agent_action = action

    def control_loop(self):
        if self.agent_action is not None:
            # Convert agent action to ROS message
            cmd = self.convert_action_to_command(self.agent_action)
            if cmd is not None:
                self.publish_command(cmd)

    def convert_action_to_command(self, action):
        """Convert agent action to appropriate ROS message"""
        if action['type'] == 'velocity':
            msg = Twist()
            msg.linear.x = action['linear_x']
            msg.angular.z = action['angular_z']
            return msg
        elif action['type'] == 'joint_trajectory':
            msg = JointTrajectory()
            msg.joint_names = action['joint_names']
            point = JointTrajectoryPoint()
            point.positions = action['positions']
            point.velocities = action['velocities']
            msg.points.append(point)
            return msg
        return None

    def publish_command(self, cmd):
        if isinstance(cmd, Twist):
            self.cmd_vel_publisher.publish(cmd)
        elif isinstance(cmd, JointTrajectory):
            self.joint_trajectory_publisher.publish(cmd)

def main():
    rclpy.init()
    bridge = AgentControllerBridge()

    # Example: Simulate AI agent setting actions
    import time
    import threading

    def ai_agent_simulation():
        time.sleep(1)  # Wait for node to initialize
        for i in range(10):
            action = {
                'type': 'velocity',
                'linear_x': 0.5 + 0.1 * i,
                'angular_z': 0.1 * i
            }
            bridge.set_agent_action(action)
            time.sleep(0.5)

    # Start AI agent simulation in separate thread
    agent_thread = threading.Thread(target=ai_agent_simulation)
    agent_thread.start()

    rclpy.spin(bridge)
    agent_thread.join()

if __name__ == '__main__':
    main()
```

### **2. Service-Based Bridging**

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from example_interfaces.srv import SetBool
from std_msgs.msg import Float64MultiArray

class ServiceBasedBridge(Node):
    def __init__(self):
        super().__init__('service_based_bridge')

        # Service server for agent commands
        self.service = self.create_service(
            SetBool, 'execute_agent_command', self.execute_command_callback)

        # Publisher for command execution
        self.command_publisher = self.create_publisher(
            Float64MultiArray, '/agent_commands', 10)

    def execute_command_callback(self, request, response):
        """Handle agent command execution"""
        try:
            # Execute the command based on request
            command_data = self.process_agent_command(request.data)
            self.command_publisher.publish(command_data)
            response.success = True
            response.message = "Command executed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Command execution failed: {str(e)}"

        return response

    def process_agent_command(self, command_flag):
        """Process the agent's command"""
        msg = Float64MultiArray()
        if command_flag:
            # Example: Send specific joint positions
            msg.data = [0.5, 1.0, -0.5]  # Joint positions
        else:
            # Example: Send stop command
            msg.data = [0.0, 0.0, 0.0]   # Stop joints
        return msg
```

### **3. Action-Based Bridging**

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ActionBasedBridge(Node):
    def __init__(self):
        super().__init__('action_based_bridge')

        # Action client to send commands to controllers
        self._action_client = ActionClient(
            self, FollowJointTrajectory, 'follow_joint_trajectory')

        # Timer for periodic action execution
        self.timer = self.create_timer(1.0, self.send_trajectory_action)

    def send_trajectory_action(self):
        """Send trajectory action to controller"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3']

        # Create trajectory points
        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.5, -0.5]
        point.velocities = [0.1, 0.1, 0.1]
        point.time_from_start.sec = 2
        goal_msg.trajectory.points.append(point)

        # Wait for action server
        self._action_client.wait_for_server()

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')
```

---

## 🔄 **Communication Patterns**

### **1. Real-time Topics**

For high-frequency control commands:

```python
# Publisher with appropriate QoS for real-time control
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos_profile = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

publisher = node.create_publisher(Twist, '/cmd_vel', qos_profile)
```

### **2. Buffered Communication**

For less time-critical commands:

```python
# Publisher with larger buffer for less critical commands
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
```

### **3. Parameter-Based Configuration**

```python
# Declare parameters for agent configuration
self.declare_parameter('agent_control_rate', 50.0)  # Hz
self.declare_parameter('max_velocity', 1.0)
self.declare_parameter('safety_timeout', 1.0)  # seconds

# Use parameters in control loop
control_rate = self.get_parameter('agent_control_rate').value
max_vel = self.get_parameter('max_velocity').value
```

---

## 🧪 **Advanced Integration Examples**

### **1. Reinforcement Learning Agent Integration**

```python
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class RLBridgeNode(Node):
    def __init__(self):
        super().__init__('rl_bridge_node')

        # Publishers and subscribers
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.reward_publisher = self.create_publisher(Float32, '/reward', 10)

        # RL state
        self.laser_data = None
        self.episode_step = 0
        self.reward = 0.0

        # Control timer
        self.timer = self.create_timer(0.1, self.rl_control_loop)

    def scan_callback(self, msg):
        self.laser_data = np.array(msg.ranges)

    def rl_control_loop(self):
        if self.laser_data is not None:
            # Get action from RL agent (simplified)
            action = self.get_rl_action(self.laser_data)

            # Convert action to Twist command
            cmd = Twist()
            cmd.linear.x = action[0]  # Forward velocity
            cmd.angular.z = action[1]  # Angular velocity

            # Publish command
            self.cmd_publisher.publish(cmd)

            # Calculate and publish reward
            reward = self.calculate_reward(cmd, self.laser_data)
            reward_msg = Float32()
            reward_msg.data = reward
            self.reward_publisher.publish(reward_msg)

    def get_rl_action(self, observation):
        # Simplified RL action selection
        # In practice, this would call your trained RL model
        min_distance = np.min(observation)

        if min_distance < 0.5:  # Obstacle too close
            return [0.0, 0.5]  # Turn
        else:
            return [0.3, 0.0]  # Move forward

    def calculate_reward(self, cmd, laser_data):
        # Simplified reward calculation
        min_distance = np.min(laser_data)

        # Positive reward for moving forward when safe
        forward_reward = cmd.linear.x if min_distance > 0.5 else 0

        # Negative reward for being too close to obstacles
        safety_penalty = -10.0 if min_distance < 0.3 else 0

        # Small penalty for turning
        turn_penalty = -0.1 * abs(cmd.angular.z)

        return forward_reward + safety_penalty + turn_penalty
```

### **2. Multi-Agent Coordination**

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import json

class MultiAgentBridge(Node):
    def __init__(self):
        super().__init__('multi_agent_bridge')

        # Publishers for different agents
        self.agent_cmd_publishers = {}
        for i in range(3):  # 3 agents
            self.agent_cmd_publishers[f'agent_{i}'] = self.create_publisher(
                Twist, f'/agent_{i}/cmd_vel', 10)

        # Publisher for coordination messages
        self.coordination_publisher = self.create_publisher(
            String, '/coordination', 10)

        # Subscribers for agent states
        self.agent_state_subscribers = {}
        for i in range(3):
            self.agent_state_subscribers[f'agent_{i}'] = self.create_subscription(
                PoseStamped, f'/agent_{i}/pose',
                lambda msg, agent_id=i: self.agent_state_callback(msg, agent_id), 10)

        # Timer for coordination
        self.timer = self.create_timer(0.5, self.coordination_loop)

        # Agent states
        self.agent_states = {}

    def agent_state_callback(self, msg, agent_id):
        self.agent_states[agent_id] = msg.pose

    def coordination_loop(self):
        # Implement coordination logic
        coordination_msg = String()
        coordination_data = {
            'timestamp': self.get_clock().now().nanoseconds,
            'agent_states': {}
        }

        for agent_id, pose in self.agent_states.items():
            coordination_data['agent_states'][agent_id] = {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            }

        coordination_msg.data = json.dumps(coordination_data)
        self.coordination_publisher.publish(coordination_msg)
```

---

## ⚙️ **Controller Types and Integration**

### **1. Velocity Controllers**

```python
from geometry_msgs.msg import Twist

# For differential drive robots
def send_velocity_command(self, linear_vel, angular_vel):
    cmd = Twist()
    cmd.linear.x = linear_vel
    cmd.angular.z = angular_vel
    self.velocity_publisher.publish(cmd)
```

### **2. Joint Trajectory Controllers**

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_joint_trajectory(self, joint_names, positions, velocities=None):
    msg = JointTrajectory()
    msg.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = positions
    if velocities:
        point.velocities = velocities
    point.time_from_start.sec = 1
    msg.points.append(point)

    self.joint_trajectory_publisher.publish(msg)
```

### **3. Position Controllers**

```python
from std_msgs.msg import Float64MultiArray

def send_position_command(self, positions):
    msg = Float64MultiArray()
    msg.data = positions
    self.position_publisher.publish(msg)
```

---

## 🚀 **Performance Optimization**

### **1. Threading Considerations**

```python
import threading
from rclpy.executors import MultiThreadedExecutor

class ThreadedAgentBridge(Node):
    def __init__(self):
        super().__init__('threaded_agent_bridge')

        # Use threading lock for shared data
        self.data_lock = threading.Lock()
        self.agent_data = None

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_callback)

    def update_agent_data(self, new_data):
        with self.data_lock:
            self.agent_data = new_data

    def control_callback(self):
        with self.data_lock:
            if self.agent_data is not None:
                cmd = self.process_agent_data(self.agent_data)
                self.cmd_publisher.publish(cmd)
```

### **2. Memory Management**

```python
# Use object pooling for message creation
class MessagePool:
    def __init__(self):
        self.twist_pool = [Twist() for _ in range(10)]
        self.pool_index = 0

    def get_twist(self):
        msg = self.twist_pool[self.pool_index]
        self.pool_index = (self.pool_index + 1) % len(self.twist_pool)
        # Reset message fields
        msg.linear.x = msg.linear.y = msg.linear.z = 0.0
        msg.angular.x = msg.angular.y = msg.angular.z = 0.0
        return msg
```

### **3. Rate Limiting**

```python
from time import time

class RateLimitedBridge(Node):
    def __init__(self):
        super().__init__('rate_limited_bridge')
        self.last_publish_time = 0
        self.publish_interval = 0.02  # 50 Hz

        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.001, self.control_loop)  # Fast timer

    def control_loop(self):
        current_time = time()
        if current_time - self.last_publish_time >= self.publish_interval:
            cmd = self.get_agent_command()
            if cmd:
                self.cmd_publisher.publish(cmd)
                self.last_publish_time = current_time
```

---

## 🔐 **Safety and Error Handling**

### **1. Safety Monitors**

```python
class SafeAgentBridge(Node):
    def __init__(self):
        super().__init__('safe_agent_bridge')

        # Safety parameters
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('safety_timeout', 1.0)

        self.last_command_time = self.get_clock().now()
        self.emergency_stop_active = False

        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.safety_check)

    def publish_command(self, cmd):
        # Apply safety limits
        cmd.linear.x = max(-self.get_parameter('max_linear_vel').value,
                          min(self.get_parameter('max_linear_vel').value, cmd.linear.x))
        cmd.angular.z = max(-self.get_parameter('max_angular_vel').value,
                           min(self.get_parameter('max_angular_vel').value, cmd.angular.z))

        # Update last command time
        self.last_command_time = self.get_clock().now()
        self.cmd_publisher.publish(cmd)

    def safety_check(self):
        # Check for timeout
        time_since_last = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if time_since_last > self.get_parameter('safety_timeout').value:
            self.emergency_stop()

    def emergency_stop(self):
        if not self.emergency_stop_active:
            stop_cmd = Twist()
            self.cmd_publisher.publish(stop_cmd)
            self.emergency_stop_active = True
            self.get_logger().warn('Emergency stop activated!')
```

### **2. Error Recovery**

```python
class ResilientAgentBridge(Node):
    def __init__(self):
        super().__init__('resilient_agent_bridge')

        self.connection_attempts = 0
        self.max_reconnect_attempts = 5

        # Initialize publishers/subscribers with error handling
        try:
            self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            self.get_logger().info('Publisher created successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to create publisher: {e}')
            self.handle_publisher_error()

    def handle_publisher_error(self):
        self.connection_attempts += 1
        if self.connection_attempts <= self.max_reconnect_attempts:
            self.get_logger().info(f'Attempting to reconnect... ({self.connection_attempts}/{self.max_reconnect_attempts})')
            # Schedule reconnection attempt
            self.create_timer(1.0, self.reconnect_publisher)
        else:
            self.get_logger().error('Max reconnection attempts reached')

    def reconnect_publisher(self):
        try:
            self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            self.connection_attempts = 0
            self.get_logger().info('Publisher reconnected successfully')
        except Exception as e:
            self.handle_publisher_error()
```

---

## 🤖 **Real-World Applications**

### **1. Autonomous Navigation**

* Path planning algorithms sending velocity commands
* Obstacle avoidance integration
* Multi-robot coordination
* Dynamic replanning based on sensor data

### **2. Manipulation Tasks**

* Inverse kinematics solutions
* Grasping algorithms
* Trajectory planning for end-effectors
* Force control integration

### **3. Learning Systems**

* Reinforcement learning policy deployment
* Imitation learning integration
* Online learning with real robots
* Simulation-to-reality transfer

### **4. Human-Robot Interaction**

* Natural language processing integration
* Gesture recognition systems
* Collaborative task execution
* Safety-aware interaction

---

## 📊 **Monitoring and Debugging**

### **1. Built-in Tools**

```bash
# Monitor bridge performance
ros2 topic hz /cmd_vel

# Echo agent commands
ros2 topic echo /agent_commands

# Check node status
ros2 node info agent_bridge

# Monitor parameters
ros2 param list agent_bridge
```

### **2. Custom Monitoring**

```python
class MonitoredAgentBridge(Node):
    def __init__(self):
        super().__init__('monitored_agent_bridge')

        # Monitoring variables
        self.command_count = 0
        self.last_command_time = self.get_clock().now()
        self.avg_command_rate = 0.0

        # Publishers for monitoring
        self.status_publisher = self.create_publisher(String, '/bridge_status', 10)

        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_command(self, cmd):
        self.cmd_publisher.publish(cmd)
        self.command_count += 1

    def publish_status(self):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_command_time).nanoseconds / 1e9
        self.avg_command_rate = self.command_count / time_diff if time_diff > 0 else 0

        status_msg = String()
        status_msg.data = f"Commands: {self.command_count}, Rate: {self.avg_command_rate:.2f} Hz"
        self.status_publisher.publish(status_msg)

        # Reset for next interval
        self.last_command_time = current_time
```

---

## ✅ **Best Practices**

### **1. Design Principles**

* **Separation of Concerns**: Keep AI logic separate from ROS communication
* **Modularity**: Design reusable bridge components
* **Scalability**: Design for multiple agents/controllers
* **Robustness**: Handle errors gracefully

### **2. Performance Guidelines**

* **Rate Control**: Limit command frequency appropriately
* **Message Optimization**: Minimize message size and frequency
* **Threading**: Use appropriate threading models
* **Memory Management**: Avoid memory leaks in long-running systems

### **3. Safety Considerations**

* **Limits**: Apply velocity, position, and force limits
* **Timeouts**: Implement command timeouts
* **Validation**: Validate agent outputs before execution
* **Emergency Stop**: Provide immediate stop capability

---

## 📚 **Advanced Topics**

### **1. Custom Message Types**

```python
# Define custom message for complex agent actions
# In msg/AgentAction.msg:
# string agent_id
# string action_type
# float64[] parameters
# time timestamp
```

### **2. Integration with ROS 2 Control**

```python
# Using ros2_control interfaces
from controller_manager_msgs.srv import SwitchController

class AdvancedAgentBridge(Node):
    def __init__(self):
        super().__init__('advanced_agent_bridge')

        # Service client for controller switching
        self.controller_switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
```

---

## ✅ **Summary**

**Bridging Python agents to ROS controllers using `rclpy`** is a **critical component** of modern robotic systems, enabling the integration of sophisticated artificial intelligence with real-time control systems. This bridge allows:

* **AI algorithms** to control physical robots
* **Modular system design** with clear separation of concerns
* **Scalable multi-agent systems**
* **Real-time performance** with appropriate safety measures
* **Flexible communication patterns** (topics, services, actions)

The successful implementation of agent-controller bridges requires careful attention to **performance**, **safety**, and **reliability** to ensure robust operation in real-world applications 🤖

---

## 📚 **Further Reading**

* ROS 2 Control Documentation: https://control.ros.org/
* rclpy API: https://docs.ros2.org/latest/api/rclpy/
* ROS 2 Design: https://design.ros2.org/
* Robot Operating System 2: The Complete Reference: https://link.springer.com/book/10.1007/978-3-030-70490-5
