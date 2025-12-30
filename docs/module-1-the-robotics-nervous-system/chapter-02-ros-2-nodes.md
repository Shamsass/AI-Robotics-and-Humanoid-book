---
title: Chapter 02 ROS 2 Nodes
---

## **ROS 2 Nodes: The Building Blocks of Robotic Systems** 🤖

![Image](https://navigation.ros.org/_images/ros2_architecture.png)

**ROS 2 nodes** are the **fundamental building blocks** of robotic systems, serving as independent software components that work together to control a robot. They enable **modular design**, **efficient communication**, and **scalable robotic systems** by providing a standardized way to organize and execute robot functionality.

---

## 🧠 **Understanding ROS 2 Nodes**

### **What is a Node?**

A **node** is a **process** that performs computation in ROS 2. It's an instance of a program that:
* **Communicates** with other nodes
* **Publishes** and **subscribes** to topics
* **Provides** and **uses** services
* **Executes** actions
* **Manages** robot functionality

### **Key Characteristics**

* **Independent**: Each node runs in its own process
* **Modular**: Functions can be separated into different nodes
* **Communicative**: Nodes interact through ROS 2 communication primitives
* **Composable**: Nodes can be combined to create complex behaviors
* **Distributed**: Nodes can run on different machines

---

## 🏗️ **Node Architecture**

### **Basic Node Structure**

```
Node Process
├── Node Handle
│   ├── Publishers
│   ├── Subscribers
│   ├── Services
│   ├── Actions
│   └── Parameters
├── Callback Groups
├── Timers
└── Execution Logic
```

### **Node Lifecycle**

1. **Initialization**: Node creates ROS 2 context
2. **Setup**: Publishers, subscribers, services created
3. **Execution**: Main loop or callbacks process data
4. **Shutdown**: Resources cleaned up properly

---

## 🔧 **Creating ROS 2 Nodes**

### **Python Implementation**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create publisher
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Create subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Robot Controller Node Started')

    def laser_callback(self, msg):
        # Process laser scan data
        self.min_distance = min(msg.ranges)

    def control_loop(self):
        # Implement control logic
        cmd = Twist()
        if self.min_distance > 1.0:
            cmd.linear.x = 0.5  # Move forward
        else:
            cmd.angular.z = 0.5  # Turn

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **C++ Implementation**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {
        // Create publisher
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // Create subscriber
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&RobotController::laser_callback, this, std::placeholders::_1));

        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Robot Controller Node Started");
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process laser scan data
        min_distance_ = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    }

    void control_loop()
    {
        auto cmd = geometry_msgs::msg::Twist();
        if (min_distance_ > 1.0) {
            cmd.linear.x = 0.5;  // Move forward
        } else {
            cmd.angular.z = 0.5; // Turn
        }
        cmd_publisher_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    float min_distance_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 🔄 **Node Communication Patterns**

### **1. Publisher-Subscriber (Topics)**

* **Asynchronous** communication
* **Many-to-many** data distribution
* **Real-time** capable
* **Decoupled** timing

```python
# Publisher
publisher = node.create_publisher(String, 'topic_name', 10)
msg = String()
msg.data = 'Hello World'
publisher.publish(msg)

# Subscriber
def callback(msg):
    print(msg.data)

subscription = node.create_subscription(String, 'topic_name', callback, 10)
```

### **2. Client-Server (Services)**

* **Synchronous** request-response
* **One-to-one** interaction
* **Blocking** until response
* **Reliable** delivery

```python
# Service Server
from example_interfaces.srv import AddTwoInts

def add_two_ints_callback(request, response):
    response.sum = request.a + request.b
    return response

service = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)

# Service Client
client = node.create_client(AddTwoInts, 'add_two_ints')
request = AddTwoInts.Request()
request.a = 1
request.b = 2
future = client.call_async(request)
```

### **3. Action-Based Communication**

* **Asynchronous** with feedback
* **Long-running** operations
* **Goal/Result/Feedback** pattern
* **Cancel/Preemption** support

```python
# Action Server
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciActionServer:
    def __init__(self):
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

---

## ⚙️ **Node Management**

### **Launching Nodes**

#### **1. Command Line**
```bash
# Run a single node
ros2 run package_name node_name

# Run with arguments
ros2 run package_name node_name --ros-args --param param_name:=value

# List active nodes
ros2 node list
```

#### **2. Launch Files (Python)**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package_name',
            executable='node_name',
            name='node_name',
            parameters=[
                {'param1': 'value1'},
                {'param2': 123}
            ],
            remappings=[
                ('original_topic', 'new_topic')
            ]
        )
    ])
```

#### **3. Composable Nodes**
```python
from rclpy_components import ComponentManager

# Run multiple nodes in a single process
# More efficient than separate processes
# Better for resource-constrained systems
```

---

## 🧪 **Node Composition Patterns**

### **1. Single Responsibility**
* Each node handles one specific function
* Easy to debug and maintain
* High modularity

### **2. Functional Grouping**
* Related functions in one node
* Reduced communication overhead
* Better performance for tightly coupled functions

### **3. Hierarchical Organization**
* Master nodes coordinate child nodes
* Clear command structure
* Scalable for complex systems

---

## 🚀 **Advanced Node Features**

### **1. Parameters**
```python
# Declare parameters
self.declare_parameter('wheel_radius', 0.05)
self.declare_parameter('max_velocity', 1.0)

# Get parameter values
wheel_radius = self.get_parameter('wheel_radius').value
max_velocity = self.get_parameter('max_velocity').value
```

### **2. Quality of Service (QoS)**
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

publisher = node.create_publisher(String, 'topic', qos_profile)
```

### **3. Callback Groups**
```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Separate callback groups for different priorities
cb_group = MutuallyExclusiveCallbackGroup()
timer = node.create_timer(0.1, callback, callback_group=cb_group)
```

---

## 🤖 **Real-World Node Examples**

### **1. Sensor Processing Node**
* Subscribes to sensor data
* Processes and filters data
* Publishes processed information
* Handles sensor calibration

### **2. Motion Control Node**
* Receives high-level commands
* Generates low-level motor commands
* Monitors robot state
* Implements safety checks

### **3. Navigation Node**
* Plans paths to goals
* Avoids obstacles
* Coordinates with other nodes
* Manages navigation state

### **4. Perception Node**
* Processes camera/lidar data
* Detects objects/features
* Tracks moving objects
* Builds environmental models

---

## ⚡ **Performance Considerations**

### **1. Resource Management**
* **Memory usage**: Efficient data structures
* **CPU load**: Optimize algorithms
* **Network bandwidth**: Compress data when needed
* **Battery life**: Optimize for power consumption

### **2. Real-Time Performance**
* **Timing constraints**: Meet deadlines
* **Deterministic behavior**: Predictable execution
* **Priority scheduling**: Critical tasks first
* **Interrupt handling**: Fast response to events

### **3. Communication Optimization**
* **Message frequency**: Balance update rate vs. bandwidth
* **Data size**: Minimize message payload
* **Connection management**: Efficient connection handling
* **Threading**: Proper thread safety

---

## 🔐 **Security and Safety**

### **1. Node Security**
* **Authentication**: Verify node identity
* **Authorization**: Control node permissions
* **Encryption**: Secure data transmission
* **Firewall**: Network access control

### **2. Safety Mechanisms**
* **Emergency stops**: Immediate shutdown capability
* **Watchdogs**: Monitor node health
* **State validation**: Verify data integrity
* **Fault tolerance**: Handle failures gracefully

---

## 📊 **Node Monitoring and Debugging**

### **1. Built-in Tools**
```bash
# List nodes
ros2 node list

# Show node info
ros2 node info <node_name>

# Echo topics
ros2 topic echo <topic_name>

# Call services
ros2 service call <service_name> <service_type> <request_data>
```

### **2. Logging**
```python
# Different log levels
node.get_logger().debug('Debug message')
node.get_logger().info('Info message')
node.get_logger().warn('Warning message')
node.get_logger().error('Error message')
node.get_logger().fatal('Fatal message')
```

### **3. Visualization**
* **RViz**: 3D visualization of robot data
* **rqt**: GUI tools for monitoring
* **PlotJuggler**: Real-time plotting
* **Custom dashboards**: Web-based monitoring

---

## ✅ **Summary**

**ROS 2 nodes** are the **essential building blocks** of modern robotic systems, providing a standardized way to organize, execute, and coordinate robot functionality. They enable:

* **Modular design** for maintainable code
* **Scalable systems** that can grow with complexity
* **Distributed computing** across multiple machines
* **Interoperability** between different components
* **Robust communication** patterns for reliable operation

Understanding nodes is **crucial** for developing effective robotic systems that can handle the complexity of real-world applications 🤖

---

## 📚 **Further Reading**

* ROS 2 Documentation: https://docs.ros.org/
* ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
* rclpy API: https://docs.ros2.org/latest/api/rclpy/
* ROS 2 Design: https://design.ros2.org/
