---
sidebar_position: 2
title: Nodes, Topics, and Services
description: Master ROS 2 communication patterns with practical examples
---

# Nodes, Topics, and Services

## Overview

In this chapter, you'll learn the three fundamental communication patterns in ROS 2:

1. **Nodes**: Independent processes that perform computation
2. **Topics**: Asynchronous publish-subscribe messaging
3. **Services**: Synchronous request-response communication

By the end, you'll build a complete publisher-subscriber system and a service-based calculator.

## Understanding Nodes

### What is a Node?

A **node** is a process that performs computation. In ROS 2:

- Each node is an independent entity
- Nodes communicate via topics, services, or actions
- A robot system typically has many nodes working together
- Examples: camera driver, motion planner, controller

### Why Multiple Nodes?

Benefits of modular node architecture:

- **Fault Isolation**: One node crash doesn't bring down the system
- **Reusability**: Share nodes across projects
- **Parallel Development**: Teams can work on different nodes
- **Testing**: Test nodes independently
- **Scalability**: Distribute nodes across multiple machines

## Creating Your First Node

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello from ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components Explained

1. **`rclpy.init()`**: Initialize the ROS 2 Python client library
2. **`Node` class**: Base class for all ROS 2 nodes
3. **`super().__init__('my_first_node')`**: Name your node (must be unique)
4. **`rclpy.spin()`**: Keep the node running and processing callbacks
5. **`rclpy.shutdown()`**: Clean up resources when done

### Running Your Node

```bash
# Save as my_first_node.py
python3 my_first_node.py

# Output:
# [INFO] [1234567890.123456789] [my_first_node]: Hello from ROS 2!
```

## Topics: Publish-Subscribe Pattern

### What are Topics?

**Topics** enable asynchronous, one-to-many communication:

- Publishers send messages to topics
- Subscribers receive messages from topics
- Many-to-many: Multiple publishers/subscribers per topic
- Asynchronous: No waiting for acknowledgment

### Use Cases

- Sensor data streams (camera, LiDAR, IMU)
- Robot state broadcasting
- Continuous monitoring
- Any high-frequency data

## Creating a Publisher

### Simple String Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Create publisher: (message_type, topic_name, queue_size)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Create timer: publish every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info('Publisher node started')
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher Components

- **`create_publisher(Type, topic, qos_profile)`**: Create publisher object
  - `Type`: Message type (e.g., `String`, `Image`, `LaserScan`)
  - `topic`: Topic name as string
  - `qos_profile`: Quality of Service settings (we'll cover later)
  
- **`create_timer(period, callback)`**: Schedule periodic callbacks
  - `period`: Seconds between calls
  - `callback`: Function to execute

- **`publish(msg)`**: Send message to topic

### Running the Publisher

```bash
# Terminal 1: Run publisher
python3 minimal_publisher.py

# Terminal 2: Echo messages
ros2 topic echo /chatter

# Terminal 3: Monitor topic info
ros2 topic info /chatter
ros2 topic hz /chatter  # Check publishing rate
```

## Creating a Subscriber

### Simple String Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Create subscriber: (message_type, topic_name, callback, queue_size)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        
        self.get_logger().info('Subscriber node started')
    
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Components

- **`create_subscription(Type, topic, callback, qos)`**: Create subscriber
  - Automatically called when message arrives
  - `callback`: Function that processes received messages

### Testing Publisher + Subscriber

```bash
# Terminal 1
python3 minimal_publisher.py

# Terminal 2
python3 minimal_subscriber.py

# You should see:
# [Publisher] Publishing: "Hello ROS 2! Count: 0"
# [Subscriber] I heard: "Hello ROS 2! Count: 0"
# [Publisher] Publishing: "Hello ROS 2! Count: 1"
# [Subscriber] I heard: "Hello ROS 2! Count: 1"
```

## Services: Request-Response Pattern

### What are Services?

**Services** provide synchronous request-response communication:

- Client sends request, waits for response
- Server processes request, sends back response
- One-to-one: Each request matched with one response
- Synchronous: Client blocks until response received

### Use Cases

- Getting robot configuration
- Triggering actions (e.g., "take photo")
- Querying state
- Any operation requiring confirmation

## Creating a Service Server

### Add Two Integers Service

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        
        # Create service: (service_type, service_name, callback)
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        
        self.get_logger().info('Service server ready')
    
    def add_two_ints_callback(self, request, response):
        # Process request
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Server Components

- **`create_service(Type, name, callback)`**: Create service server
  - `callback`: Function with signature `(request, response) -> response`
  - Request and response types defined in `.srv` files

## Creating a Service Client

### Calling the Add Service

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        
        # Create client: (service_type, service_name)
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        self.get_logger().info('Service client ready')
    
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # Asynchronous call
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = MinimalClientAsync()
    
    # Send request
    future = node.send_request(5, 7)
    
    # Wait for response
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        response = future.result()
        node.get_logger().info(f'Result: {response.sum}')
    else:
        node.get_logger().error('Service call failed')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing Service

```bash
# Terminal 1: Start server
python3 minimal_service.py

# Terminal 2: Call from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"

# Or run client
python3 minimal_client_async.py
```

## Quality of Service (QoS)

### What is QoS?

QoS settings control message delivery behavior:

- **Reliability**: Best effort vs guaranteed delivery
- **Durability**: Transient local (keep last N) vs volatile
- **History**: Keep last N vs keep all
- **Liveliness**: Automatic, manual node, or manual topic

### Common QoS Profiles

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Sensor data profile (best effort, volatile)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Reliable profile (guaranteed delivery)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Create publisher with QoS
self.publisher_ = self.create_publisher(
    String,
    'topic',
    sensor_qos  # Use custom QoS
)
```

### When to Use Which QoS

| Use Case | Reliability | Durability | Example |
|----------|-------------|------------|---------|
| High-frequency sensors | Best Effort | Volatile | Camera, LiDAR |
| Critical commands | Reliable | Transient Local | Motor commands |
| Configuration | Reliable | Transient Local | Robot parameters |
| State updates | Best Effort | Volatile | Joint states |

## Practical Exercise: Robot Velocity Controller

### Challenge

Create a system with:
1. **Publisher Node**: Publishes target velocities
2. **Subscriber Node**: Receives and "executes" velocities
3. **Service Node**: Provides emergency stop service

### Solution Outline

```python
# velocity_publisher.py
# Publishes geometry_msgs/Twist to /cmd_vel

# velocity_controller.py
# Subscribes to /cmd_vel
# Provides /emergency_stop service

# emergency_stop_client.py
# Calls /emergency_stop service
```

Try implementing this yourself! Hints:

- Use `geometry_msgs.msg.Twist` for velocity commands
- Use `std_srvs.srv.Trigger` for emergency stop
- Add state tracking (moving/stopped)

## Common Debugging Commands

```bash
# List all nodes
ros2 node list

# Get node info
ros2 node info /my_node

# List topics
ros2 topic list

# Topic info
ros2 topic info /chatter

# Echo topic messages
ros2 topic echo /chatter

# Publish to topic
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello'"

# List services
ros2 service list

# Service info
ros2 service type /add_two_ints

# Call service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

## Key Takeaways

1. **Nodes** are independent processes that form your robot system
2. **Topics** are for asynchronous, many-to-many communication (sensors, state)
3. **Services** are for synchronous, request-response patterns (commands, queries)
4. **QoS** settings control message delivery guarantees
5. Use ROS 2 CLI tools for debugging and testing

## Next Steps

Now that you understand ROS 2 communication basics, you're ready to explore:

→ **Next Chapter**: [Parameters and Launch Files](./03-parameters-launch.md) *(coming soon)*

Or dive into simulation:

→ **Module 2**: [Gazebo Simulation](../module-02-gazebo/01-introduction.md) *(coming soon)*

## Additional Resources

- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)
- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts.html)
- [Message Types Reference](https://docs.ros2.org/humble/api/)
- [QoS Guide](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

---

**💡 Pro Tip**: Always test nodes individually before integrating them. Use `ros2 topic echo` and `ros2 service call` to verify behavior without writing test code.

Happy coding! 🚀
