---
title: "ROS 2 Practical Exercises"
module: "ros2"
word_count: 320
learning_objectives:
  - "Implement basic ROS 2 nodes in Python or C++"
  - "Create publishers and subscribers for topic communication"
  - "Develop services for request-response communication"
  - "Build action clients and servers for goal-oriented tasks"
prerequisites:
  - "Understanding of ROS 2 concepts and architecture"
  - "Basic programming skills in Python or C++"
references:
  - "@quigley2009ros"
validation_status: draft
---

# ROS 2 Practical Exercises

## Exercise 1: Creating Your First ROS 2 Node

### Objective
Create a simple ROS 2 node that publishes "Hello, Robot!" messages to a topic.

### Steps
1. Create a new ROS 2 package: `ros2 pkg create --build-type ament_python hello_robot`
2. Navigate to the package directory
3. Create a Python node file in `hello_robot/hello_robot/` directory
4. Import necessary ROS 2 libraries: `rclpy` and `std_msgs`
5. Create a node class that inherits from `Node`
6. Implement a publisher that sends string messages to `/robot_status` topic
7. Use a timer to publish messages every 2 seconds
8. Initialize the node and spin it

### Expected Output
The node should continuously publish "Hello, Robot!" messages to the `/robot_status` topic.

## Exercise 2: Publisher-Subscriber Communication

### Objective
Create two nodes: one publisher and one subscriber that communicate via a topic.

### Steps
1. Create a publisher node that sends integer messages to `/counter` topic
2. Create a subscriber node that listens to `/counter` and prints received values
3. Implement the publisher to increment a counter and publish values
4. Run both nodes in separate terminals
5. Verify that the subscriber receives and displays the counter values

### Expected Output
The subscriber should display the counter values published by the publisher node.

## Exercise 3: Service-Based Communication

### Objective
Create a service server and client that perform simple arithmetic operations.

### Steps
1. Define a service interface using `.srv` file (e.g., `AddTwoInts.srv`)
2. Create a service server that adds two integers
3. Create a service client that sends requests to the server
4. The client should send several requests with different values
5. The server should respond with the sum of the two integers

### Expected Output
The client should receive correct sums for each request sent to the server.

## Exercise 4: Action-Based Communication

### Objective
Implement an action server that simulates a robot moving to a goal position.

### Steps
1. Define an action interface using `.action` file (e.g., `MoveToGoal.action`)
2. Create an action server that simulates movement to a goal
3. Provide feedback during the movement simulation
4. Create an action client that sends goals to the server
5. Handle the feedback and final result from the server

### Expected Output
The client should receive feedback during the simulated movement and a final result when the goal is reached.

## Summary

These exercises provide hands-on experience with the fundamental ROS 2 communication patterns. Completing them will give you practical skills in developing robotic applications using ROS 2's node, topic, service, and action mechanisms.