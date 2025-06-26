# MotionControllerNode

## Overview

The `MotionControllerNode` class is a ROS2 node responsible for following the planned path by **`PathPlannerNode`** and sending velocity commands to the Tekbot robot.  
It subscribes to the planned path and odometry topics, computes the required velocity to reach the next waypoint, and publishes velocity commands on `/cmd_vel`.

---

## Class: `MotionControllerNode`
```c++
class MotionControllerNode : public rclcpp::Node
{
    public:
    MotionControllerNode();
    ~MotionControllerNode() = default;

    private:
    nav_msgs::msg::Path _current_path;
    geometry_msgs::msg::Pose _current_pose;

    void plannedPathCallback(const nav_msgs::msg::Path::SharedPtr path);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr infos);
    void traceReceivedPath(void);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _planned_path_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;
};
```
  - Definition of the MotionControllerNode class, inheriting from `rclcpp::Node`

### Attributes

- **_current_path**:  
  `nav_msgs::msg::Path`  
  Stores the latest received path to follow.

- **_current_pose**:  
  `geometry_msgs::msg::Pose`  
  Stores the robot's current pose (from odometry).

- **_planned_path_subscriber**:  
  Subscription to `/planned_path` topic.

- **_odom_subscriber**:  
  Subscription to `/odom` topic.

- **_cmd_vel_publisher**:  
  Publisher for `/cmd_vel` topic.

---

### Methods

#### Constructor and Subscriptions

```c++
MotionControllerNode::MotionControllerNode() : Node("motion_controller_node")
{
    //subscribe to the planned path published on /planned_path
    _planned_path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10, std::bind(&MotionControllerNode::plannedPathCallback, this, std::placeholders::_1));
    //subscribe to odometry for current robot pose, published on /odom
    _odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MotionControllerNode::odomCallback, this, std::placeholders::_1));
    //publisher for velocity commands, published on /cmd_vel
    _cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}
```
- **Purpose:** Initializes subscriptions and publisher for path, odometry, and velocity commands
---

#### Planned Path Callback

```c++
void MotionControllerNode::plannedPathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    //store the path to follow
    _current_path = *path;
    return;
}
```
- **Purpose:** Updates the internal path to be followed by the robot.

---

#### Odometry Callback

```c++
void MotionControllerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr infos)
{
    //update the current robot pose
    _current_pose = infos->pose.pose;
    //follow the path using the updated pose
    traceReceivedPath();
    return;
}
```
- **Purpose:** Tracks the robot's current position and triggers path following logic.

---

#### Path Following Logic

```c++
void MotionControllerNode::traceReceivedPath(void)
{
    //if there is no path to follow, do nothing
    if (_current_path.poses.empty()) return;

    //take the first waypoint in the path as the immediate target
    const auto& target = _current_path.poses.front().pose;
    double dx = target.position.x - _current_pose.position.x;
    double dy = target.position.y - _current_pose.position.y;

    //proportional controller for linear and angular velocity
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.5 * sqrt(dx*dx + dy*dy);
    cmd_vel.angular.z = 2.0 * atan2(dy, dx);

    //publish the velocity command
    _cmd_vel_publisher->publish(cmd_vel);
    return;
}
```
- **Purpose:** Generates and sends velocity commands to move the robot towards the next waypoint.

---

#### Main Function

```c++
int main(int ac, char **av)
{
    rclcpp::init(ac, av);
    rclcpp::spin(std::make_shared<MotionControllerNode>());
    rclcpp::shutdown();
    return 0;
}
```
- **Purpose:** Launches the MotionControllerNode and keeps it running until shutdown.

---