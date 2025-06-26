# PathPlannerNode

## Overview

The `PathPlannerNode` class is a ROS2 node responsible for path planning in the Tekbot robot navigation stack.  
It subscribes to the map, odometry, and goal topics, computes a path using the A* algorithm, and publishes the planned path for the robot to follow.

---

## Class: `PathPlannerNode`
```c++
class PathPlannerNode : public rclcpp::Node
{
    public:
    PathPlannerNode();
    ~PathPlannerNode() = default;

    private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr infos);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr infos);
    std::vector<Position> computePath();
    nav_msgs::msg::Path PositionToNav(std::vector<Position> path);
    void publishPath(nav_msgs::msg::Path path);

    nav_msgs::msg::OccupancyGrid _map;
    geometry_msgs::msg::Pose _current_position;
    geometry_msgs::msg::Pose _goal_position;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_subscriber;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_publisher;
};
```
  - Definition of the PathPlannerNode class, inheriting from `rclcpp::Node`

### Attributes

- **_map**:  
  `nav_msgs::msg::OccupancyGrid`  
  Stores the latest received occupancy grid map.

- **_current_position**:  
  `geometry_msgs::msg::Pose`  
  Stores the robot's current pose (from odometry).

- **_goal_position**:  
  `geometry_msgs::msg::Pose`  
  Stores the target pose to reach.

- **_map_subscriber**:  
  Subscription to `/map` topic.

- **_odom_subscriber**:  
  Subscription to `/odom` topic.

- **_goal_subscriber**:  
  Subscription to `/goal_pose` topic.

- **_path_publisher**:  
  Publisher for `/planned_path` topic.

---

### Methods

#### Constructor and Subscriptions

```c++
PathPlannerNode::PathPlannerNode() : Node("path_planner_node")
{
    //subscribe to the occupancy grid map published on /map
    _map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PathPlannerNode::map_callback, this, std::placeholders::_1));

    //subscribe to odometry for current robot pose, published on /odom
    _odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PathPlannerNode::odom_callback, this, std::placeholders::_1));

    //subscribe to goal pose, published on /goal_pose
    _goal_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PathPlannerNode::goal_callback, this, std::placeholders::_1));

    //publisher for the planned path, published on /planned_path
    _path_publisher = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
}
```
- **Purpose:** Initializes subscriptions and publisher for map, odometry, goal, and path

---

#### Map Callback

```c++
void PathPlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    //store the latest map for use in path planning
    _map = *map;
}
```
- **Purpose:** Keeps the node's internal map up to date for path computation.

---

#### Odometry Callback

```c++
void PathPlannerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr infos)
{
    //update the current robot pose for use as the path start
    _current_position = infos->pose.pose;
}
```
- **Purpose:** Tracks the robot's current position in the environment.

---

#### Goal Callback and Path Computation

```c++
void PathPlannerNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr infos)
{
    //update the goal position to reach
    _goal_position = infos->pose;
    //compute the path from current to goal position
    nav_msgs::msg::Path path = this->PositionToNav(this->computePath());
    //publish the computed path for the robot to follow
    this->publishPath(path);
}
```
- **Purpose:** Triggers path computation and publishing when a new goal is set on /goal_pose topic.

---

#### Path Conversion

```c++
nav_msgs::msg::Path PathPlannerNode::PositionToNav(std::vector<Position> path)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map"; //set the frame to map
    path_msg.header.stamp = this->now(); //the timestamp

    for (const auto& pos : path) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = this->now();

        //convert grid position (cell indices) to world coordinates
        pose_stamped.pose.position.x = _map.info.origin.position.x + pos.x * _map.info.resolution;
        pose_stamped.pose.position.y = _map.info.origin.position.y + pos.y * _map.info.resolution;
        pose_stamped.pose.position.z = 0.0;
        //set orientation to identity (no rotation)
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;

        // Add this pose to the path
        path_msg.poses.push_back(pose_stamped);
    }

    return path_msg;
}
```
- **Purpose:** Converts the computed path (as grid indices) into a ROS2 Path message for visualization and execution.

---

#### Path Computation (A* Algorithm)

```c++
std::vector<Position> PathPlannerNode::computePath(void)
{
    int width = _map.info.width;
    int height = _map.info.height;
    //initialize the A* algorithm with the current map
    AStar astar_algo(_map);

    //convert current and goal positions from world coordinates to grid indices
    Position start = {static_cast<int>(_current_position.position.x), static_cast<int>(_current_position.position.y), 0};
    Position end = {static_cast<int>(_goal_position.position.x), static_cast<int>(_goal_position.position.y), 0};

    //create the A* grid and open list
    auto grid = astar_algo.create_astar_grid(height, width, end);
    std::vector<t_astar_node*> open_list;

    //initialize the A* search
    astar_algo.initialize_astar(grid, start, end, open_list);

    std::vector<Position> path;
    t_astar_node* current = nullptr;

    //A* loop: search for the optimal path
    while (!open_list.empty()) {
        //get the node with the lowest cost (f)
        current = astar_algo.pop_lowest_f(open_list);

        if (!current) break;

        current->visited = true;
        //if the goal is reached, reconstruct the path
        if (astar_algo.is_end_node(current, end)) {
            path = astar_algo.reconstruct_path(current);
            astar_algo.delete_astar_grid(grid);
            return path;
        }
        //process all valid neighbors of the current node
        astar_algo.process_neighbors(grid, current, end, open_list);
    }
    //clean up and return an empty path if no solution found
    astar_algo.delete_astar_grid(grid);
    return {};
}
```
 - **Purpose:** Implements the A* pathfinding algorithm to find a collision-free path from the robot's current position to the goal.

---

#### Path Publishing

```c++
void PathPlannerNode::publishPath(nav_msgs::msg::Path path)
{
    path.header.frame_id = "map";
    _path_publisher->publish(path);
}
```
  - **Purpose:** Makes the computed path available to other nodes (motion_controller_node) by publishing on /lanned_path.

---

#### Main Function

```c++
int main(int ac, char **av)
{
    rclcpp::init(ac, av);
    rclcpp::shutdown();
    return 0;
}
```
- **Purpose:** Launches the PathPlannerNode and keeps it running until shutdown.

---