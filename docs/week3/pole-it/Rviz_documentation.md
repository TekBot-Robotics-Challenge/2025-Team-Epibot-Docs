# RVIZ2 DOCUMENTATION 

## Introduction

In this part of documanetation you will find all information about RViz2 Configuration and Visualization for our Autonomous Navigation robot.

i will describe the setup and usage of RViz2 for visualizing data during the autonomous navigation test using ROS 2 Humble, Gazebo Classic, and SLAM Toolbox. All visualizations were configured manually witshin RViz2 to monitor the TekBot's state, sensor readings, and navigation behavior.

## Installing RViz2 on Ubuntu 22.04 (ROS 2 Humble)

Ensure ROS 2 Humble is installed on the system.
```bash
ros2 --version
```

In RViz2 is installed by default with the ROS 2 desktop version. If not, install it manually:

```bash
sudo apt install ros-humble-rviz2
```
Source the ROS 2 workspace before launching:
source ./tekbot_sim/install/setup.bash
```
Launch RViz2 using:
```bash
rviz2
```

I used the official tekbot_sim repository which contains both the robot description and the simulation environment. The maze simulation was launched using:

```bash
ros2 launch maze_solving tekbot_maze.launch.py
```

This command starts the simulation in Gazebo Classic and spawns the TekBot robot in a pre-defined maze. We used SLAM Toolbox to map the environment and localize the robot.

I configured RViz2 to visualize different data streams relevant to the robot's navigation process. I manually added several display types and linked them to the correct ROS 2 topics.

when we launch rviz2 you
To add display types on the left panel 

### RobotModel
robot_description (parameter)

### TF
Shows the transform frames between robot components `/tf`
### LaserScan
Visualizes LIDAR data used for obstacle detection `/scan`
### Odometry
Shows the robot’s estimated position based on wheel encoders `/odom`
### Path
Displays the path calculated by the pathfinding algorithm `/plan`
### Map
Displays the SLAM-generated 2D occupancy grid `/map`
The fixed frame that we use in RViz2 is`base_link`
  
This is the test Procedure
-  Cloned and built the tekbot_sim repository using and launch the differents nodes
- Launched the robot in Gazebo using the provided launch file.
```bash
    git clone https://github.com/charif-tekbot/tekbot_sim.git
    ros2 launch maze_solving tekbot_maze.launch.py
```
- Launched RViz2 in a separate terminal session.
- Manually added all necessary display types in RViz2 [Check here to learn more](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)
- Verified that each topic was publishing data by using ros2 topic echo
- Used SLAM Toolbox to generate the map and track robot localization.

Conclusion
Through RViz2, I was able to monitor the robot’s pose, the SLAM map, sensor input, and planned navigation path. This setup was essential for debugging and validating the robot’s behavior in the maze environment.
