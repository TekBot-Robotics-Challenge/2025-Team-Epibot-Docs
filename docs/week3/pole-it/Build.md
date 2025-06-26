# Documentation – Building & Launching `launch.py` Files

## 1. Prerequisites

Make sure you have installed:

* **ROS 2 Humble** (`source /opt/ros/humble/setup.bash`)
* `colcon` for building
* `rosdep` for dependency management

---

## 2. Workspace Initialization

If not already done:

```bash
cd ~/tekbot_ws
colcon build
source install/setup.bash
```

---

## 3. Install ROS 2 Dependencies (if needed)

From the workspace:

```bash
cd ~/tekbot_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4. Building the Workspace

```bash
cd ~/tekbot_ws
colcon build
```

> If you encounter issues:

```bash
colcon build --packages-select <package_name> --symlink-install --event-handlers console_direct+
```

---

## 5. Launching `launch.py` Files

General syntax:

```bash
ros2 launch <package_name> <launch_file_name>.launch.py
```

### Launch the tekbot robot on the maze

To start gazbo and see the tekbot robot on the maze:

```bash
ros2 launch maze_solving tekbot_maze.launch.py
```

<video src="/gazebo_launch.webm" controls autoplay muted style="width: 100%; max-width: 800px; height: auto;">
  Your browser does not support the video tag.
</video>

---

### Launch the maze solving

To launch the launch file in oder to solve the maze :

```bash
ros2 launch maze_solving maze.launch.py
```

<video src="/path_finding_launch.webm" controls autoplay muted style="width: 100%; max-width: 800px; height: auto;">
  Your browser does not support the video tag.
</video>


---

### Launch the rviz

To launch rviz in oder to vizualise the mapping:

```bash
ros2 launch tekbot_description rviz.launch.py
```

<video src="/rviz_launch.webm" controls autoplay muted style="width: 100%; max-width: 800px; height: auto;">
  Your browser does not support the video tag.
</video>

---
