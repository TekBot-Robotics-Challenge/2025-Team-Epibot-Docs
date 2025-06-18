# ROS 2 Launch File Documentation: `sensor_launch.py`

This document provides a complete explanation of the `sensor_launch.py` file used to launch two ROS 2 nodes — a publisher and a subscriber — in the `sensor_data_evaluation` package.

---

## 📁 File Location

This launch file is expected at:

```Bash
sensor_data_evaluation/launch/sensor_launch.py
```


---

## Purpose

The launch file automates the execution of two nodes:
- `publisher_node`: publishes messages
- `subscriber_node`: subscribes and listens to messages

This simplifies testing and development in a typical ROS 2 Publisher/Subscriber setup.

---

## Prerequisites

Before using this launch file, ensure:

1. The ROS 2 package `sensor_data_evaluation` exists.
   
2. It builds two executables:
   - `publisher_node`
   - `subscriber_node`
  
3. Your `CMakeLists.txt` includes this to install the `launch/` folder:
   ```Cmake
   install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
   ```

4. You build the package with:
    ```Bash
    colcon build --packages-select sensor_data_evaluation
    ```

5. Then source your workspace:
    ```Bash
    source install/setup.bash
    ```

## Launch File Content

Create the file at `sensor_data_evaluation/launch/sensor_launch.py` with the following content:

```Python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_data_evaluation',
            executable='publisher_node',
            name='publisher',
        ),
        Node(
            package='sensor_data_evaluation',
            executable='subscriber_node',
            name='subscriber',
        )
    ])
```

### Line-by-Line Breakdown

| Line                                   | Description                                                                                                                            |
| -------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| `from launch import LaunchDescription` | Imports the class for defining a launch description.                                                                                   |
| `from launch_ros.actions import Node`  | Imports the `Node` class to define ROS 2 nodes in launch files.                                                                        |
| `generate_launch_description()`        | Standard function ROS 2 looks for in a launch file.                                                                                    |
| `LaunchDescription([...])`             | Returns a list of nodes/actions to be launched.                                                                                        |
| `Node(...)`                            | Specifies each node with:<br>• `package`: the ROS 2 package<br>• `executable`: compiled node binary<br>• `name`: the runtime node name |

---

##  How to Run the Launch File

To start both nodes via the launch file:

```Bash
ros2 launch sensor_data_evaluation sensor_launch.py
```

### Expected behavior:

- publisher_node and subscriber_node will both be started.
  
- You should see output in the terminal for both nodes.
 
- Use ros2 node list to verify that the nodes are running.

---

## How to Test Nodes Individually

You can run each node on its own using:

```Bash
ros2 run sensor_data_evaluation publisher_node
ros2 run sensor_data_evaluation subscriber_node
```

## Clean and Rebuild

If needed, clean and rebuild your workspace with:

```Bash
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

## Verification Checklist

- Launch file is placed in launch/ inside your package. 
  
- It is installed via CMakeLists.txt.
  
- Launching works with ros2 launch sensor_data_evaluation sensor_launch.py.
  
- Nodes appear with ros2 node list.

## References

- ROS 2 Launch Tutorial: <https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html>
- ROS 2 Nodes: <https://docs.ros.org/en/ros2_documentation/kilted/Concepts/Basic/About-Nodes.html>
- `colcon build:` <https://colcon.readthedocs.io>

