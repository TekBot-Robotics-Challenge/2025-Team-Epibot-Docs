# Setup and quick start (Jetson Nano)

We list the minimal setup steps to run the Dofbot software found in `final_version`. This is a pragmatic checklist — adapt to your environment as needed.

1) System & ROS
- Install ROS (the code uses `moveit_commander`, `moveit` and `sensor_msgs` — ROS Noetic or a ROS1-compatible distribution is expected for the provided scripts).

2) Python dependencies
- Install the Python packages used in the repo:
  - `opencv-python` (cv2) — camera capture
  - `smbus2` — I2C communication
  - `catkin` / `rosdep` if using ROS packages

We recommend installing via pip inside your ROS environment:

```
pip3 install opencv-python smbus2
```

3) Enable I2C on Jetson / Raspberry Pi (if applicable)
- Make sure the I2C bus is enabled and accessible by the user that will run the nodes. Check `i2cdetect -y 1` to probe devices.

4) Camera
- Confirm the camera is available under `/dev/video0` (or set the `~camera_id` param accordingly). Test with `v4l2-ctl --list-devices` or a small OpenCV script.

5) Launching (example)
- Start `roscore` and any required MoveIt and AI nodes.
- Run the camera node (example):

```
rosrun communication_i2c camera_publisher.py _i2c_bus:=1 _i2c_address:=0x3E _camera_id:=0
```

- Note: the camera node in this repo starts the I2C interface itself.

6) AI node
- Our AI classifier should subscribe to `/dofbot/camera/image_raw` and publish the classification result on `/dofbot/waste_type` (or the topic configured via `~classification_topic`). It should also publish an image status (`ok` / `processing`) on `/dofbot/image_status`.

7) MoveIt / Pick & Place
- Start the MoveIt planning scene and ensure the MoveGroups are named `dofbot` and `grip_group`, or adapt the code to match your configuration.
