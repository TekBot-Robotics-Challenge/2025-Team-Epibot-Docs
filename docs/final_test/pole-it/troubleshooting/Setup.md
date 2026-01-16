# Troubleshooting: system setup issues and fixes

This page lists practical setup-related problems we encountered and how we solved them. Use it as a short reference when things go wrong during installation or first runs.

I2C-related
- Symptom: `I2C` read/write errors or no response from the conveyor.
- Fixes we applied:
	- Confirmed the correct I2C bus (`i2c_bus=1`) and address (`0x3E`) in ROS params.
	- Verified physical wiring and SDA/SCL orientation; used `i2cdetect -y 1` to check presence.
	- Added the running user to the `i2c` group or used `udev` rules to allow access without sudo.

Camera-related
- Symptom: black frames or camera not detected.
- Fixes:
	- Re-seated the CSI ribbon and rebooted the Jetson.
	- Tested the camera with a small OpenCV script to ensure it's working outside ROS.
	- Tweaked camera exposure/white balance in the vision node parameters.

Python / ROS dependencies
- Symptom: missing modules (ImportError) when running nodes.
- Fixes:
	- Installed `opencv-python`, `smbus2` via `pip3` inside the ROS Python environment.
	- Source ROS environment (`source /opt/ros/<distro>/setup.bash`) before running `rosrun`/`roslaunch`.

MoveIt / planning
- Symptom: planning fails, IK service unavailable.
- Fixes:
	- Ensure `move_group` is running and that the robot description and controllers are loaded.
	- As a fallback we used fixed joint targets in `pick_place.py` and increased tolerances to make early tests more robust.

Permissions and services
- Symptom: nodes fail silently or log permission denied.
- Fixes:
	- Check user groups, add the user to `dialout`/`i2c` where appropriate.
	- Use `rosservice list` and `rostopic list` to check services and topics are up before starting dependent nodes.
