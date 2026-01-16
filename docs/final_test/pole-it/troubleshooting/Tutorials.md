# Tutorials we used and how we applied them

We list the tutorials and guides we followed and briefly explain how we applied each to our workflow.

1) JupyterLab
- Why: we used JupyterLab to run interactive code on both the VM and the Jetson. It sped up experimentation and debugging.
- How we used it: launched kernels on the VM, mounted the `Final_Test` folder, and iteratively tested camera capture and AI inference steps.

2) MoveIt tutorials
- Why: MoveIt taught us how to configure a planning scene, create MoveGroups, and run simple motion planning.
- How we used it: adapted the tutorials to the `dofbot` move group name and used the `pick_place.py` code as a bridge between MoveIt planning and the physical robot.

3) Multi-machine ROS connection
- Why: our AI and planning nodes run on a VM while the Jetson runs the hardware interface; we needed a stable ROS multi-machine setup.
- How we used it: configured ROS_MASTER_URI and ROS_IP on each machine, used bridged networking in the VM, and tested with `rostopic echo` between hosts.

4) SSH connection and remote editing
- Why: to edit files and control the Jetson remotely.
- How we used it: installed and configured `ssh`, set up key-based auth, and installed VSCode on the VM for a comfortable development environment.

5) System updates and package installation
- Why: several issues were caused by missing or outdated packages.
- How we used it: followed the vendor steps to update packages, installed `smbus2`, `opencv-python`, and ROS dependencies via `rosdep` where possible.

6) Arm control basics
- Why: to get the robot moving and to discover joint limits and safe positions.
- How we used it: tested gripper open/close, ran homing sequences, and recorded joint angles in `read_servo_positions.py` to use as references for pick/place poses.
