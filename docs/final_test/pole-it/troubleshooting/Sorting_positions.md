# Sorting positions and joint-state workflow

This page documents how we determined joint positions for sorting locations, how we read real servo positions, and how we publish the joint values to the ROS `joint_states` topic for MoveIt and debugging.

Reading real servo positions
- We used `Final_Test/final_version/touble_shooting/read_servo_positions.py` which reads (or simulates) servo angles and converts them into joint radians. Key points:
  - The script maps raw servo angles (degrees) into joint radians using the formula `joint_angle = (servo_angle - 90) * (pi/180)` for most servos.
  - Servo 6 uses a custom mapping in the script: `joint_angle = (servo_angle - 180) / 116.0` (this is hardware-specific; keep it as-is unless you calibrate differently).
  - The script was used to publish `/joint_states` so MoveIt sees the current physical configuration.

Finding deposit / sorting poses
- Steps we followed:
  1. Manually move the arm (via app or small scripts) to a safe pre-grasp pose and record servo angles using `read_servo_positions.py` output.
  2. Lower the arm to pick and record servo angles for pre-grasp and grasp positions.
  3. For each sorting bin (hazardous / household / recyclable) we adjusted joint values until the trajectory placed the cube reliably into the bin.

Publishing joint states
- We publish a `sensor_msgs/JointState` message on `/joint_states` containing the 6 joint names and the converted radians. MoveIt and other ROS nodes can subscribe to this topic to plan and visualize.

Converting servo angles to MoveIt joints (practical tips)
- Always test the conversion on a safe pose before trying a full grasp.
- When MoveIt reports collisions or unreachable poses, slightly relax joint tolerances or move to a safer pre-grasp and reattempt.

Example: sample positions we recorded (angles are stored in the repo logs)
- HOME / neutral: [90, 39, 16, 79, 90, 90] (servo degrees) — see `read_servo_positions.py` for default test values
- PRE-GRASP: change servo 2 and 3 slightly downward to reach the cube
- PRE-PLACE (bin 1 / 2 / 3): tuned by trial and error and then saved as joint arrays in `dechet_handler` scripts.

Notes and caveats
- The direct mapping from servo angles to radians is dependent on mechanical offsets; if you rebuild the arm, re-run the calibration and update the conversion constants.
- We recommend storing final verified joint arrays as named poses either inside MoveIt (SRDF) or as Python constants in `dechet_handler`.
