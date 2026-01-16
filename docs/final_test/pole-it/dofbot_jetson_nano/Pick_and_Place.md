# Pick & Place (MoveIt) — dechet_handler

We document the pick-and-place implementation found in `final_version/dechet_handler/src/pick_place.py`.

Main components
- MoveIt move groups we use in the code:
  - `dofbot` (arm MoveGroup)
  - `grip_group` (gripper MoveGroup)

Key steps in our pick/place cycle
1. Initialization: we initialize `moveit_commander` and create `RobotCommander`, `PlanningSceneInterface`, and `MoveGroupCommander` instances.
2. Scene setup: we add a small box representing the cube near the robot using `scene.add_box("cube", pose, size=(0.03,0.03,0.03))`.
3. Pre-grasp / approach: `pre_grasp()` or `approach_pick_object()` bring the arm into a safe pose.
4. Grasp: we call `close_gripper()` and `grasp_execution()` to attach the object to the end effector in the planning scene.
5. Lift / transport: `lift_object()` and `pre_place_execution()` move the object toward the deposit location.
6. Release: `release_object()` removes the attached object from the scene and opens the gripper.

Notes and defaults
- We use relaxed tolerances and faster scaling factors to prioritize speed over precision (see `set_max_velocity_scaling_factor` and `set_max_acceleration_scaling_factor`).
- There is a fallback to fixed joint-angle targets when the IK service is not available.

Useful outputs and debug
- `print_workspace_info()` prints active joint names, current pose, end effector link and planning frame to ROS logs.

## High-level pseudocode (Python)

```python
# Simplified pick-and-place flow (pseudocode)
def pick_and_place(pick_pose, place_pose):
  moveit.init()  # RobotCommander, PlanningSceneInterface, MoveGroup
  scene.add_box('object', pick_pose, size=(0.03,0.03,0.03))

  # approach and grasp
  if not plan_to(pick_pose.approach):
    plan_to(fallback_joint_pose)
  open_gripper()
  plan_to(pick_pose)
  close_gripper()
  scene.attach_box('object')

  # transport and release
  plan_to(place_pose.pre_place)
  plan_to(place_pose)
  open_gripper()
  scene.remove_attached_object('object')

```

This pseudocode captures the high-level sequence: initialize MoveIt, approach, grasp, attach the object, transport, and release — with a simple fallback to fixed joint poses when planning fails.
