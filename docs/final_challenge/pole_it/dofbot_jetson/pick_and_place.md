# PickPlace Class

## Overview

The `PickPlace` class is the core manipulation module of the waste sorting system. It provides a complete Pick & Place pipeline with MoveIt integration, inverse kinematics computation, and robust error handling.

---

## Table of Contents

1. [Class Initialization](#class-initialization)
2. [Motion Planning Methods](#motion-planning-methods)
3. [Gripper Control](#gripper-control)
4. [Pick & Place Pipeline](#pick--place-pipeline)
5. [Inverse Kinematics](#inverse-kinematics)
6. [Scene Management](#scene-management)
7. [Error Handling Strategy](#error-handling-strategy)
8. [Configuration Parameters](#configuration-parameters)

---

## Class Initialization

### Constructor

```python
def __init__(self, dechet_pose, depot_pose)
```

**Parameters:**
- `dechet_pose` (geometry_msgs/PoseStamped): 3D pose of the waste object to pick
- `depot_pose` (list): Joint configuration for deposit position [j1, j2, j3, j4, j5]

**Initialization Sequence:**

1. Initialize MoveIt Commander
2. Create robot, scene, and planning group interfaces
3. Configure motion parameters (velocity, acceleration, tolerances)
4. Initialize IK service proxy
5. Display workspace information
6. Wait for system stabilization (1 second)

**Motion Parameters:**

| Parameter                     | Value | Purpose                      |
| ----------------------------- | ----- | ---------------------------- |
| Max velocity scaling          | 0.3   | Safety and stability         |
| Max acceleration scaling      | 0.3   | Smooth motion                |
| Position tolerance            | 0.01  | 1cm precision                |
| Orientation tolerance         | 0.1   | Relaxed orientation          |
| Planning time                 | 15.0s | Complex path computation     |
| Planning attempts             | 20    | Robust planning              |

---

## Motion Planning Methods

### 1. Joint Space Motion

```python
def go_to_joint_position(self, joint_values, description="Mouvement")
```

**Purpose:** Execute motion using direct joint angle control (most reliable)

**Parameters:**
- `joint_values` (list): 5 joint angles in radians [j1, j2, j3, j4, j5]
- `description` (str): Motion description for logging

**Returns:** `bool` - Success status

**Use Cases:**
- Home position
- Pre-defined poses (pre-grasp, pre-place)
- Fallback when Cartesian planning fails

**Example:**
```python
home_position = [0.0, 0.0, 0.0, 0.0, 0.0]
success = picker.go_to_joint_position(home_position, "Return HOME")
```

---

### 2. Cartesian Space Motion with IK

```python
def go_to_pose_with_ik(self, target_pose, description="Mouvement")
```

**Purpose:** Move to Cartesian pose using inverse kinematics

**Strategy:**
1. Attempt IK computation via service
2. If IK succeeds, execute joint motion
3. If IK fails, fallback to `set_pose_target` planning

**Parameters:**
- `target_pose` (geometry_msgs/Pose): Target end-effector pose
- `description` (str): Motion description

**Returns:** `bool` - Success status

**Fallback Chain:**
```
IK Service → Joint Execution
     ↓ (failure)
set_pose_target → MoveIt Planning → Execution
```

---

### 3. Cartesian Path Planning

```python
def approach_pick_object(self)
def lift_object()
def approach_place_object()
def retreat()
```

**Purpose:** Execute linear motions using Cartesian path computation

**Configuration:**
- Step resolution: 1cm (`eef_step=0.01`)
- Jump threshold: `True` (avoid discontinuities)
- Success threshold: 90% path completion

**Fallback Strategy:**

```
Cartesian Path (>90% fraction)
     ↓ (failure)
IK-based direct motion
     ↓ (failure)
Joint-space incremental motion
```

**Example - Approach Object:**
```python
waypoints = []
wpose = self.arm_group.get_current_pose().pose
waypoints.append(copy.deepcopy(wpose))

wpose.position.z -= 0.10  # Descend 10cm
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = self.arm_group.compute_cartesian_path(
    waypoints, 0.01, True
)
```

---

## Inverse Kinematics

### IK Service Interface

```python
def compute_ik_for_pose(self, target_pose)
```

**Purpose:** Compute joint angles for a Cartesian pose

**Service:** `/compute_ik` (moveit_msgs/GetPositionIK)

**Request Structure:**
```python
ik_request.group_name = "dofbot"
ik_request.robot_state = current_state
ik_request.pose_stamped = target_pose_stamped
ik_request.timeout = rospy.Duration(5.0)
```

**Returns:**
- `list`: Joint angles [j1, j2, j3, j4, j5] on success
- `None`: If IK solution not found

**Error Codes:**

| Code | Meaning                           |
| ---- | --------------------------------- |
| 1    | SUCCESS                           |
| -31  | NO_IK_SOLUTION (unreachable pose) |
| -1   | FAILURE (general error)           |
| -12  | INVALID_ROBOT_STATE               |
| -22  | TIMED_OUT                         |

**Example:**
```python
joint_values = self.compute_ik_for_pose(target_pose)
if joint_values is not None:
    self.arm_group.go(joint_values, wait=True)
```

---

## Gripper Control

### Open Gripper

```python
def full_open(self)
def open_gripper(self)
```

**Joint Value:** `-1.0` (fully open)

**Execution:**
```python
active_joints = self.gripper_group.get_active_joints()
joint_target = dict(zip(active_joints, [-1.0]))
self.gripper_group.set_joint_value_target(joint_target)
self.gripper_group.go(wait=True)
```

**Use Cases:**
- Before picking
- After releasing object

---

### Close Gripper

```python
def close_gripper(self)
```

**Joint Value:** `-0.41` (grasp position)

**Use Cases:**
- Grasping object
- Holding during transport

**Stabilization:** 200ms delay after closing (`rospy.sleep(0.2)`)

---

## Pick & Place Pipeline

### Complete Execution Sequence

```python
# Setup
scene_setup()           # Add object to planning scene
full_open()             # Open gripper

# Pick Phase
go_home()               # Start from known position
approach_pick_object()  # Descend to object
grasp_execution()       # Close gripper + attach object
lift_object()           # Lift object

# Place Phase
pre_place_execution()   # Move to deposit zone
approach_place_object() # Descend to deposit
release_object()        # Open gripper + detach object
retreat()               # Lift after release

# Cleanup
cleanup()               # Remove object, shutdown
```

---

### Individual Pipeline Methods

#### 1. Scene Setup

```python
def scene_setup(self)
```

**Purpose:** Add object to MoveIt planning scene

**Object Parameters:**
- Name: `"cube"`
- Size: 3cm × 3cm × 3cm
- Frame: Inherited from `dechet_pose.header.frame_id`

**Verification:**
```python
known_objects = self.scene.get_known_object_names()
# Verify "cube" in known_objects
```

---

#### 2. Grasp Execution

```python
def grasp_execution(self)
```

**Actions:**
1. Close gripper
2. Attach object to end-effector link
3. Verify attachment

**Attachment Syntax:**
```python
eef_link = self.arm_group.get_end_effector_link()
touch_links = self.robot.get_link_names(group='grip_group')
all_touch_links = list(set(touch_links + [eef_link]))

self.scene.attach_box(
    eef_link,      # link
    'cube',        # object name
    touch_links=all_touch_links
)
```

**Verification:**
```python
attached_objects = self.scene.get_attached_objects(['cube'])
return len(attached_objects) > 0
```

---

#### 3. Pre-Place Execution

```python
def pre_place_execution(self)
```

**Strategy:** Use fixed joint configuration (depot_pose)

**Rationale:**
- Joint space motion is most reliable
- Avoids IK failures in workspace boundaries
- Depot positions are pre-calibrated

**Implementation:**
```python
return self.go_to_joint_position(
    self.depot_pose, 
    "PRE-PLACE (angles fixes)"
)
```

---

#### 4. Release Object

```python
def release_object(self)
```

**Actions:**
1. Detach object from end-effector
2. Wait for scene update (1 second)
3. Open gripper

**Detachment Syntax:**
```python
eef_link = self.arm_group.get_end_effector_link()
self.scene.remove_attached_object(eef_link, 'cube')
rospy.sleep(1)  # Critical for scene synchronization
```

---

#### 5. Cleanup

```python
def cleanup(self)
```

**Actions:**
1. Return to home position
2. Remove object from scene
3. Shutdown MoveIt Commander

**Critical:** Must be called to release resources

---

## Scene Management

### Planning Scene Interface

**Available Methods:**
- `add_box(name, pose, size)`: Add collision object
- `attach_box(link, name, touch_links)`: Attach object to robot
- `remove_attached_object(link, name)`: Detach object
- `remove_world_object(name)`: Remove object from scene
- `get_known_object_names()`: List objects in scene
- `get_attached_objects(names)`: Get attached objects

**Frame of Reference:**
- All poses must be in `"base_link"` frame
- Objects added with `PoseStamped` with correct `header.frame_id`

---

## Error Handling Strategy

### Multi-Level Fallback System

Each motion method implements a fallback chain:

#### Example: Approach Pick Object

```
Level 1: Cartesian Path (>90% completion)
   ↓ failure
Level 2: IK-based direct motion
   ↓ failure
Level 3: Joint-space incremental motion
   ↓ failure
Return False (abort)
```

### Logging Levels

| Symbol | Meaning | Method          |
| ------ | ------- | --------------- |
| ✓      | Success | `rospy.loginfo` |
| ⚠      | Warning | `rospy.logwarn` |
| ✗      | Error   | `rospy.logerr`  |

### Return Values

All motion methods return `bool`:
- `True`: Motion successful
- `False`: Motion failed (caller should abort)

---

## Configuration Parameters

### MoveIt Planning Groups

| Group       | Purpose            | DOF |
| ----------- | ------------------ | --- |
| `dofbot`    | 5-axis arm         | 5   |
| `grip_group`| Gripper (parallel) | 1   |

### Joint Limits (Radians)

Defined in URDF/SRDF configuration files.

### Workspace Boundaries

**Recommended Safe Zone:**
- X: 0.10 to 0.25 m (forward)
- Y: -0.15 to 0.15 m (lateral)
- Z: 0.05 to 0.20 m (height above base)

**Critical:** Poses outside this zone may cause IK failures

---

## Debugging and Diagnostics

### Workspace Information

```python
def print_workspace_info(self)
```

**Displays:**
- Planning group name
- End-effector link
- Planning frame
- Active joints
- Current end-effector position

**Usage:** Automatically called during initialization

---

### IK Debugging

When IK fails, check:
1. Is pose within workspace boundaries?
2. Is orientation feasible for the arm configuration?
3. Are joint limits respected?
4. Is the IK service running? (`rosservice list | grep compute_ik`)

**Common Fixes:**
- Relax orientation constraints
- Move target closer to robot base
- Adjust Z-height to avoid singularities

---

## Dependencies

### ROS Packages

```python
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
```

### System Requirements

- ROS Melodic/Noetic
- MoveIt 1.x
- DOFBOT URDF/SRDF configuration
- IK service (`compute_ik`) running

---

## Performance Characteristics

### Typical Execution Times

| Phase            | Duration |
| ---------------- | -------- |
| Scene setup      | 1-2s     |
| Joint motion     | 3-5s     |
| Cartesian motion | 4-7s     |
| IK computation   | 0.1-0.5s |
| Grasp/release    | 0.5-1s   |
| **Total cycle**  | **15-25s** |

### Optimization Considerations

- Joint-space motion is faster than Cartesian
- Pre-computed depot poses avoid IK overhead
- Cartesian paths with high fraction (>95%) are fastest
- Planning time can be reduced for simple motions

---

## Best Practices

### Motion Planning

1. **Always use joint space for known positions**
   - Home, pre-grasp, deposit positions
   
2. **Use Cartesian paths for linear motions**
   - Approach, lift, retreat
   
3. **Use IK for dynamic target poses**
   - Vision-based object picking

### Error Recovery

1. **Check return values**
   ```python
   if not picker.approach_pick_object():
       return False  # Abort safely
   ```

2. **Log all failures**
   - Enables post-mortem analysis
   
3. **Clean up on exit**
   ```python
   try:
       picker.execute_pipeline()
   finally:
       picker.cleanup()
   ```

### Scene Synchronization

1. **Always wait after scene modifications**
   ```python
   self.scene.add_box(...)
   rospy.sleep(1)  # Critical
   ```

2. **Verify attachments**
   ```python
   attached = self.scene.get_attached_objects(['cube'])
   if not attached:
       rospy.logerr("Attachment failed")
   ```

---

## Known Limitations

1. **No retry logic**: Single attempt per motion
2. **Static deposit poses**: No dynamic placement
3. **No collision recovery**: Abort on planning failure
4. **Single object only**: No multi-object queue
5. **No force feedback**: Open-loop grasping

---

## Future Enhancements

### Planned Improvements

- [ ] Adaptive grasp force control
- [ ] Multi-attempt retry with pose adjustment
- [ ] Dynamic deposit pose computation
- [ ] Collision recovery strategies
- [ ] Vision-in-the-loop grasping
- [ ] ROS2 migration with action servers

---

## API Summary

### Essential Methods

```python
# Initialization
picker = PickPlace(object_pose, deposit_joints)

# Setup
picker.scene_setup()
picker.full_open()

# Motion
picker.go_to_joint_position(joints)
picker.go_to_pose_with_ik(pose)

# Gripper
picker.open_gripper()
picker.close_gripper()

# Pipeline
picker.grasp_execution()
picker.lift_object()
picker.release_object()

# Cleanup
picker.cleanup()
```

---

## Conclusion

The `PickPlace` class provides a robust, industrial-grade manipulation interface with:

✓ Multi-level error handling  
✓ IK computation and fallback  
✓ Scene synchronization  
✓ Comprehensive logging  
✓ Modular pipeline architecture  

Suitable for research, teaching, and prototype industrial applications.