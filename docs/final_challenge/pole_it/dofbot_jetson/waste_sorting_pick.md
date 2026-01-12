# Waste Sorting Pick & Place System

### ROS / MoveIt / DOFBOT / Jetson

## 1. Overview

This project implements an autonomous robotic waste sorting system based on ROS.
It integrates perception outputs (waste type and pose) with a manipulation pipeline executing a fully automated Pick & Place sequence.

The system is designed for:

* Deterministic execution
* Modular architecture
* Fault tolerance
* ROS topic synchronization
* Industrial-style robotic workflow

---

## 2. System Architecture

```text
Perception Stack
   |
   |--> /dofbot/waste_type   (std_msgs/String)
   |
   |--> /dofbot/waste_pose   (geometry_msgs/PoseStamped)
                    |
                    v
             DechetListener (ROS Node)
                    |
                    v
                JetsonRun
                    |
                    v
                PickPlace
                    |
                    v
              DOFBOT Arm
```

---

## 3. Node: `dechet_listener`

### Purpose

Synchronizes perception outputs and triggers manipulation only when both:

* Waste type
* Waste pose

are available.

### Responsibilities

* Subscribe to perception topics
* Store latest valid messages
* Prevent partial execution
* Trigger Pick & Place pipeline
* Reset state after execution

---

### Subscribed Topics

| Topic                | Type                        | Description                 |
| -------------------- | --------------------------- | --------------------------- |
| `/dofbot/waste_type` | `std_msgs/String`           | Waste classification result |
| `/dofbot/waste_pose` | `geometry_msgs/PoseStamped` | Waste 3D pose               |

---

### Internal State

```python
self.last_type: Optional[str]
self.last_pose: Optional[PoseStamped]
```

---

### Synchronization Logic

```python
if self.last_type is not None and self.last_pose is not None:
    self.pnp.execute(...)
```

This guarantees:

* No execution with partial data
* No race conditions
* One execution per object

---

## 4. Class: `JetsonRun`

### Role

Acts as the manipulation orchestration layer.

It:

* Maps waste type to deposit pose
* Instantiates PickPlace pipeline
* Controls execution order
* Handles errors
* Ensures proper shutdown

---

## 5. Waste Type to Deposit Mapping

| Waste Type | Deposit Pose |
| ---------- | ------------ |
| dangereux  | Right zone   |
| menager    | Left zone    |
| recyclable | Back zone    |

Implementation:

```python
def find_place_position(self, type_dechet):
```

Each pose is defined as a 5-DOF joint configuration.

---

## 6. Pick & Place Execution Pipeline

### Execution Flow

1. Scene setup
2. Gripper open
3. Approach object
4. Grasp
5. Lift
6. Pre-place
7. Approach deposit
8. Release
9. Retreat
10. Cleanup

---

### Execution Control

Each step is validated:

```python
if not picker.approach_pick_object():
    return False
```

No unsafe continuation is allowed.

---

## 7. Error Handling Strategy

### Exceptions

```python
except Exception as e:
```

* Full traceback logging
* Immediate abort
* Safe shutdown

### Interruptions

```python
except KeyboardInterrupt:
```

Graceful stop.

### Finalization

```python
moveit_commander.roscpp_shutdown()
```

Ensures MoveIt resources are released.

---

## 8. Logging Policy

Every major action is logged:

* Initialization
* Step execution
* Failures
* Completion

This enables:

* Runtime debugging
* Post-mortem analysis
* System validation

---

## 9. Determinism and Safety

| Property                | Status |
| ----------------------- | ------ |
| Deterministic execution | Yes    |
| Partial data protection | Yes    |
| Collision-safe planning | MoveIt |
| Controlled gripper      | Yes    |
| Step validation         | Yes    |
| Graceful shutdown       | Yes    |

---

## 10. Extensibility

The system can be extended to:

* New waste categories
* Vision confidence filtering
* Dynamic deposit pose computation
* Multi-object queues
* ROS2 migration

---

## 11. Performance Considerations

* Message synchronization avoids incorrect actions
* Joint-space deposit poses minimize planning time
* Modular PickPlace class isolates motion logic
* Stateless listener ensures reliability

---

## 12. Code Responsibilities Summary

| Module         | Responsibility     |
| -------------- | ------------------ |
| Perception     | Detection + Pose   |
| DechetListener | Synchronization    |
| JetsonRun      | Orchestration      |
| PickPlace      | Motion control     |
| MoveIt         | Path planning      |
| DOFBOT         | Physical execution |

---

## 13. Failure Scenarios

| Scenario         | Behavior     |
| ---------------- | ------------ |
| Missing pose     | No execution |
| Missing type     | No execution |
| Motion failure   | Abort        |
| Grasp failure    | Abort        |
| Planning failure | Abort        |
| ROS interruption | Safe stop    |

---

## 14. Compliance with Robotics Software Practices

This system follows:

* Separation of concerns
* Single responsibility principle
* Defensive programming
* ROS communication standards
* MoveIt execution standards

---

## 15. Example Runtime Log

```text
[TYPE] Reçu : recyclable
[POSE] Reçu : frame_id=camera_link x=0.142 y=0.031 z=0.020
→ Les deux infos sont reçues, lancement du Pick & Place.
=== Début Pick & Place ===
✓ PICK AND PLACE TERMINÉ AVEC SUCCÈS
```

---

## 16. Limitations

* No temporal message synchronization filter (ApproximateTime)
* Deposit pose is static
* Single object pipeline
* No retry logic on failure

---

## 17. Future Improvements

* Message filters synchronization
* Pose confidence validation
* Adaptive grasp orientation
* Vision feedback loop
* ROS2 port
* Multi-object queue manager

---

## 18. Repository Structure Recommendation

```text
src/
 ├── dechet_listener.py
 ├── jetson_run.py
 ├── pick_place.py
 ├── perception/
 ├── config/
 └── launch/
```

---

## 19. Target Use Cases

* Academic research
* Robotics teaching
* Waste sorting prototype
* Industrial proof of concept

---

## 20. Conclusion

This system demonstrates a complete perception-to-manipulation robotic pipeline implemented with ROS and MoveIt, respecting professional robotics software engineering principles.