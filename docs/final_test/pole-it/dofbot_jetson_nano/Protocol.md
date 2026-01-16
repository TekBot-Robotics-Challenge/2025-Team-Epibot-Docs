## High-level pseudocode (Python)

```python
   i2c.write('DOFBOT CONNECTED')
   resp = i2c.read(timeout=1.0)
   return resp == 'CONVEYOR CONNECTED'
def main_loop(i2c):
   if not handshake(i2c):
   reconnect()
   while True:
   i2c.write('ANY WASTE DETECTED?')
   reply = i2c.read()
   if reply == 'OK':
            ros_publish('/dofbot/camera/trigger', 'capture')
            label = ros_wait_for('/dofbot/waste_type')
            i2c.write(f'GARBAGE:{map_label(label)}')
   elif reply == 'CONVEYOR DISCONNECTED':
            handle_disconnect()
   sleep(2.0)
```

This sketch shows the handshake and the polling-style main loop used by the system.
# Waste Sorting Protocol

Actors
- Conveyor — Raspberry Pi that handles detection and responds over I2C
- Dofbot — Jetson Nano running the camera/I2C bridge and ROS nodes
- Virtual Machine / PC — optional AI classification and MoveIt planning nodes

**Important implementation notes**
- In our implementation the Dofbot initiates the low-level I2C handshake: it writes `DOFBOT CONNECTED` and expects `CONVEYOR CONNECTED` back.
- The conveyor does not push a continuous "GARBAGE_DETECTED" message; instead the Dofbot polls the conveyor with `ANY WASTE DETECTED?` and the conveyor replies `OK` (waste detected) or `KO` (no waste).
- On `OK` we publish a single image to the camera topic; the AI returns a classification string on the configured topic and the bridge sends `GARBAGE:<id>` back over I2C.
- The current waste ID mapping used by `communication_i2c/camera_publisher.py` is a compact 4-value set (1..4).

---

## PHASE 1: HANDSHAKE (Connection Establishment)

**Sequence**
1. Dofbot writes `DOFBOT CONNECTED` by I2C to the conveyor (we initiate the handshake).
2. Conveyor replies `CONVEYOR CONNECTED` by I2C.
3. We (Dofbot) publish the conveyor status on a ROS topic (bridge implementations vary); VM/AI subscribe to that topic to become aware of the conveyor.
4. VM/AI publish `DOFBOT CONNECTED` back on a ROS topic for bookkeeping; the Dofbot then starts its polling loop.

Sequence diagram (I2C handshake + optional VM bridge)
```
DOFBOT                    CONVEYOR                   VIRTUAL MACHINE
   |                          |                              |
   |─────[I2C]───────────────>|                              |
   | "DOFBOT CONNECTED"      |                              |
   |                          |                              |
   |<────[I2C]────────────────|                              |
   | "CONVEYOR CONNECTED"    |                              |
   |                          |                              |
   |────[ROS Publish]*------->|                              |
   | /conveyor_connection     |                              |
   |                          |────[ROS Publish]-------------->|
   |                          | /dofbot_connection            |
   |                          |                              |
   v                          v                              v
 READY                      READY                          READY

```

---

## PHASE 2: CONTINUOUS SORTING LOOP (our polling model)

We adapted the original continuous loop to match the polling model implemented in `I2CConveyorInterface`.

1. Dofbot periodically writes `ANY WASTE DETECTED?` to the conveyor via I2C.
2. Conveyor replies with one of:
   - `OK` — a cube is present and ready for capture
   - `KO` — no cube present
3. On `OK`, we trigger a single image capture and publish the image on `/dofbot/camera/image_raw` (sensor_msgs/Image). The camera node publishes one initial image on startup to warm the AI and allow an initial calibration step.
4. The AI node subscribes to `/dofbot/camera/image_raw`, runs the classifier, then publishes:
   - `/waste_pos` — estimated object pose/position
   - `/dofbot/waste_type` (std_msgs/String) — classification label (string)
   - `/dofbot/image_status` (std_msgs/String) — status such as `processing` or `ok` to indicate readiness for the next capture
5. The camera/I2C bridge maps the classification string to an integer ID and calls `I2CConveyorInterface.send_waste_type(waste_id)`, which writes `GARBAGE:<id>` to the conveyor.
6. The conveyor acknowledges the classification and will place the next cube after sorting. If the conveyor reports `CONVOYEUR DISCONNECTED` or another error, the Dofbot attempts reconnection.

Complete sequence diagram
```
CONVEYOR                    DOFBOT                      VM (Node1)              VM (Node2)
    |                          |                              |                      |
    |<────[I2C Poll]────────── |                              |                      |
    |  (Dofbot asks: ANY WASTE?)                              |                      |
    |                          |                              |                      |
    |────[I2C reply: OK/KO] -->|                              |                      |
    |      (OK)                |                              |                      |
    |                          |─────[ROS Publish]───────────>|                      |
    |                          |  /dofbot/camera/image_raw    |                      |
    |                          |  (Image 640x480 BGR8)        |                      |
    |                          |                              |                      |
    |                          |                              |──[AI Process]──>     |
    |                          |                              |  (PyTorch / TF model) |
    |                          |                              |                      |
    |                          |                              |─────[Publish]───────>|
    |                          |                              |  /waste_pos          |
    |                          |                              |  /dofbot/waste_type  |
    |                          |                              |                      |
    |                          |                              |                      |──[MoveIt Plan]──>
    |                          |                              |                      |
    |                          |<────────[bridge maps label]──|                      |
    |                          |  (send GARBAGE:<id> via I2C) |                      |
    |                          |───[I2C] GARBAGE:<id> ------->|                      |
    |<────[I2C ack]────────────|                              |                      |
    |                          |                              |                      |
    |──[Place new cube]──────>|                              |                      |
    |                          |                              |                      |
    └──────────────────────────┴──────────────────────────────┴──────────────────────┘
         (LOOP BACK TO TOP)
```

---

## Message table

| Direction | Message / Topic | Meaning |
|-----------|-----------------|---------|
| I2C (Dofbot→Conveyor) | `DOFBOT CONNECTED` | Handshake — Dofbot announces itself |
| I2C (Conveyor→Dofbot) | `CONVEYOR CONNECTED` | Handshake — conveyor confirms |
| I2C (Dofbot→Conveyor) | `ANY WASTE DETECTED?` | Poll question, sent periodically by Dofbot |
| I2C (Conveyor→Dofbot) | `OK` / `KO` | `OK` → object present; `KO` → none |
| ROS (Dofbot→VM) | `/dofbot/camera/image_raw` | sensor_msgs/Image — single image published on trigger |
| ROS (VM→Dofbot) | `/dofbot/waste_type` | std_msgs/String — classification label |
| ROS (VM→Dofbot) | `/dofbot/image_status` | std_msgs/String — `processing` / `ok` to control capture rate |
| I2C (Dofbot→Conveyor) | `GARBAGE:<id>` | Classification result (id mapping below) |

### Waste ID mapping
- 1 — `dangereux` (hazardous)
- 2 — `menager` (household)
- 3 — `recyclable` (recyclable)
- 4 — `no_waste` (unknown / fallback)

---

## Failure modes and recovery
- The `I2CConveyorInterface` wraps I2C operations with a lock and counts consecutive failures. After a threshold it marks the bus disconnected and attempts reconnection.
- If the AI or MoveIt reports errors, we rely on ROS logging and the `image_status` topic to avoid triggering new captures while processing is ongoing.

---