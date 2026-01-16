## Dofbot — Jetson Nano waste sorting (project overview)

In this document set we describe the Dofbot waste sorting system implemented on a Jetson Nano. We sort small cubic objects representing different waste categories (household, recyclable, hazardous) that travel on a conveyor. The conveyor detects object presence and communicates via I2C with the Dofbot. We capture a single image with the onboard camera, send it to an AI node for classification (either on the Jetson or on a remote VM), and perform a pick-and-place based on the classification.

In this folder we include the following pages:

- `Protocol.md` — communication sequences and message formats (I2C and ROS topics)
- `Communication_I2C.md` — low-level I2C interface details and example messages
- `Vision_Node.md` — camera publisher and topics, parameters and waste type mapping
- `Pick_and_Place.md` — MoveIt-based pick & place flow and important functions
- `Setup.md` — quick setup notes and required packages for Jetson/ROS

Primary code reference: `Final_Test/final_version/` (communication_i2c, dofbot_ai_vision, dechet_handler)

---

Next: read `Protocol.md` for a high-level sequence diagram that matches the implementation.
