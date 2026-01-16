# Communication (I2C) — details and defaults

Defaults
- I2C bus: param `~i2c_bus`, default = 1
- I2C address: param `~i2c_address`, default = 0x3E
- Poll rate: param `~i2c_poll_rate`, default = 2.0 seconds

Principal behaviours
- We implement the Dofbot as an I2C master: it periodically writes `ANY WASTE DETECTED?` and reads the conveyor response.
- If the conveyor replies `OK`, we trigger a single image capture (camera node callback).
- We send classification results as text messages `GARBAGE:<id>` where id is an integer code.

Message formats used by the implementation
- `DOFBOT CONNECTED` — Dofbot announces itself during connection.
- `CONVEYOR CONNECTED` — conveyor announces itself during connection.
- `ANY WASTE DETECTED?` — poll question from Dofbot to conveyor.
- `OK` — conveyor: waste detected, trigger capture.
- `KO` — conveyor: no waste present.
- `GARBAGE:<id>` — Dofbot → conveyor: classification result.
- `CONVOYEUR DISCONNECTED` / `CONVEYOR DISCONNECTED` — explicit disconnection notice.

Waste ID mapping
- The exact integer codes are supplied by the camera/bridge node before sending `GARBAGE:<id>` to the conveyor. The current `communication_i2c/camera_publisher.py` implementation (latest version) maps classification strings to a small 4-entry set:
	- 1 — `dangereux` (hazardous)
	- 2 — `menager` (household)
	- 3 — `recyclable` (recyclable)
	- 4 — `no_waste` (no waste / unknown / fallback)

Notes
- The interface uses `smbus2` and wraps read/write in a lock to prevent concurrent access.
- The code will attempt to reconnect automatically when repeated failures occur.

## High-level pseudocode (Python)

```python
# Simplified I2C polling loop (pseudocode)
def main():
	i2c = I2C(bus=1, address=0x3E)
	i2c.write("DOFBOT CONNECTED")
	while True:
		i2c.write("ANY WASTE DETECTED?")
		reply = i2c.read()
		if reply == "OK":
			# trigger camera node to capture and publish one image
			ros_publish('/dofbot/camera/trigger', 'capture')
			# wait for classifier to publish a label on /dofbot/waste_type
			label = ros_wait_for('/dofbot/waste_type')
			waste_id = map_label(label)  # map string -> int
			i2c.write(f"GARBAGE:{waste_id}")
		sleep(POLL_INTERVAL)

```

This pseudocode summarizes the master polling loop and the classification relay to the conveyor.
