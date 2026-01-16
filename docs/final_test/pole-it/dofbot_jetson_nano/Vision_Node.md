# Vision node & camera publisher

We summarize the camera/vision related implementation found in `final_version/communication_i2c/camera_publisher.py` and `dofbot_ai_vision/publisher.py`.

Node names and topics (observed in code)
- Node: `camera_publisher` (ROS node name)
- Camera image topic: default `/dofbot/camera/image_raw` (sensor_msgs/Image)
- Classification topic: `/dofbot/waste_type` (std_msgs/String) — the current `communication_i2c/camera_publisher.py` uses this default; other camera implementations in the repo may use `/dofbot/waste_classification` so keep topic names consistent across your launch files.
- Image status topic: `/dofbot/image_status` (std_msgs/String) — used by IA to report `ok`, `processing`, etc.

Trigger behaviour
- We use a trigger model: the camera node publishes a single image when the I2C interface reports `OK`.
- On startup the node publishes one initial image automatically so the AI can warm up.

Parameters (ROS params used)
- `~camera_id` (default 0) — camera device
- `~frame_rate` (default 30)
- `~image_width` and `~image_height` (defaults: 640x480)
- `~camera_topic`, `~classification_topic`, `~image_status` — topic names are configurable via params

Waste classification mapping
The camera bridge maps classification strings to integer IDs before sending `GARBAGE:<id>` to the conveyor. The current `communication_i2c/camera_publisher.py` mapping is:

- `dangereux` → 1
- `menager` → 2
- `recyclable` → 3
- `no_waste` → 4 (default/fallback)

## High-level pseudocode (Python)

```python
# Camera-triggered capture and classification (pseudocode)
def on_i2c_trigger():
	image = capture_frame(camera_id)
	ros_publish('/dofbot/camera/image_raw', image)
	ros_publish('/dofbot/image_status', 'processing')
	label = run_classifier(image)  # returns e.g. 'recyclable'
	ros_publish('/dofbot/waste_type', label)
	ros_publish('/dofbot/image_status', 'ok')

def startup():
	# publish a warm-up image on start
	image = capture_frame(camera_id)
	ros_publish('/dofbot/camera/image_raw', image)

```

A concise summary of the camera node behaviour: it captures on trigger, publishes the image and status, runs a classifier, and publishes the label.
