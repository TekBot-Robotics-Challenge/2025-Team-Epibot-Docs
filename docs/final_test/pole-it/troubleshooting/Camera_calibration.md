# Camera calibration — checkerboard workflow and issues

We document the camera calibration workflow we ran (using a checkerboard) and the issues we encountered while calibrating the Dofbot camera. We ran the calibration interactively in JupyterLab and also tried manual positioning of the arm so the camera had a stable view of the checkerboard.

[!Calibration de la caméra](../../public/images/Final_Test_IT_Pole/camera_calibration.jpeg)

Summary
- Goal: obtain camera intrinsics (camera matrix and distortion coefficients) and a robust camera pose for the vision pipeline.
- Method: place a printed checkerboard in front of the camera, move the arm to an optimal calibration pose, capture multiple images, and run the calibration notebook.
- Outcome: we found precomputed calibration parameters on the VM (`/home/calibrationdata`) and used those values for the final runs. The notebook we used is retained in our notes and the key steps are summarized below.

Why we did this manually
- The Jetson camera sometimes needs careful exposure/positioning to see the whole checkerboard. We therefore positioned the arm to a repeatable pose before capturing images so that the checkerboard filled roughly 60–80% of the frame and was in focus.

Notebook workflow (what we executed)
1. Move the arm to the calibration pose — our target servo angles for calibration were: (90, 90, 5, 5, 89, 179). We used a small Jupyter notebook that:
   - connected to the Arm via `Arm_Lib`
   - defined a `move_arm()` helper to move to the calibration pose and then return to a safe pose
   - set servo 6 explicitly during tests to find the best camera tilt
2. Capture test images from the camera (OpenCV `cv2.VideoCapture`) and verify each image shows the full checkerboard.
3. If images were black / no camera available, re-seat the CSI/USB cable and reboot the Jetson. We also checked camera id (0 vs 1) used by `cv2.VideoCapture`.
4. Run the calibration script that detects checkerboard corners across many images and computes intrinsics using OpenCV's `calibrateCamera()`.
5. Save the camera matrix and distortion coefficients to YAML / NumPy files for later use by the vision node.

Common problems we encountered and how we fixed them
- Camera cannot be opened (OpenCV: cap.isOpened() == False)
  - Fixes: verify the correct camera id (0 vs 1), re-seat the CSI ribbon, confirm camera works with a standalone OpenCV script, and reboot the Jetson if necessary.

- Matplotlib temporary config warning in Jupyter
  - Observed warning: Matplotlib created a temporary config/cache directory because the default path was not writable. Fix: set the `MPLCONFIGDIR` environment variable to a writable directory in the notebook or create the expected `.config/matplotlib` directory for the `jetson` user.

- Checkerboard not detected / poor corner detection
  - Fixes: ensure checkerboard fills ~60–80% of the image, use even illumination (avoid strong shadows), increase camera exposure or autofocus, and re-take images with better framing.

- Checkerboard too small or too close to edges
  - Fixes: move the checkerboard so it's centered, or move the arm/tilt the camera for a better angle. Use the `move_arm()` helper to make fine adjustments and re-capture.

- Calibration results inconsistent / large reprojection error
  - Fixes: discard problematic frames (blurry, partially visible), add more images from slightly different viewpoints, and ensure the checkerboard is flat.

How we verified calibration results
- Calculate reprojection error after `calibrateCamera()`; we kept runs with mean reprojection error < 0.5 px when possible.
- Visually reproject detected corners onto the original images to ensure good alignment.

Final decision — using VM calibration data
- While experimenting we discovered working calibration parameter files already present on the VM at `/home/calibrationdata` (intrinsics + distortion). To save time and ensure consistent results across runs we used those vendor-provided calibration parameters for the final experiments and integrated them into the vision node configuration.


Practical commands / snippets
- Set a writable Matplotlib config dir in the notebook before importing pyplot:

```
import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib_config'
import matplotlib.pyplot as plt
```

- Quick camera check outside ROS:

```
python3 -c "import cv2; cap=cv2.VideoCapture(0); print(cap.isOpened()); ret, f = cap.read(); print(ret); cap.release()"
```

Recommendations
- Keep a small set (20–40) of well framed checkerboard images saved with incremental filenames (calib_01.jpg ...). Re-run calibration until reprojection error is stable.
- Store the final YAML or NumPy parameter files in the repo so they can be reused by others and by CI tests.

## High-level pseudocode (Python)

```python
# Move arm, capture images, and calibrate (pseudocode)
def move_arm_to_calibration_pose():
  arm.connect()
  arm.write_servos([90, 90, 5, 5, 89, 179])
  wait_until_stable()

def capture_calibration_images(n=30):
  cap = cv2.VideoCapture(camera_id)
  images = []
  for i in range(n):
    ret, img = cap.read()
    if not ret:
      raise RuntimeError('camera capture failed')
    save(f'calib_{i:02d}.jpg', img)
    images.append(img)
    sleep(0.5)
  cap.release()
  return images

def calibrate(images):
  corners = detect_checkerboard_corners(images)
  camera_matrix, dist_coefs = cv2.calibrateCamera(corners)
  save_params('camera_params.npz', camera_matrix, dist_coefs)

def run_calibration():
  move_arm_to_calibration_pose()
  imgs = capture_calibration_images(30)
  calibrate(imgs)

```

This pseudocode outlines the manual calibration flow we used: position the arm, capture a set of well-framed images, and run OpenCV calibration to compute and save intrinsics.
