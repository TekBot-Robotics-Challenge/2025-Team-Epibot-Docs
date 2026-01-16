
# Initial setup — how we started, what we learned and practical steps

At the beginning we were really lost. This challenge introduced us to ROS and to programming for robotics. We spent time learning the basics and discovering how the Jetson Nano actually works. Early on we didn’t realise the Jetson Nano is a full computer (it runs Linux and comes with a complete OS image). After that discovery we changed our workflow: we started connecting to the Jetson by SSH and doing most of the development remotely instead of relying on the mobile app.

Below we describe the story briefly, the practical steps that helped us get a stable development environment, and concrete commands we ran to improve responsiveness.

## Our early mistakes and what changed
- At first we followed high-level documentation and experimented with the mobile app for calibration and quick tests. The app is useful for demos and manual servo control, but it hides the system-level details we needed to fix problems and automate workflows.
- After we learned the Jetson Nano includes a full OS we switched to an SSH-first workflow. Working over SSH made it straightforward to run ROS nodes, inspect logs, update packages and run Jupyter notebooks on the Jetson.
- The system felt slow at first. Running standard OS updates and removing unused packages noticeably improved responsiveness. After those updates the Jetson was much more usable for development and running the vision pipeline.

## Practical checklist (what we do first)
- Power the Jetson Nano and confirm it boots (headless or desktop image).
- Connect to the Jetson over SSH from your host machine (replace <user> and <jetson_ip>):

```bash
ssh <user>@<jetson_ip>
```

- Install OS and Python package updates (these helped our slow-first-boot issues):

```bash
sudo apt update && sudo apt upgrade -y
sudo apt autoremove -y
sudo reboot
```

- Create a small working directory on the Jetson for notebooks, captures and calibration images. We used `~/dofbot_work`.
- Use `tmux` or `screen` for long-running processes so SSH sessions are resilient to disconnects:

```bash
sudo apt install -y tmux
tmux new -s dofbot
```

## Camera and calibration notes
- We initially used the mobile app to get familiar with servo motions and the camera, but for calibration we used a Jupyter notebook on the Jetson so we could capture many images reproducibly.
- Recommended quick camera check (on the Jetson):

```bash
python3 -c "import cv2; cap=cv2.VideoCapture(0); print('open', cap.isOpened()); ret, f = cap.read(); print('captured', ret); cap.release()"
```

- If the camera returns black frames: re-seat the CSI ribbon (or USB connector), check the camera id (0 or 1), and reboot. We also exported a writable Matplotlib config dir to avoid warnings during notebook runs:

```python
import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib_config'
import matplotlib.pyplot as plt
```

## Improving responsiveness — what we ran
We addressed perceived slowness primarily by installing updates and removing unused packages (commands above). Additional small steps that helped us during development:
- Close heavy GUI applications on the Jetson and prefer a headless session over SSH when running Vision/ROS workloads.
- Use `tmux` to detach long-running nodes or notebook kernels and keep them alive after SSH disconnects.
- Reboot after kernel or package upgrades to ensure drivers (camera, GPU) are fully reloaded.

## SSH-first workflow tips
- Keep a small set of helper scripts on the Jetson for starting the camera publisher, the AI node and the I2C bridge (we use simple shell wrappers that set ROS params and `rosrun`/`roslaunch`).
- Use `roslaunch` with parameter files so topic and node names are consistent across restarts.
- For interactive work, forward a single port for Jupyter or use an SSH tunnel to access the Jetson browser UI securely from your host.

## Final note
We learned the most by trying things and then simplifying: move to SSH, keep the Jetson updated, and automate common start/stop tasks with small scripts. These small changes made the rest of the project much easier and reduced time lost to configuration issues.
