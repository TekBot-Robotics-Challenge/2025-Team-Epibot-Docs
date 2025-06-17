## Download ROS2-Humble

### Step 1: Install Distrobox

Create a virtual container to install Ubuuntu 22.04

```bash
sudo dnf install distrobox #(Fedora)
sudo apt install distrobox #(Ubuntu)
```

Distrobox allows you to create a virtual machine to obtain Ubuntu 22.04 to install ROS2-humble. ROS2-humble is only available on Ubuntu 22.04, so we need to have Distrobox (if you don't have ubuntu 22.04)

---

### Step2: Create a virtual container

First we need to create a virtual container and enter this container

```bash
distrobox create --name container_name --image ubuntu:22.04
distrobox enter container_name
```

 --image 22.04 because we would like to have an Ubuntu 22.04 operating system

---

### Step3: Download Ros2-Humble

Firstly, Configure the repository

```bash
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Secondly, Install Ros2-Humble

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

Finaly, Configure the repository

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

Now ROS2-Humble is finaly installed!
To test the downloading you can copied the following command:

```bash
ros2 run demo_nodes_cpp talker
```
---
