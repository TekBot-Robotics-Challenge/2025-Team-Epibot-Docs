# VM (Virtual Machine) installation and VSCode setup

We documented how we found, imported and prepared the VM used for AI and MoveIt.

Finding and downloading the VM
- The vendor provides a prebuilt VM image (check the `Final_Test/final_version/ubuntu20.04-yahboom` folder for the image files if included), or download the OVA/VMX from the vendor page.

Importing into VirtualBox / VMware
- Import the OVA/VMX into your virtualization tool.
- Set networking to `Bridged` or `Host-only` depending on whether you want the VM to be accessible from the Jetson. For multi-machine ROS, `Bridged` is usually easiest.

Guest configuration
- Increase shared memory if you run heavy AI workloads.
- Install `open-vm-tools` / `virtualbox-guest-additions` for clipboard and folder sharing if needed.

Installing VSCode on the VM
- Download and install Visual Studio Code for Ubuntu (deb package) or use the Snap package:

```
sudo snap install --classic code
```

- Install the Python extension and the Remote - SSH extension to develop on the Jetson from VSCode if desired.
