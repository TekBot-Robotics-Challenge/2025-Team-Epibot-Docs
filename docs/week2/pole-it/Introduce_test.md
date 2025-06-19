# ROS2 Test Project: Sensor Data Evaluation

## Introduction

This test project was designed as an introduction to the ROS2 (Robot Operating System 2) framework. The objective was to get hands-on experience with ROS2's core communication concepts, including **nodes**, **topics**, and the **publish/subscribe** model. The test consists of implementing a ROS2 package with two main components: a *publisher node* and a *subscriber node*, using both **C++** and **Python**.

## Objectives of the Test

The primary goal was to create a ROS2 package named `sensor_data_evaluation` that includes:

- A **publisher node** that generates random sensor data (temperature, humidity, and pressure) and publishes it every 0.5 seconds on a topic named `/sensor_data`.
- A **subscriber node** that listens to this topic and checks whether the received data falls within predefined valid ranges:
  - Temperature: 15°C to 35°C
  - Humidity: 30% to 70%
  - Pressure: 950 hPa to 1050 hPa
Additionally, a **launch file** was required to run both nodes simultaneously.

## Skills and Concepts Learned
Through this test, I gained practical experience in the following areas:

- Creating and organizing a ROS2 package.
- Writing ROS2 nodes in both C++ and Python.
- Publishing and subscribing to topics.
- Working with custom data structures and message types.
- Using launch files to manage and execute multiple nodes.
- Logging and validating data in real-time within a ROS2 system.

# Bonus: Multimachine Communication with ROS2 (Fast DDS)

## Introduction

As a bonus to this project, I explored **Multimachine communication using ROS2**, where ROS2 nodes can run on different physical machines and communicate with each other over a local network. This is made possible by **DDS (Data Distribution Service)**, which is the default middleware in ROS2 (Fast DDS). It allows **decentralized**, **serverless** communication through a **publish/subscribe model**.

## Key Concepts of DDS in ROS2

- **Automatic Discovery**: Nodes automatically detect each other on the network. There is no need for a central master or `roscore`.
- **Direct Communication**: Messages are sent directly from the publisher to the subscriber.
- **Quality of Service (QoS)**: The behavior of message transmission (reliability, persistence, frequency) can be finely tuned using QoS profiles.

## Steps to Set Up Multimachine ROS2 Communication

### 1. Ensure All Devices Are on the Same Network

Make sure that all the computers (e.g., PC1 and PC2) are connected to the **same local network** (Wi-Fi or Ethernet). You can verify this by checking that their IP addresses are in the same subnet.

```bash
# On each machine, get your IP address:
ip a   # or use `ifconfig` if available
```

### 2 Network Connectivity

Check that each machine can reach the other
```bash
# From PC1 to PC2:
ping <IP_ADDRESS_OF_PC2>

# From PC2 to PC1:
ping <IP_ADDRESS_OF_PC1>
```
If the ping is successful, then the machines are properly connected.

### 3. Set the ROS Domain ID

To isolate ROS2 communication, each machine must have the same ROS_DOMAIN_ID. You can set it temporarily with:

```bash 
export ROS_DOMAIN_ID=0
```

On PC1 (Publisher)
Launch the publisher node that sends sensor data:

```bash
ros2 run sensor_data_evaluation sensor_publisher
```

On PC2 (Subscriber)
Launch the subscriber node that receives and checks the data:
```bash
ros2 run sensor_data_evaluation sensor_subscriber
```
You should now see the subscriber receiving messages published from the other machine.

### 4. Architecture Diagram

```text
            ┌────────────┐                          ┌────────────┐
            │   PC1      │                          │   PC2      │
            │------------│                          │------------│
            │ Publisher  │── /sensor_data topic ──► │ Subscriber │
            └────────────┘                          └────────────┘
                   ▲                                      ▲
                   │                                      │
                   ▼                                      ▼
            ┌────────────┐                          ┌────────────┐
            │   PC3      │                          │   ...      │
            │------------│                          │------------│
            │ Subscriber │                          │ Subscriber │
            └────────────┘                          └────────────┘

     All machines on the same local network and sharing ROS_DOMAIN_ID=0
```

Multimachine communication in ROS2 is straightforward when the following conditions are met:

- All devices are on the same local network.
- They share the same ROS_DOMAIN_ID.
- ROS2 is installed and sourced correctly on each machine.
- Firewall rules do not block DDS traffic.

[Check this link if you want to know about  this topic](https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/)
