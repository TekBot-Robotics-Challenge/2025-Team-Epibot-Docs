# Test2: Sensor Data Evaluation – ROS2-humble

## Overview

This project is part of **Test 2: Introduction to ROS 2**. It introduces the core concepts of **ROS 2 (Robot Operating System)** through a simple simulation of sensor data communication using **nodes**, **topics**, **publishers**, and **subscribers**.

You are required to implement a ROS 2 package that simulates sensor data generation and validation using the **C++** or **Python** programming languages.

---

## Objectives

* Understand and use the ROS 2 communication model (Publisher/Subscriber).
* Create a ROS 2 package that includes:

  * A **publisher node** that sends random sensor data every 0.5 seconds.
  * A **subscriber node** that validates if the data is within expected ranges.
* Ensure the whole system runs without errors.

---

## What You Will Learn

* How to create and configure a ROS 2 package.
* How to define custom messages (if needed).
* How to use **rclcpp** (for C++) or **rclpy** (for Python).
* How to publish and subscribe to topics.
* How to implement periodic publishing using timers.
* How to validate sensor data and log messages accordingly.

---

## Technical Specifications

### 1. **Package Name**

```
sensor_data_evaluation
```

---

### 2. **Publisher Node**

| Feature      | Description                                                                                                |
| ------------ | ---------------------------------------------------------------------------------------------------------- |
| Node name    | Any name (e.g., `_publisher`)                                                                              |
| Topic        | `/sensor_data`                                                                                             |
| Message type | Custom message with 3 fields (temperature, humidity, pressure)                                             |
| Frequency    | Publishes every **0.5 seconds**                                                                            |
| Data range   | - Temperature: **15°C to 35°C**  <br> - Humidity: **30% to 70%**  <br> - Pressure: **950 hPa to 1050 hPa** |

---

### 3. **Subscriber Node**

| Feature          | Description                                                                 |
| ---------------- | --------------------------------------------------------------------------- |
| Node name        | Any name (e.g., `_subscriber`)                                              |
| Subscribed topic | `/sensor_data`                                                              |
| Action           | Validates incoming data fields against defined ranges                       |
| Logging          | - If data is valid → log OK message  <br> - If not → log error with details |

---

## Suggested Project Structure

```bash
sensor_data_evaluation/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── sensor_data_evaluation/
│       └── publisher.hpp
├── src/
│   ├── publisher.cpp
│   └── subscriber.cpp
├── msg/
│   └── DataCollect.msg
```

---

## 📦 Custom Message Definition

If using a custom message, create `msg/DataCollect.msg`:

```plaintext
float32 temperature
float32 humidity
float32 pressure
```

Then, update `CMakeLists.txt` and `package.xml` to build messages.

---

## Build & Run

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select sensor_data_evaluation
source install/setup.bash
```

### Run Nodes

```bash
# Terminal 1
ros2 run sensor_data_evaluation publisher_node

# Terminal 2
ros2 run sensor_data_evaluation subscriber_node
```

> Replace `publisher_node` and `subscriber_node` with your actual executable names.

---

## References

* [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
* [Creating ROS 2 Packages](https://docs.ros.org/en/humble/Tutorials/Creating-Your-First-ROS2-Package.html)
* [Writing a Publisher and Subscriber in C++](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
