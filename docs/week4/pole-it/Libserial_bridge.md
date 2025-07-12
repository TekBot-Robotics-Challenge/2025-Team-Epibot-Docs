Perfect, Jordan. Here's a **complete and professional English documentation** of your ROS 2 and LibSerial bridge, formatted from A to Z.

---

# **ROS 2 Serial Bridge with Arduino using LibSerial**

## Overview

This module establishes **bidirectional communication** between a **ROS 2 (Humble)** node and an **Arduino** device using the **LibSerial** library. It reads serial data from the Arduino and publishes it as ROS messages, while also interpreting the incoming messages to control or notify the Arduino accordingly.

---

## Table of Contents

1. [Dependencies](#dependencies)
2. [Folder Structure](#folder-structure)
3. [How It Works](#how-it-works)
4. [Serial Communication Protocol](#serial-communication-protocol)
5. [Code Explanation](#code-explanation)

   * [Constructor and Destructor](#1-constructor--destructor)
   * [Serial Setup](#2-serial-setup)
   * [Serial Reading Loop](#3-serial-reading-loop)
6. [How to Build and Run](#how-to-build-and-run)
7. [Testing and Debugging](#testing-and-debugging)
8. [Extending the Project](#extending-the-project)

---

## Dependencies

Ensure the following are installed:

* ROS 2 Humble
* LibSerial (C++ Serial Communication Library)
* CMake
* Colcon
* A connected Arduino board that communicates over USB serial

Install LibSerial on Ubuntu:

```bash
sudo apt-get install libserial-dev
```

---

## Folder Structure

```
arduino_button_pkg/
├── include/
│   └── arduino_button_pkg/
│       └── serial_publisher.hpp
├── src/
│   └── serial_publisher.cpp
├── CMakeLists.txt
└── package.xml
```

---

## How It Works

* The node `arduino_bridge` scans for available serial ports and tries to connect to one.
* Once connected, it launches a dedicated thread to read data from the serial interface.
* Received messages are parsed and published to the topic `/button_topic`.
* Messages like `GARBAGE RED` or `NONE` are recognized and used to send commands back to the Arduino (`MOTOR ON`, `MOTOR OFF`, `ko`, etc.).
* It ensures robust communication by acknowledging a special startup message from the Arduino: `"ARDUINOs CONNECTED"`.

---

## Serial Communication Protocol

The node uses a **simple string-based protocol** to communicate with the Arduino. Here is the protocol outline:

### From Arduino to ROS:

* `"ARDUINOs CONNECTED"` → Handshake signal
* `"GARBAGE RED"`, `"GARBAGE GREEN"` → When a garbage is on the convoyer
* `"NONE"` → No garbage is on the convoyer
* Any other string → Unknown / malformed data

### From ROS to Arduino:

* `"ROS CONNECTED"` → Sent once after successful port connection
* `"MOTOR ON"` → Sent when a valid `GARBAGE COLOR` is received to start the convoyer
* `"MOTOR OFF"` → Sent when `"NONE"` is received to stop the convoyer
* `"ko"` → Sent if the command is invalid

---

## Code Explanation

### serial_publisher.hpp

```cpp

/*
** EPIBOT PROJECT, 2025
** 2025-Team-Epibot-Code
** File description:
** serial_publisher
*/

#ifndef BRIDGE_HPP
    #define BRIDGE_HPP
    #include <rclcpp/rclcpp.hpp>
    #include <std_msgs/msg/string.hpp>
    #include <libserial/SerialPort.h>
    #include <thread>
    #include <chrono>
    #include <algorithm>

class ArduinoBridge : public rclcpp::Node {
    public:
        ArduinoBridge(); // Constructor
        ~ArduinoBridge(); // Destructor
    private:
        void setup_serial(); // to set up the serial port
        void read_serial(); // to communicate with the serial port
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher; /* To publish messages to the subscriber
        who communicates directly with the website*/
        std::unique_ptr<LibSerial::SerialPort> _serial_port; // the serail port
        std::thread _serial_thread; /* threads to communicate with both the serial
        port and the subscriber that updates the website*/
        std::atomic<bool> _running{true}; // if the program is running or not
        void setup_port(std::string port); // to set up the port;
        void serial_communication(std::string line,
            std::vector <std::string> colors, std::string color_cmd); // to etablish the communication with the serial port
};
#endif

```

### 1. Constructor & Destructor

#### serial_publisher.cpp

```cpp

/*
** EPIBOT PROJECT, 2025
** 2025-Team-Epibot-Code
** File description:
** serial_publisher
*/

#include "../include/serial_publisher.hpp"

bool verif = false;

ArduinoBridge::ArduinoBridge() : Node("arduino_bridge")
{
    _publisher = this->create_publisher<std_msgs::msg::String>("button_topic", 10);
    setup_serial();
    if (_serial_port && _serial_port->IsOpen()) {
        _serial_port->Write("ROS CONNECTED\n");
    }
    _serial_thread = std::thread(&ArduinoBridge::read_serial, this);
    RCLCPP_INFO(this->get_logger(), "Arduino Bridge started");
}

ArduinoBridge::~ArduinoBridge()
{
    _running = false;
    if (_serial_thread.joinable()) {
        _serial_thread.join();
    }
    if (_serial_port && _serial_port->IsOpen()) {
        _serial_port->Close();
    }
}

void ArduinoBridge::setup_port(std::string port)
{
    _serial_port = std::make_unique<LibSerial::SerialPort>();
    _serial_port->Open(port);
    _serial_port->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    _serial_port->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    _serial_port->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    _serial_port->SetParity(LibSerial::Parity::PARITY_NONE);
    _serial_port->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
}

void ArduinoBridge::setup_serial()
{
    std::vector<std::string> possible_ports = {
        "/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", 
        "/dev/ttyUSB1", "COM3", "COM4", "COM5"
    };
    
    for (const auto& port : possible_ports) {
        try {
            setup_port(port);
            if (_serial_port->IsOpen()) {
                RCLCPP_INFO(this->get_logger(), "Connected on port %s", port.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(2));
                return;
            }
        } catch (const LibSerial::OpenFailed& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to open the port %s: %s", port.c_str(), e.what());
            continue;
        } catch (const LibSerial::AlreadyOpen& e) {
            RCLCPP_WARN(this->get_logger(), "Port %s already opened: %s", port.c_str(), e.what());
            continue;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error with port %s: %s", port.c_str(), e.what());
            continue;
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "No serial port found!");
}

void ArduinoBridge::serial_communication(std::string line, std::vector <std::string> colors, std::string color_cmd)
{
    if (line == "ARDUINO CONNECTED") {
        verif = true;
        RCLCPP_INFO(this->get_logger(), "ARDUINO CONNECTED");
        return;
    }
    if (verif) {
        color_cmd = line.c_str() + 8;
        if (strncmp(line.c_str(), "GARBAGE ", 8) == 0 &&
        std::find(colors.begin(), colors.end(), color_cmd) != colors.end()) {
            _serial_port->Write("MOTOR ON");
            RCLCPP_INFO(this->get_logger(), "MOTOR ON");
            auto msg = std_msgs::msg::String();
            msg.data = line;
            _publisher->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Message sent: %s", line.c_str());
        }else if (line == "NONE") {
            _serial_port->Write("MOTOR OFF");
            RCLCPP_INFO(this->get_logger(), "MOTOR OFF");
        } else {
            _serial_port->Write("ko");
            RCLCPP_INFO(this->get_logger(), "ko");
        }
    }
}
    
void ArduinoBridge::read_serial()
{
    std::vector <std::string> colors = {"RED", "GREEN", "YELLOW", "BLUE"};
    std::string color_cmd;
    std::string line;

    while (_running && rclcpp::ok()) {
        try {
            if (_serial_port && _serial_port->IsOpen() && _serial_port->GetNumberOfBytesAvailable() > 0) {
                _serial_port->ReadLine(line, '\n', 100);
                line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
                line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
              
                if (!line.empty()) {
                    serial_communication(line, colors, color_cmd);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error while serial reading %s", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

```

#### `ArduinoBridge::ArduinoBridge()`

* Initializes the ROS 2 node.
* Creates a publisher to `/button_topic`.
* Calls `setup_serial()` to establish serial connection.
* Starts a background thread to read from the serial interface.

#### `ArduinoBridge::~ArduinoBridge()`

* Cleans up the thread and closes the serial port if open.
* Ensures no thread leaks or hanging processes.

---

### 2. Serial Setup

#### `ArduinoBridge::setup_serial()`

* Scans a list of possible serial device paths:

  ```cpp
  {"/dev/ttyACM0", "/dev/ttyUSB0", ..., "COM3"}
  ```
* For each port, it attempts to:

  * Open it with LibSerial
  * Set parameters:

    * Baud rate: 115200
    * Character size: 8
    * No flow control
    * No parity
    * 1 stop bit
  * If successful, sends `"ROS CONNECTED"` as handshake and returns.
* If no ports succeed, logs an error.

---

### 3. Serial Reading Loop

#### `ArduinoBridge::read_serial()`

* Continuously runs in a separate thread.
* Waits for `"ARDUINOs CONNECTED"` before processing any commands.
* When active:

  * Parses incoming strings.
  * If message starts with `"GARBAGE "` followed by a known color (`RED`, `GREEN`, etc.), it:

    * Sends `"MOTOR ON"` to Arduino
    * Publishes the command to `/button_topic`
  * If message is `"NONE"`, sends `"MOTOR OFF"`
  * Otherwise sends `"ko"` to indicate invalid input
* Uses `std::find` to check if the color exists in a valid color list.

---

## How to Build and Run

### 1. Clone and Build the Package

```bash
cd ~/ros2_ws/src
git clone git@github.com:TekBot-Robotics-Challenge/2025-Team-Epibot-Code.git
cd ~/ros2_ws
colcon build
```

### 2. Source and Run

```bash
source install/setup.bash
ros2 run arduino_button_pkg arduino_bridge
```

<video src="/week4/videos/test_bridge.mp4" controls autoplay muted style="width: 100%; max-width: 800px; height: auto;">
  Your browser does not support the video tag.
</video>

---

## Testing and Debugging

### On the Arduino Side:

Ensure your Arduino sends:

* `"ARDUINOs CONNECTED\n"` after setup
* Messages like `"GARBAGE RED\n"`, `"NONE\n"` when garbage is on the convoyer or not

### On ROS Side:

* Use `rqt_console` or `ros2 topic echo /button_topic` to inspect published messages.
* Confirm logs print port detection and message flow.

---

## Extending the Project

You can easily extend this bridge by:

* Adding support for **bidirectional ROS messages** (subscriber node → Arduino commands)
* Using a **custom message type** instead of raw strings
* Adding **timestamping**, **checksums**, or **more complex protocol framing**
* Adding **service calls** to interact with the Arduino in real time

---

## Notes

* The protocol assumes the Arduino will always send newline-terminated (`\n`) strings.
* The system is fault-tolerant: if the port is busy or closed, it logs a warning and continues.
* You may adapt port names for your OS (e.g., `COMx` for Windows, `/dev/ttyUSBx` for Linux).

---
