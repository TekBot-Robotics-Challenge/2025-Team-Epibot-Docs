# ROS 2 Humble – Sensor Data Publisher

## Project Overview

This ROS 2 Humble project implements a **publisher node** that simulates sensor data by generating **random temperature, humidity, and pressure values** and publishes them periodically to a topic named `/sensor_data` using a custom message interface.

---

## Objectives

* Generate random sensor values (temperature, humidity, pressure)
* Publish this data every 500 milliseconds on a custom topic
* Use a custom message definition

---

## Project Structure

```
sensor_data_evaluation
├── custom_interfaces
│   ├── CMakeLists.txt
│   ├── msg
│   │   └── DataCollect.msg
│   └── package.xml
└── sensor_data_evaluation
    ├── build
    │   ├── CMakeCache.txt
    │   └── CMakeFiles
    │       ├── 3.30.5
    │       │   ├── CMakeCCompiler.cmake
    │       │   ├── CMakeCXXCompiler.cmake
    │       │   ├── CMakeDetermineCompilerABI_C.bin
    │       │   ├── CMakeDetermineCompilerABI_CXX.bin
    │       │   ├── CMakeSystem.cmake
    │       │   ├── CompilerIdC
    │       │   │   ├── a.out
    │       │   │   └── CMakeCCompilerId.c
    │       │   └── CompilerIdCXX
    │       │       ├── a.out
    │       │       └── CMakeCXXCompilerId.cpp
    │       ├── cmake.check_cache
    │       └── CMakeConfigureLog.yaml
    ├── CMakeLists.txt
    ├── include
    │   └── sensor_data_evaluation
    │       ├── publisher.hpp
    │       └── subscriber.hpp
    ├── launch
    │   └── sensor_launch.py
    ├── package.xml
    └── src
        ├── publisher.cpp
        └── subscriber.cpp
```

---

## Custom Message Interface

The project uses a custom message defined in:

**`custom_interfaces/msg/DataCollect.msg`**

```msg
int32 temperature
int32 humidity
int32 pressure
```

---

## Project Behavior

### 1. Initialization

The ROS 2 node `_publisher` is initialized, which includes:

* A **publisher** of type `custom_interfaces::msg::DataCollect`
* A **wall timer** that triggers a callback every 500 milliseconds

### 2. Data Publishing

Every 500 milliseconds:

* A `Robot` object generates new random sensor values
* These values are assigned to a `DataCollect` message
* The message is published on the topic `/sensor_data`

---

## Project Architecture

The project follows an object-oriented architecture with two main classes:
1. **Publisher**: Inherits from `rclcpp::Node` and handles ROS2 publishing
2. **Robot**: Simulates a robot with sensors and generates random data

The data flow is as follows:
```
Robot (generates data) → Publisher (publishes data) → "sensor_data" Topic
```

## Implementation Details

### publisher.hpp

```cpp
#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

// Standard includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <ctime>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/data_collect.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
    public:
        Publisher();
        ~Publisher() = default;

    private:
        void publishing();
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<custom_interfaces::msg::DataCollect>::SharedPtr _publisher;
        size_t _count;
};

class Robot
{
    public:
        Robot() = default;
        ~Robot() = default;
        void update();
        void display(custom_interfaces::msg::DataCollect&);
    private:
        int _temp;       // Temperature in Celsius
        int _hum;       // Humidity percentage (0-100)
        int _pressions;  // Pressure in hPa (0-1500)
};

#endif
```

#### 1. `Publisher` Class

Inherits from `rclcpp::Node`. It handles:

* Timer creation
* Periodic publishing

```cpp
class Publisher : public rclcpp::Node
{
    public:
        Publisher();
        ~Publisher() = default;

    private:
        void publishing();
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<custom_interfaces::msg::DataCollect>::SharedPtr _publisher;
        size_t _count;
};
```

#### 2. `Robot` Class

Handles sensor value generation and message population:

```cpp
class Robot
{
    public:
        Robot() = default;
        ~Robot() = default;
        void update(); // Generates random values
        void display(custom_interfaces::msg::DataCollect&); // Fills the message
    private:
        int _temp;
        int _hum;
        int _pressions;
};
```

#### Key Points:
- **Publisher**:
  - Inherits from `rclcpp::Node` for ROS2 functionality
  - Uses a timer for regular publishing (500ms interval)
  - Publishes to "sensor_data" topic with custom `DataCollect` message
  
- **Robot**:
  - Simulates sensors with random values
  - `update()`: Generates new random values
  - `display()`: Fills ROS message with current values

---

### Source File – `publisher.cpp`

```cpp
#include "../include/sensor_data_evaluation/publisher.hpp"
#include "custom_interfaces/msg/data_collect.hpp"
#include <iostream>

// Publisher constructor
Publisher::Publisher() : Node("_publisher"), _count(0)
{
  _publisher = create_publisher<custom_interfaces::msg::DataCollect>("sensor_data", 10);
  _timer = create_wall_timer(500ms, std::bind(&Publisher::publishing, this));
}

// Publishing callback
void Publisher::publishing()
{
  custom_interfaces::msg::DataCollect message;
  Robot bot;
  bot.update();
  bot.display(message);
  _publisher->publish(message);
  _count++;
}

// Updates sensor values
void Robot::update()
{
  std::srand(std::time(0));
  _temp = std::rand() % 100;         // Temperature 0-100°C
  _hum = std::rand() % 100;          // Humidity 0-100%
  _pressions = std::rand() % 1500;   // Pressure 0-1500 hPa
}

// Fills ROS message
void Robot::display(custom_interfaces::msg::DataCollect &msg)
{
  msg.temperature = _temp;
  msg.humidity = _hum;
  msg.pressure = _pressions;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
```

#### `Publisher::Publisher()`

* Creates the publisher and timer
* Publishes messages at a fixed interval of 500 milliseconds

#### `Publisher::publishing()`

* Creates a message
* Uses the `Robot` class to generate and insert data
* Publishes the message

#### `Robot::update()`

* Uses `std::rand()` to generate:

  * Temperature: 0 to 99
  * Humidity: 0 to 99
  * Pressure: 0 to 1499

> Note: `std::srand(std::time(0))` is called at each update, which resets the random seed each time.

#### `Robot::display()`

* Assigns the generated values to the `DataCollect` message

#### `main()`

* Initializes the ROS 2 system
* Spins the node
* Shuts down gracefully

---

## Build Instructions

### 1. Add Dependencies in `CMakeLists.txt`

Ensure these lines are present:

```cmake
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(publisher_node src/publisher.cpp)
ament_target_dependencies(publisher_node rclcpp custom_interfaces)

install(TARGETS
  publisher_node
  DESTINATION lib/${PROJECT_NAME})
```

### 2. Add Dependencies in `package.xml`

```xml
<depend>rclcpp</depend>
<depend>custom_interfaces</depend>
```

---

## How to Run

### 1. Build the Workspace

```bash
colcon build --packages-select sensor_data_evaluation
source install/setup.bash
```

### 2. Run the Node

```bash
ros2 run sensor_data_evaluation publisher_node
```

### 3. Echo the Topic

```bash
ros2 topic echo /sensor_data
```

#### Sample Output:

```
temperature: 78
humidity: 41
pressure: 1032
---
```

---
