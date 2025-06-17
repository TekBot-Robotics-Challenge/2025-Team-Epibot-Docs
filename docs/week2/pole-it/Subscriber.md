# Subscriber

## Overview

This ROS2 C++ node subscribes to the `sensor_data` topic and evaluates incoming sensor data (temperature, humidity, and pressure) using a custom ROS2 message type: `custom_interfaces::msg::DataCollect`. It prints a color-coded summary to the terminal, indicating whether each value is within the given range.

---

## The `Subscriber` Class

The core of this node is the `Subscriber` class, which inherits from `rclcpp::Node`. It manages the subscription and the logic for analyzing and displaying sensor data.

```cpp
class Subscriber : public rclcpp::Node {
public:
    Subscriber();
    ~Subscriber() = default;
    void topic_callback(const custom_interfaces::msg::DataCollect &msg);
    std::string analyseInfo(double temp, double hum, double pression);
private:
    rclcpp::Subscription<custom_interfaces::msg::DataCollect>::SharedPtr subscription_;
};
```

### Key Methods

- **Constructor (`Subscriber()`)**  
  Initializes the node and creates a subscription to the `sensor_data` topic using the custom message type.

- **`topic_callback(const custom_interfaces::msg::DataCollect &msg)`**  
  Callback function triggered on each new message. It calls `analyseInfo` and logs the formatted analysis.

- **`analyseInfo(double temp, double hum, double pression)`**  
  Evaluates the sensor data and returns a formatted string with color-coded results.

---

## How It Works

### Initialization

The constructor sets up the subscription:

```cpp
Subscriber::Subscriber() : Node("_subscriber")
{
    subscription_ = this->create_subscription<custom_interfaces::msg::DataCollect>(
        "sensor_data", 10, std::bind(&Subscriber::topic_callback, this, _1));
}
```

### Message Analysis

The `analyseInfo` method checks each value and builds a color-coded summary:

```cpp
std::string Subscriber::analyseInfo(double temp, double hum, double pression)
{
    std::string str;
    if (temp >= 15 && temp <= 30) {
        str += "\n\n\033[32m- Temperature \"" + std::to_string(temp);
        str += "\" is included between [15, 30]\033[0m\n\n";
    } else {
        str += "\n\n\033[31m- Temperature \"" + std::to_string(temp);
        str += "\" is not included between [15, 30]\033[0m\n\n";
    }
    if (hum >= 30 && hum <= 70) {
        str += "\033[32m- Humidity \"" + std::to_string(hum);
        str += "\" is included between [30, 70]\033[0m\n\n";
    } else {
        str += "\033[31m- Humidity \"" + std::to_string(hum);
        str += "\" is not included between [30, 70]\033[0m\n\n";
    }
    if (pression >= 950 && pression <= 1050) {
        str += "\033[32m- Pressure \"" + std::to_string(pression);
        str += "\" is included between [950, 1050]\033[0m\n\n";
    } else {
        str += "\033[31m- Pressure \"" + std::to_string(pression);
        str += "\" is not included between [950, 1050]\033[0m\n\n";
    }
    return str;
}
```

### Callback Function

The callback processes each message and logs the analysis:

```cpp
void Subscriber::topic_callback(const custom_interfaces::msg::DataCollect &msg)
{
    std::string str = this->analyseInfo(msg.temperature, msg.humidity, msg.pressure);
    RCLCPP_INFO(this->get_logger(), "%s", str.c_str());
}
```

### Main Function

The main function initializes ROS2, spins the node, and shuts down cleanly:

```cpp
int main(int ac, char **av)
{
    rclcpp::init(ac, av);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}
```

---

## Example

If the node receives the following message (via the custom message):

- `temperature = 25`
- `humidity = 50`
- `pressure = 1000`

The output will be:

- Temperature "25" is included between [15, 30]
- Humidity "50" is included between [30, 70]
- Pressure "1000" is included between [950, 1050]

Each line is color-coded in the terminal for clarity.

---
