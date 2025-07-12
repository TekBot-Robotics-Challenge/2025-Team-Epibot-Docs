# Overview

Here is the `ColorBridge` class, a ROS2 node that listens for color messages, extracts the color, and sends it as a JSON payload to our website using HTTP POST requests. The node subscribes to the `color_reader` topic, processes incoming messages, and communicates with a web API.

---

# Class: `ColorBridge`

The `ColorBridge` class inherits from `rclcpp::Node` and provides the following main functionalities:

- **Subscription** to a ROS2 topic for color messages.
- **Color extraction** from incoming messages.
- **HTTP POST** to send the extracted color to an API in JSON format.

## Class Declaration

```cpp
class ColorBridge : public rclcpp::Node
{
    public:
        ColorBridge();
        std::string retrieve_color(const std::string msg);
        void send_color_to_website(const std::shared_ptr<std_msgs::msg::String> msg);
        ~ColorBridge(){}
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};
```

---

# Constructor

The constructor initializes the node and creates a subscription to the `color_reader` topic.

```cpp
ColorBridge::ColorBridge() : Node("color_bridge") {
    this->subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "color_reader", 10,
        std::bind(&ColorBridge::send_color_to_website, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Start to send data to the website");
}
```

**Explanation:**  
- The node is named `"color_bridge"`.
- It subscribes to the `color_reader` topic.
- Incoming messages trigger the `send_color_to_website` callback.

---

# Method: `retrieve_color`

This method extracts a color string from the input message.

```cpp
std::string ColorBridge::retrieve_color(const std::string msg)
{
    std::vector<std::string> colors{"RED", "GREEN", "BLUE", "YELLOW"};

    for (const auto &color : colors) {
        size_t pos = msg.find(color);
        if (pos != std::string::npos)
            return color;
    }
    return std::string();
}
```

**Explanation:**  
- Searches for one of the predefined colors (`RED`, `GREEN`, `BLUE`, `YELLOW`) in the input string.
- Returns the first color found, or an empty string if none are found.

---

# Method: `send_color_to_website`

This callback is triggered when a message is received. It extracts the color and sends it to a web API.

```cpp
void ColorBridge::send_color_to_website(const std::shared_ptr<std_msgs::msg::String> msg)
{
    std::string color = retrieve_color(msg->data);
    if (color.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse color");
        return;
    }
    CURL* curl = curl_easy_init();
    nlohmann::json jsonData;
    jsonData["color"] = color;
    if (curl) {
        std::string jsonString = jsonData.dump();
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        std::string url = "https://conveyor-site.vercel.app/api/data";
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonString.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        CURLcode res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            RCLCPP_ERROR(this->get_logger(), "CURL error: %s", curl_easy_strerror(res));
        }
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CURL");
    }
}
```

**Explanation:**  
- Calls `retrieve_color` to extract the color from the message.
- If a color is found, creates a JSON object and sends it via HTTP POST to the specified URL.
- Uses libcurl for HTTP requests and nlohmann::json for JSON serialization.
- Logs errors if color parsing or HTTP requests fail.

---

# Main Function

The entry point initializes ROS2, creates the node, and starts spinning.

```cpp
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

**Explanation:**  
- Initializes ROS2.
- Instantiates the `ColorBridge` node.
- Spins the node to process callbacks.
- Shuts down ROS2 on exit.
---