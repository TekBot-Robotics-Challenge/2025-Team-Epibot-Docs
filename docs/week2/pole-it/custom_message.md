# Create a Custom Message

In ROS 2, **standard messages** (such as those in the `std_msgs` or `sensor_msgs` packages) provide common data types like integers, strings, arrays, and sensor data. These are useful for many basic communication needs between nodes.

However, robotics applications often require exchanging more complex or application-specific data. In such cases, **custom messages** allow you to define exactly the data structure your system needs, improving clarity, maintainability, and reducing ambiguity in communication.

## Best Practice: Creat a Package for Custom Messages

A recommended practice in ROS 2 development is to create a **dedicated package** for your custom messages. This approach offers several advantages:

- **Reusability:** Other packages in your project (or even in other projects) can depend on your message definitions without circular dependencies.
- **Modularity:** Keeping message definitions separate from business logic or node implementations makes your codebase cleaner and easier to maintain.
- **Build Efficiency:** Changes to message definitions only require rebuilding the message package and dependent packages, not the entire workspace.

**In summary:**  
While standard messages cover many use cases, custom messages are essential for complex or unique data exchange. Organizing them in a separate package is a best practice that enhances modularity, reusability, and maintainability in ROS 2


### 1. Creating the Package

To create a new ROS 2 package specifically for custom message definitions, I used the following command:
```bash
ros2 pkg create custom_interfaces --build-type ament_cmake
```

```text
custom_interfaces/
├── CMakeLists.txt
├── msg/
│   └── DataCollect.msg
└── package.xml
```

let me explain every file inside the package

# CMakeLists.txt:

## Overview
The `CMakeLists.txt` file is used to configure the build process of the project using CMake. It specifies the minimum required version of CMake, declares the project, finds necessary packages, and defines how the source code should be compiled and linked.

## Role

The `CMakeLists.txt` file is the main build configuration file for CMake-based projects. In the context of ROS 2 (Robot Operating System 2), it defines how the package is built, what dependencies it has, and how the resulting binaries and resources are installed. It acts as a blueprint for the build system, guiding it through the compilation and linking process.

## Importance
- **Build Automation:** It automates the process of compiling source code, linking libraries, and generating executables.
- **Dependency Management:** It ensures all required libraries and packages are found and used correctly.
- **Portability:** It allows the project to be built on different platforms and environments with minimal changes.
- **Integration with ROS 2:** It enables the package to be recognized and built within the ROS 2 ecosystem using `ament_cmake`.

## Content

A typical `CMakeLists.txt` for a ROS 2 package includes the following sections:

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_project_name)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
```

## Explanation of Content

1. **CMake Version Requirement**  
   `cmake_minimum_required(VERSION 3.5)`  
   Specifies the minimum version of CMake required.

2. **Project Declaration**  
   `project(my_project_name)`  
   Declares the project name.

3. **Finding Dependencies**  
   `find_package(ament_cmake REQUIRED)`  
   `find_package(rclcpp REQUIRED)`  
   Finds and loads necessary packages.

4. **Adding Executables**  
   `add_executable(my_node src/my_node.cpp)`  
   Defines how to build the executable from source files.

5. **Linking Libraries**  
   `ament_target_dependencies(my_node rclcpp)`  
   Links the executable with required libraries.

6. **Installation Instructions**  
   ```
   install(TARGETS
     my_node
     DESTINATION lib/${PROJECT_NAME}
   )
   ```
   Specifies where to install the built files.

7. **ament Package Macro**  
   `ament_package()`  
   Marks the package as an ament package for ROS2.
---

**In summary:**  
The `CMakeLists.txt` file is essential for building, managing dependencies, and integrating your project within the ROS 2 ecosystem. It ensures that your code is compiled and installed correctly, making your package reusable

# DataCollect.msg:

## Role

The `DataCollect.msg` file defines a custom message type for ROS 2. Message files (`.msg`) specify the structure of data that nodes exchange in a ROS 2 system. This file allows different parts of your robot software to communicate using a standardized data format.

## Importance

- **Standardized Communication:** It ensures that all nodes exchanging this message type understand the data structure.
- **Code Generation:** ROS 2 automatically generates code in multiple programming languages (C++, Python, etc.) based on this file, making it easy to use the message in your code.
- **Maintainability:** Centralizing the message definition makes it easier to update and maintain the data structure as your project evolves.

## Content
The content of `DataCollect.msg` describes the fields and their data types that make up the message:

```msg
# DataCollect.msg
float32 temperature
float32 humidity
float32 pressure
```

- `float32 temperature`: A floating-point value representing the temperature measurement.
- `float32 humidity`: A floating-point value representing the humidity measurement.
- `float32 pressure`: A floating-point value representing the pressure measurement.

Each line defines a field in the message, specifying its type and name.

---

**In summary:**  
The `DataCollect.msg` file is crucial for defining custom data structures used in ROS 2 node communication. It standardizes how information is shared, enables automatic code generation, and improves the maintainability of your robotic software.

# package.xml

## Role

The `package.xml` file is a manifest file used in ROS 2 packages. It provides essential metadata about the package, such as its name, version, description, maintainers, licenses, and dependencies. This file is crucial for the build system and ROS 2 tools to understand how to build, install, and use the package.

## Importance

- **Metadata:** Describes the package for users and tools.
- **Dependency Management:** Lists all dependencies required to build and run the package.
- **Licensing and Maintenance:** Specifies the license and maintainers for legal and support reasons.
- **Integration:** Allows ROS 2 tools to discover and use the package correctly.

## Example Content and Line-by-Line Explanation

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>                <!-- The name of your package -->
  <version>0.0.1</version>               <!-- The current version of your package -->
  <description>My custom ROS 2 package</description> <!-- A brief description -->
  <maintainer email="your.email@example.com">Your Name</maintainer> <!-- Maintainer info -->
  <license>Apache-2.0</license>          <!-- License type for your package -->

  <!-- Build tool dependency (always required for ROS 2 packages) -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Build dependencies (needed to build the code) -->
  <build_depend>rclcpp</build_depend>
  <build_depend>std_msgs</build_depend>

  <!-- Execution dependencies (needed at runtime) -->
  <exec_depend>rclcpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- Test dependencies (optional, for running tests) -->
  <!-- <test_depend>ament_lint_auto</test_depend> -->

  <!-- Export section for additional information -->
  <export>
    <!-- Additional export tags can go here -->
  </export>
</package>
```

### Explanation

- `<?xml version="1.0"?>`  
  Declares the XML version.
- `<package format="3">`  
  Starts the package manifest and specifies the format version (use 3 for ROS 2).
- `<name>my_package</name>`  
  The unique name of your package.
- `<version>0.0.1</version>`  
  The version number of your package.
- `<description>My custom ROS 2 package</description>`  
  A short description of what your package does.
- `<maintainer email="your.email@example.com">Your Name</maintainer>`  
  The maintainer's name and email address.
- `<license>Apache-2.0</license>`  
  The license under which your package is released.
- `<buildtool_depend>ament_cmake</buildtool_depend>`  
  Declares the build tool dependency (always needed for ROS 2 C++ packages).
- `<build_depend>rclcpp</build_depend>`  
  Declares a build dependency (needed to compile the code).
- `<build_depend>std_msgs</build_depend>`  
  Another build dependency.
- `<exec_depend>rclcpp</exec_depend>`  
  Declares a runtime dependency (needed when running the code).
- `<exec_depend>std_msgs</exec_depend>`  
  Another runtime dependency.
- `<export> ... </export>`  
  Section for additional information or plugins (can be left empty if not needed).
---

**In summary:**  
The `package.xml` file is essential for describing your ROS 2 package, managing dependencies, and ensuring proper integration with the

[Visit ROS 2 documentation to see how to creat a custom ros message](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

**Other Useful Link** 
[Building Robust Communication with Custom Messages](https://faun.pub/mastering-ros2-building-robust-communication-with-custom-messages-services-and-actions-9a49b4cd8b16)
[Create your own ROS2 custom message](https://roboticsbackend.com/ros2-create-custom-message/)
