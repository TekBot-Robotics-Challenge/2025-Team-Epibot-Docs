# Gazebo Simulation Setup for Tekbot

This documentation explains how to set up a basic Gazebo simulation using URDF and XACRO files for a robot called **Tekbot**, including wheels and a sample environment object (cylinder). It is designed for complete beginners using ROS 2 and Gazebo.

---

## Table of Contents

1. [Overview](#overview)
2. [Project Structure](#project-structure)
3. [URDF and XACRO Explanation](#urdf-and-xacro-explanation)
4. [Tekbot URDF](#tekboturdf)
5. [Wheel URDF Xacro](#wheelurdfxacro)
6. [Cylinder URDF Xacro](#cylinderurdfxacro)

---

## Overview

The goal of this setup is to simulate a mobile robot (Tekbot) in Gazebo using:

* URDF (Unified Robot Description Format) to describe the robot
* XACRO (XML Macros) to simplify repetitive URDF elements like wheels
* Gazebo plugins for simulation behavior

---

## Project Structure

```bash
tekbot_ws/
└── src/
    └── tekbot_description/
        ├── urdf/
        │   ├── tekbot.urdf
        │   ├── wheel.urdf.xacro
        │   └── cylinder.urdf.xacro
        ├── launch/
        │   └── gazebo.launch.py
        └── world/
            └── my_world.world
```

---

## URDF and XACRO Explanation

### `tekbot.urdf`

This is the **main file** that builds the robot by connecting multiple parts together.

### Purpose:
- Declares the robot name and base
- Loads the wheel models using macros
- Assembles all parts into one robot

### Structure:
1. **Robot Declaration**:
    ```xml
    <robot name="tekbot" xmlns:xacro="http://ros.org/wiki/xacro">
    ````

2. **Base Link**:

   * `base_footprint` is the main body of the robot. It serves as the central frame.
   * It is defined as a simple box or cylinder (optional visual/collision not shown in this file).

3. **Including the Wheels**:

   * Two wheels are loaded using the `wheel` macro from `wheel.urdf.xacro`
   * Example:

     ```xml
     <xacro:wheel name="left_wheel" x="0" y="0.1" z="0.03" yaw="0"/>
     <xacro:wheel name="right_wheel" x="0" y="-0.1" z="0.03" yaw="0"/>
     ```

4. **Joints**:

   * Joints are added to connect the wheels to the base.
   * They are **continuous joints**, allowing the wheels to rotate endlessly.
   * Positioned using the same coordinates defined for the wheels.

---

### `wheel.urdf.xacro`

This file defines a **reusable macro** to generate a wheel. It is used in `tekbot.urdf` to create the left and right wheels.

### Purpose:

* Allows customization of each wheel's position and name
* Reduces code duplication
* Wraps the actual wheel geometry in a macro

### Structure:

1. **Macro Declaration**:

   ```xml
   <xacro:macro name="wheel" params="name x y z yaw">
   ```

2. **Transform**:

   * A `<joint>` is defined to position the wheel relative to the base
   * Uses `<origin>` tag with `x`, `y`, `z`, and `yaw` as input parameters

3. **Wheel Link**:

   * Loads a cylinder shape by calling the `cylinder.urdf.xacro` macro
   * The `link name` is passed along with pose and size info

---

### `cylinder.urdf.xacro`


This file defines the **geometry of a cylinder** (used for the wheel body).

### Purpose:

* Reusable shape for cylindrical objects like wheels
* Abstracts out the URDF for a basic cylinder

### Structure:

1. **Macro Declaration**:

   ```xml
   <xacro:macro name="cylinder" params="name radius length">
   ```

2. **Link Block**:

   * Defines the `visual`, `collision`, and `inertial` properties
   * Cylinder is defined along the Z-axis

3. **Geometry**:

   ```xml
   <geometry>
     <cylinder radius="${radius}" length="${length}"/>
   </geometry>
   ```

4. **Material and Color**:

   * The cylinder is given a gray color using a `<material>` tag
