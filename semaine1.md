# __TEST INPUT: DOCUMENTATION__

## General context

Understanding orientation in space, knowing what is left, right, up or down, is a fundamental skill acquired through our senses during childhood.

For robots, this spatial awareness must be replicated using sensors like gyroscopes and accelerometers. These devices convert physical data into electrical signals, enabling robots to measure velocity, rotation and orientation in space.

This test focuses on identifying and using a sensor that combines both the functions of a gyroscope and an accelerometer to determine spatial orientation and motion, which are essential for navigation and control.

## PART 1: Identifying components

### The MPU-6050: _gyroscope and accelerometer in one sensor_

The MPU-6050 is a sophisticated device that combines:

- a 3-axis gyroscope
- a 3-axis accelerometer
- a Digital Motion Processor (DMP)

This effectively makes the MPU-6050 the world's first 6-axis motion processing unit. It measures pitch (lateral axis rotation), roll (longitudinal axis rotation) and yaw (vertical axis rotation), which are fundamental for determining an object's orientation along the x, y and z axes.

#### How does it work ?

The MPU-6050 works by using tiny sensors inside it to detect movement and rotation in three directions. It can also be connected to an external compass to provide even more detailed motion tracking.

The MPU-6050 communicates with the Microcontroller Unit (MCU) using the Inter-Integrated Circuit (I2C) connection protocol with Serial Clock Line (SCL) and Serial Data Line (SDA) interfaces, which lets the devices exchange information easily. This setup allows the MPU-6050 to measure the speed, the rotation and the orientation of an object, all of which help the robot understand how it's moving in space. 

This sensor can also connect to other sensors (e.g: magnetic sensors) to track even more complex movements. It's designed to work with different voltage levels and can be used in systems where many sensors need to work together.
