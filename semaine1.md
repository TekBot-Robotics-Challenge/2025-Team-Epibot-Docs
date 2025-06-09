# __TEST INPUT: DOCUMENTATION__

In this documentation:

- General context of the test
- Description of main components
- Assembly of components
- Helpful ressources used for this project and its documentation

## General context

Understanding orientation in space, knowing what is left, right, up or down, is a fundamental skill acquired through our senses during childhood.

For robots, this spatial awareness must be replicated using sensors like gyroscopes and accelerometers. These devices convert physical data into electrical signals, enabling robots to measure velocity, rotation and orientation in space.

This test focuses on identifying and using a sensor that combines both the functions of a gyroscope and an accelerometer to determine spatial orientation and motion, which are essential for navigation and control.

## PART 1: Description of main components

### a. The MPU-6050: gyroscope and accelerometer in one sensor

The MPU-6050 is a sophisticated device that combines:

- a 3-axis gyroscope
- a 3-axis accelerometer
- a Digital Motion Processor (DMP)

This effectively makes the MPU-6050 the world's first 6-axis motion processing unit. It measures pitch (lateral axis rotation), roll (longitudinal axis rotation) and yaw (vertical axis rotation), which are fundamental for determining an object's orientation along the x, y and z axes.

<p align="center">
    <img src="https://github.com/user-attachments/assets/59d5bcdb-afe3-46cf-83fc-9d5e54b45559">
    <em>MPU-6050 sensor
</p>

<!-- ![MPU-6050 sensor](https://github.com/user-attachments/assets/59d5bcdb-afe3-46cf-83fc-9d5e54b45559) -->

<!-- add def of I2C, gyroscope, accelerometer, SCL, SDA -->
The MPU-6050 works by using tiny sensors inside it to detect movement and rotation in three directions. It can also be connected to an external compass to provide even more detailed motion tracking.

The MPU-6050 communicates with the Microcontroller Unit (MCU) using the Inter-Integrated Circuit (I2C) connection protocol through the Serial Clock Line (SCL) and Serial Data Line (SDA) wires, which lets the devices exchange information easily. This setup allows the MPU-6050 to measure the speed, the rotation and the orientation of an object, all of which help the robot understand how it's moving in space. 

This sensor can also connect to other sensors (e.g: magnetic sensors) to track even more complex movements. It's designed to work with different voltage levels and can be used in systems where many sensors need to work together.

### b. Other main components 

#### The OLED screen: Visualize the data

The SSD1306 is a popular and compact OLED display module used in a variety of projects to display text or graphics. It communicates with the microcontroller via the I2C communication protocol (just like the MPU-6050 sensor), making it easy to integrate.

<p align="center">
    <img src="https://github.com/user-attachments/assets/3198e572-b41c-4c76-8e33-dbfeda0f17e9" width="500">
    <em>SSD1306 screen
</p>

<!-- ![SSD1306 OLED screen](https://github.com/user-attachments/assets/7cfd0c36-6237-4052-9faf-cabdfcbb0275) -->

The SSD1306 is ideal for viewing the pitch, roll and yaw readings from the MPU-6050 sensor, with a simple, yet pleasing interface or menu. It also allows for real-time motion updates.

#### The Arduino board: Central Control Unit

The Arduino is a small electronic board that acts as the "brain" of the project. It can be programmed to control sensors, screens, motors, lights and much more. It acts as a basic computer that interacts with the physical world. These attributes of the Arduino make it perfect for this project, as it can read data from our MPU-6050 and output it on our SSD1306 screen while handling communication between all components using I2C.

<p align="center">
    <img src="https://github.com/user-attachments/assets/3198e572-b41c-4c76-8e33-dbfeda0f17e9">
    <em>The Arduino Board
</p>
<!-- ![The Arduino Board](https://github.com/user-attachments/assets/3198e572-b41c-4c76-8e33-dbfeda0f17e9) -->

## PART 2: Assembly of components

### a. The Arduino code

#### Fetching data from the MPU-6050 sensor

#### Outputting the readings onto the SSD1306 screen

### b. Creating our custom power supply
