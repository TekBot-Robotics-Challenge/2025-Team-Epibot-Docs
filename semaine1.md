# __TEST INPUT: DOCUMENTATION__

In this documentation:

- [General context](#general-context)
- [PART 1: Description of components](#part-1-description-of-main-components)
- [PART 2: Assembly of components](#part-2-assembly-of-components)
- [PART 3: Testing the project](#part-3-testing-the-project)
- [Helpful ressources you can consult](#helpful-ressources-you-can-consult)

## General context

Understanding orientation in space, **knowing what is left, right, up or down**, is a fundamental skill acquired through our senses during childhood.

For robots, this spatial awareness must be replicated **using sensors like gyroscopes and accelerometers**. These devices convert physical data into electrical signals, enabling robots to measure velocity, rotation and orientation in space.

This test focuses on identifying and using a sensor that **combines both the functions** of a gyroscope and an accelerometer to determine spatial orientation and motion, which are essential for navigation and control.

## PART 1: Description of components

### a. The MPU-6050: gyroscope and accelerometer in one sensor

The [MPU-6050](https://www.allelcoelec.com/blog/mpu-6050-in-action-practical-guide-to-setup%2Cconfiguration%2Cand-noise-management.html?srsltid=AfmBOooVL5CkBlAuw8WV0Yz7l7ZA2u8Ld6yBZFQf7kaTwiemNMmpUUey&utm_source=chatgpt.com#8.%20MPU-6050-Based%20Motion%20Trajectory%20Calculation) is a sophisticated device that combines:

- a 3-axis gyroscope
- a 3-axis accelerometer
- a Digital Motion Processor (DMP)

This effectively makes the MPU-6050 the world's first 6-axis motion processing unit. It measures pitch (lateral axis rotation), roll (longitudinal axis rotation) and yaw (vertical axis rotation), which are fundamental for determining an object's orientation along the x, y and z axes.

The MPU-6050 communicates with the [Microcontroller Unit](https://en.wikipedia.org/wiki/Microcontroller) (MCU) using the [Inter-Integrated Circuit](https://en.wikipedia.org/wiki/I%C2%B2C) (I2C) connection protocol through the Serial Clock Line (SCL) and Serial Data Line (SDA) wires, which lets the devices exchange information easily. This setup allows the MPU-6050 to measure the speed, the rotation and the orientation of an object, all of which help the robot understand how it's moving in space. 

<p align="center">
    <img src="https://github.com/user-attachments/assets/59d5bcdb-afe3-46cf-83fc-9d5e54b45559">
</p>

The MPU-6050 works by using tiny sensors inside it to detect movement and rotation _in three directions_. It can also be connected to an external compass to provide even more detailed motion tracking.

#### The gyroscope

The [gyroscope](https://en.wikipedia.org/wiki/Gyroscope) in the MPU-6050 _measures rotation_. This sensor operates on the principles of angular momentum, maintaining its orientation via the gyroscopic effect. This fascinating phenomenon allows the measurement of rotation direction and angles by identifying deviations from an initial axis.

#### The accelerometer

The [accelerometer](https://en.wikipedia.org/wiki/Accelerometer) in the MPU-6050 _measures acceleration_: how fast an object speeds up, slows down, or changes its direction. It uses the piezoelectric effect to evaluate acceleration forces, detecting the electrical charge produced by a moving object. In other terms, it works by detecting tiny forces inside the sensor when it moves.

#### The Digital Motion Processor (DMP)

The Digital Motion Processor in the MPU-6050 _processes_ the rotation data from the gyroscope and the movement data from the accelerometer through sophisticated algorithms such as Kalman filtering. The result is quaternions, a mathematical representation that blends rotational and translational data for in-depth motion analysis. The DMP also takes a load off the main processor, making the whole system faster and more efficient.

### b. The OLED screen: Visualize the data

The **SSD1306** is a popular and compact OLED display module used in a variety of projects to display text or graphics. It communicates with the microcontroller via the _I2C communication protocol_ (just like the MPU-6050 sensor), making it easy to integrate.

<p align="center">
    <img src="https://github.com/user-attachments/assets/7cfd0c36-6237-4052-9faf-cabdfcbb0275" width="500">
</p>

The SSD1306 is **ideal** for viewing the pitch, roll and yaw readings from the MPU-6050 sensor, with a simple, yet pleasing interface or menu. It also allows for real-time motion updates.

### c. The Arduino UNO: Central Control Unit

The [Arduino UNO](https://docs.arduino.cc/hardware/uno-rev3/) is a small electronic board that acts as the **"brain"** of the project. It can be programmed to control sensors, screens, motors, lights and much more. It acts as a basic computer that interacts with the physical world. These attributes of the Arduino make it perfect for this project, as it can read data from our MPU-6050 and output it on our SSD1306 screen while handling communication between all components using I2C.

<p align="center">
    <img src="https://github.com/user-attachments/assets/3198e572-b41c-4c76-8e33-dbfeda0f17e9">
</p>

### d. Creating our custom power supply

To further our study of the MPU-6050 sensor and to be able to test its functionning, we should build a small custom power supply that provides safe and stable voltage to all components. For that, we will use:

- The **Arduino UNO** as the main controller
- Our **MPU-6050 sensor**
- The **SSD10306 OLED display**
- A **tension regulator**
- A **9V electric battery** as our external power source

#### Here's a synoptic diagram to better illustrate it all:

<!-- <p align="center">
    <img src="https://github.com/user-attachments/assets/3198e572-b41c-4c76-8e33-dbfeda0f17e9">
</p> -->

## PART 2: Assembly of components

### a. The Arduino code

#### Fetching data from the MPU-6050 sensor

#### Outputting the readings onto the SSD1306 screen

### b. Joining the components

#### Connecting the SSD1306 to the Arduino UNO

<p align="center">
    <img src="https://github.com/user-attachments/assets/d6634e37-e542-4dfd-8961-57af85efb327">
</p>

#### Connecting the MPU-6050 to the Arduino UNO

<p align="center">
    <img src="https://github.com/user-attachments/assets/24712601-d2ee-4b3e-91b5-27eb747b680c" width="600">
</p>

## PART 3: Testing the project

Check the wiring again first,then place the setup flat in the palm of your hand.

- To test the pitch, tilt your hand up and down (like a head nod).
- To test the roll, tilt your hand sideways (like moving your head left and right).
- To test the yaw, spin your hand in place clockwise or counter-clockwise while keeping it level.

#### Here's a demonstration video

<!-- <p align="center">
    <a href="the video" target="_blank">
        <img src="the thumbnail" width="600">
    </a>
</p> -->

## Helpful ressources you can consult

- [Download KiCad](https://www.kicad.org/)
- [Download the Arduino IDE](https://www.arduino.cc/)
- https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/
- https://lastminuteengineers.com/oled-display-arduino-tutorial/
- https://en.wikipedia.org/wiki/
- https://www.instructables.com
- https://docs.arduino.cc/
- https://www.geeksforgeeks.org
