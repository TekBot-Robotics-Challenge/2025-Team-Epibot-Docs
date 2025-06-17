# TEST 2 - THE BLACK BOX: Documentation

In this documentation:

- [1. General Context](#1-general-context)
- [2. Description of components](#2-description-of-components)
- [3. Computer-Aided Design](#3-computer-aided-design)
- [4. Computer-Aided Manufacturing](#4-computer-aided-manufacturing)
- [5. Embedded Software and Microcontroller Logic](#5-embedded-software-and-microcontroller-logic)
- [6. The arduino code](#6-the-arduino-code)
- [7. Assembly of components](#7-assembly-of-components)
- [8. Testing the project](#8-testing-the-project)
- [9. Helpful Ressources](#9-helpful-Ressources)

## 1. General Context

Black boxes are critical devices commonly used in fields such as aviation, automotive, and rail transport to record and monitor key operational data from equipment. By continuously collecting information like speed, position, and orientation, these systems help in understanding how machines behave and can provide useful insights in case of incidents or for performance improvement. The goal here is to develop a compact black box capable of capturing motion data, including speed and position, using a gyroscope and accelerometer sensor.

This second test of the Tekbot Robotics Challenge introduces the design and implementation of a simple embedded black box system. The goal here is to develop a compact black box capable of capturing motion data, including speed and position, using a gyroscope and accelerometer sensor. The collected data should be sent in real-time to a control station, where it can be displayed and analyzed. The project provides hands-on experience with embedded systems, data transmission, and hardware-software integration.

### Objectives of the test

- **Accurately capture and record motion data:** Measure and record the speed and position of the cube as it moves, using sensor input.
- **Enable real-time data transmission:** Continuously send the collected data to the control station without delay.
- **Facilitate clear and immediate data visualization:** Present the motion information on a display so users can quickly understand the cube’s movements.
- **Demonstrate system reliability and robustness:** Show that the system works smoothly and provides dependable results during testing.
- **Encourage hands-on integration of skills:** Offer an opportunity to integrate knowledge of embedded systems, sensor data handling, and system integration through a hands-on project.

## 2. Description of components <!-- add real images -->

### a. The ATmega328P microcontroller

The [ATmega328P](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf) is an 8-bit [microcontroller](www.geeksforgeeks.org/digital-logic/microcontroller-and-its-types/) chip that serves as the **central processing unit** in many electronic devices. It is based on the [AVR architecture](https://en.wikipedia.org/wiki/AVR_microcontrollers) and is designed for tasks that involve reading sensor data, controlling outputs like LEDs or motors, and communicating or exchanging information with other devices. It is the chip used in the popular [Arduino UNO board](https://docs.arduino.cc/hardware/uno-rev3/).

#### KEY FEATURES

- **CPU:** 8-bit AVR architecture running at up to 20 MHz (handles calculations and instructions quickly and efficiently)
- **Memory:** Flash Memory (32 KB), SRAM (2 KB), EEPROM (1 KB) for data and main program storing
- **GPIO Pins:** 23 programmable input/output pins for reading signals
- **Timers:** Three built-in timers (two 8-bit, one 16-bit)
- **Analog Inputs:** 6 pins for reading changing electrical signals
- **Communication interfaces:** Supports USART (serial comminication), I2C and SPI protocols
- **PWM:** 6 channels for Pulse Width Modulation
- **Operating Voltage:**  Works with power supplies from 1.8V to 5.5V
- **Low Power Modes:** Multiple sleep modes for energy efficiency

### b. The MPU-6050: gyroscope and accelerometer in one sensor

The [MPU-6050](https://www.allelcoelec.com/blog/mpu-6050-in-action-practical-guide-to-setup%2Cconfiguration%2Cand-noise-management.html?srsltid=AfmBOooVL5CkBlAuw8WV0Yz7l7ZA2u8Ld6yBZFQf7kaTwiemNMmpUUey&utm_source=chatgpt.com#8.%20MPU-6050-Based%20Motion%20Trajectory%20Calculation) is a sensor module that combines a 3-axis gyroscope and a 3-axis accelerometer, enabling the detection of both rotational movement and linear acceleration in three dimensions. It features a Digital Motion Processor (DMP), which processes the raw sensor data using built-in algorithms to deliver reliable information on speed, rotation, and orientation. Communication with microcontrollers is accomplished via the I2C protocol, allowing for smooth integration into embedded systems. With its ability to provide real-time, accurate measurements while minimizing the processing load on the main system, the MPU-6050 is well suited for applications that require motion tracking and analysis.

### c. The LCD screen

The [Liquid-Crystal Display](https://en.wikipedia.org/wiki/Liquid-crystal_display) (LCD) is a type of screen found in many electronic devices, like calculators, clocks, and TVs. It works by using a thin layer of liquid crystals placed between two filters. These crystals can change how light passes through them when  electricity is applied. The LCD itself does not create light. Instead, a light behind the screen (called a backlight) shines through the crystals. By controlling the crystals with electrical signals, the LCD can show different shapes, letters, or pictures in black and white or in color.

### d. The Zener Diode

The [Zener Diode](https://en.wikipedia.org/wiki/Zener_diode) is a special type of semiconductor diode that is designed to allow current to flow in the reverse direction when the voltage across it reaches a specific value, called the Zener breakdown voltage. What this means is that a Zener diode can maintain a constant voltage in a circuit, even if the input voltage changes. When the voltage across the Zener diode exceeds its breakdown voltage, it starts conducting in reverse and keeps the voltage steady, making it very useful for voltage regulation and protecting sensitive electronic components from voltage spikes.

### e. The tension regulator

<!--
LM-1950 -- takes a gt 9v to output a 9v

1N4733A diode zener -- stabilize the tension and brings it to 5V

4 batteries lithium -- power up 14.8v

mpu-6050 -- min 3.3v max 5v

lcd screen -- min 3.3v max 5v
-->

### f. The Cube (black box)

For this test, the black box will be represented by a **cube**. It will be mainly composed of the MPU-6050 sensor and the ATmega328P chip mounted on a Printed Circuit Board (PCB). The main functions of the cube will be the acquisition of movement data and the transmission of this data to the Control Station via an I2C bus.

### g. The Control Station

The Control Station's main tasks are the processing and displaying of the data it receives from **the cube**. It will be composed of another ATmega328P chip mounted on a dedicated PCB and the LCD screen.

### h. Our custom power supply

To power up our setup, we should build a power supply that is independent from the cube and that will provide safe and sufficient voltage to all the components. For that, we will use:

<!-- list it all here -->

## 3. Computer-Aided Design
<!-- Setting up with KiCad -- installation, finding components, adding them -->

### a. KiCad schematic diagrams
<!-- quick explanation, adding image -->

### b. Printed Circuit Board (PCB) design
<!-- quick explanation and video -- add 3D visu -->

### c. The Cube design
<!-- quick explanation and video -- add 3D visu -->

## 4. Computer-Aided Manufacturing

## 5. Microcontroller Logic on the I2C bus

### a. What is I2C ?

The [Inter-Integrated Circuit](https://en.wikipedia.org/wiki/I%C2%B2C) (I2C), is a communication protocol that allows multiple electronic components, like sensors and microcontrollers, to talk to each other using just two wires: one for data (SDA) and one for the clock signal (SCL). With I2C, a single device acts as the controller (or master) that manages the communication, while other devices act as followers (or slaves). Each device on the I2C bus has a unique address, so the controller can talk to each one individually, even though they share the same wires. This makes I2C a simple and efficient way to connect many devices together in a circuit, saving wiring and making communication between parts much easier.

### b. Microcontroller configuration <!-- and sensor data acquisition/processing -->

The microcontroller configuration is one of the core aspects of this test. It ensures seamless communication between components, accurate data collection from sensors, and reliable transmission of information to the control station, forming the foundation of the system’s functionality.

In this project, two microcontrollers ATmega328P are used.

- The **Cube microcontroller** is configured as the I2C master. It is responsible for collecting motion data from the MPU-6050 sensor (which acts as an I2C slave), processing this data, and sending it to the control station.
- The **Control station microcontroller** is also configured as an I2C slave, listening for incoming data from the cube.

The LCD screen is directly connected to the **Control station microcontroller**, and is therefore an I2c slave. If additional sensors or peripherals were to be added (perhaps more sensors), they could also be configured as I2C slaves.

This master-slave structure is necessary because it guarantees that only one device (the master) controls the communication, avoiding conflicts and ensuring that messages are sent and received in an orderly and predictable way. In summary, the master-slave relationship organizes the roles and responsibilities of each device, making the system both scalable and reliable.

### c. Sensor Data Acquisition and Processing through the I2C bus

Accurate acquisition and processing of sensor data are essential for monitoring the motion and orientation of the cube. The sensor used in this project is a combined gyroscope and accelerometer: the MPU6050, which communicates with the cube’s microcontroller via the I2C bus.

#### Acquisition Steps:

- The microcontroller initializes the sensor by setting up communication parameters and, if necessary, calibrating the sensor to minimize measurement errors.
- At regular intervals, the microcontroller requests raw data from the sensor, including acceleration on each axis (X, Y, Z) and angular velocity (rotation rates) on each axis.
- The I2C protocol ensures that the sensor can reliably respond to requests from the master microcontroller without data collisions.

#### Processing Steps:

- The raw sensor readings are converted into meaningful physical units (e.g., acceleration in g’s or m/s², angular velocity in °/s).
- Processing algorithms, such as filtering (e.g., moving average or complementary filter), are applied to reduce noise _(small, unwanted variations or disturbances in the sensor data that do not represent the actual movement of the cube)_ and improve data stability.
- The processed sensor data is then formatted for transmission to the control station over the I2C bus and for display on the LCD.

This systematic approach to data acquisition and processing enables the system to provide real-time, accurate feedback on the cube’s speed and position, which is crucial for the core functionality of the project.

<!-- +----------------------------------------------------------------------+ -->

## 6. The arduino code
<!-- - black box
- station -->

## 7. Assembly of components
<!-- - wiring and connections
- Integration of custom power supply
- Main steps for assembly -->

## 8. Testing and Validation
<!-- - Testing procedures
- Demonstration video // vimeo -->

## 9. helpful Ressources

- https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP/?queryID=undefined
- https://www.fs-pcba.com/fr/fabrication-dun-circuit-imprime-etape-par-etape/
- https://youtu.be/uDUp4cLXFrY?si=2rA-MZsFL_sJVutS
- https://github.com/sdouze/Atmega-Standalone
