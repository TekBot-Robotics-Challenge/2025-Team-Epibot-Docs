# TEST 2 - THE BLACK BOX: Documentation

In this documentation:

- [1. General Context](#1-general-context)
- [2. Description of components](#2-description-of-components)
- [3. Computer-Aided Design](#3-computer-aided-design)
- [4. Embedded Software and Microcontroller Logic](#4-embedded-software-and-microcontroller-logic)
- [5. Programming the circuits](#5-programming-the-circuits)
- [6. Assembly of components](#6-assembly-of-components)
- [7. Testing the project](#7-testing-the-project)
- [8. Helpful Ressources](#8-helpful-ressources)

## 1. General Context

Black boxes are critical devices commonly used in fields such as aviation, automotive, and rail transport to record and monitor key operational data from equipment. By continuously collecting information like speed, position, and orientation, these systems help in understanding how machines behave and can provide useful insights in case of incidents or for performance improvement. The goal here is to develop a compact black box capable of capturing motion data, including speed and position, using a gyroscope and accelerometer sensor.

This second test of the Tekbot Robotics Challenge introduces the design and implementation of a simple embedded black box system. The goal here is to develop a compact black box capable of capturing motion data, including speed and posit+ion, using a gyroscope and accelerometer sensor. The collected data should be sent in real-time to a control station, where it can be displayed and analyzed. The project provides hands-on experience with embedded systems, data transmission, and hardware-software integration.

### Objectives of the test

- **Accurately capture and record motion data:** Measure and record the speed and position of the cube as it moves, using sensor input.
- **Enable real-time data transmission:** Continuously send the collected data to the control station without delay.
- **Facilitate clear and immediate data visualization:** Present the motion information on a display so users can quickly understand the cube’s movements.
- **Demonstrate system reliability and robustness:** Show that the system works smoothly and provides dependable results during testing.
- **Encourage hands-on integration of skills:** Offer an opportunity to integrate knowledge of embedded systems, sensor data handling, and system integration through a hands-on project.

## 2. Description of components

### a. The ATmega328P microcontroller

The [ATmega328P](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf) is an 8-bit [microcontroller](https://www.geeksforgeeks.org/digital-logic/microcontroller-and-its-types/) chip that serves as the **central processing unit** in many electronic devices. It is based on the [AVR architecture](https://en.wikipedia.org/wiki/AVR_microcontrollers) and is designed for tasks that involve reading sensor data, controlling outputs like LEDs or motors, and communicating or exchanging information with other devices. It is the chip used in the popular [Arduino UNO board](https://docs.arduino.cc/hardware/uno-rev3/).

<p align="center">
    <img src="https://github.com/user-attachments/assets/403a51b1-f52f-4d34-8fca-dbdd6fee4f12" width="500">
</p>

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

<p align="center">
    <img src="https://github.com/user-attachments/assets/d76a29bc-0cd0-4bb3-a4a2-e1efaf1bdac3" width="500">
</p>

### c. The LCD screen

The [Liquid-Crystal Display](https://en.wikipedia.org/wiki/Liquid-crystal_display) (LCD) is a type of screen found in many electronic devices, like calculators, clocks, and TVs. It works by using a thin layer of liquid crystals placed between two filters. These crystals can change how light passes through them when  electricity is applied. The LCD itself does not create light. Instead, a light behind the screen (called a backlight) shines through the crystals. By controlling the crystals with electrical signals, the LCD can show different shapes, letters, or pictures in black and white or in color.

<p align="center">
    <img src="https://github.com/user-attachments/assets/d16d5dc1-02c4-456c-839f-8a997d4395e6" width="500">
</p>

### d. The Zener Diode

The [Zener Diode](https://en.wikipedia.org/wiki/Zener_diode) is a special type of semiconductor diode that is designed to allow current to flow in the reverse direction when the voltage across it reaches a specific value, called the Zener breakdown voltage. What this means is that a Zener diode can maintain a constant voltage in a circuit, even if the input voltage changes. When the voltage across the Zener diode exceeds its breakdown voltage, it starts conducting in reverse and keeps the voltage steady, making it very useful for voltage regulation and protecting sensitive electronic components from voltage spikes.

<p align="center">
    <img src="https://github.com/user-attachments/assets/ff801b0f-7cb2-41a0-b1c1-73a898b3c432" width="500">
</p>

### e. The LM-7908: a voltage regulator

The [LM-7908](https://www.alldatasheet.com/datasheet-pdf/view/125278/NSC/LM1950.html) is a [voltage regulator](https://en.wikipedia.org/wiki/Voltage_regulator). Its purpose to keep the voltage at a constant level. It uses a negative feedback system to monitor and adjust the output, ensuring stable voltage even when the input voltage or load conditions change.

<p align="center">
    <img src="https://github.com/user-attachments/assets/20bb4d2a-3de9-4176-aa23-6ea9f7a78bbe" width="500">
</p>

### f. The Cube (black box)

For this test, the black box will be represented by a **cube**. It will be mainly composed of the MPU-6050 sensor and the ATmega328P chip mounted on a Printed Circuit Board (PCB). The main functions of the cube will be the acquisition of movement data and the transmission of this data to the Control Station via an I2C bus.

### g. The Control Station

The Control Station's main tasks are the processing and displaying of the data it receives from **the cube**. It will be composed of another ATmega328P chip mounted on a dedicated PCB and the LCD screen.

### h. Our custom power supply

To power up our setup, we should build a power supply that is independent from the cube and that will provide safe and sufficient voltage to all the components. For that, we will use:

- 4 lithium batteries to supply 14.8V
- a zener diode to stabilize the voltage at 5V for the control station
- a LM-7809 to bring the voltage to 9V for the black box

## 3. Computer-Aided Design

### a. KiCad schematic diagrams

We used the KiCad EDA (_download [here](https://www.kicad.org/)_) to design and document the schematic for this project. KiCad is a powerful, open-source Electronic Design Automation (EDA) suite that enables users to create professional-quality schematics and printed circuit boards. It offers a comprehensive set of tools for circuit design, simulation, and layout, making it ideal for both hobbyists and professionals. For more detailed information about using KiCad and its features, you can refer to the [official documentation](https://docs.kicad.org/).

Find below, the KiCad schematics diagram for this project. They provide clear illustrations of how the components are connected to each other in The Cube and in The Control Station.

#### The Cube (black box)

![The cube](https://github.com/user-attachments/assets/5b2bef66-c82e-46cd-beb2-565d138a38dd)

#### The Control station

![The control station](https://github.com/user-attachments/assets/293dc6e6-418b-4cd1-a4d3-5b1472e13e56)

### b. Printed Circuit Board (PCB) design

We also used the KiCad EDA to design and layout the printed circuit board (PCB) for this project. Below, you will find the finalized PCB images as designed in KiCad:

#### The Cube (black box)

- PCB overview in the KiCad PCB editor

<p align="center">
    <img src="https://github.com/user-attachments/assets/beb9c3ef-64f3-4b76-a476-d52fb612c29d" width="500">
</p>

- PCB 3D view


- Real-life realisation of The Cube's PCB

<p align="center">
    <img src="https://github.com/user-attachments/assets/63a6d940-3927-44fc-a894-c0a368d5f875" width="500">
</p>

_You can download the Cube's KiCad files [here](https://github.com/kkbroxane/2025-Team-Epibot-Docs/raw/main/docs/week2/pole-electronic/schematics/black_box_cube_schematics.zip)_.

#### The Control station

- PCB overview in the KiCad PCB editor

<p align="center">
    <img src="https://github.com/user-attachments/assets/df219587-fc33-4f1d-80de-9f16135cf56e" width="500">
</p>

- PCB 3D view


- Real-life realisation of the Control Station's PCB

<p align="center">
    <img src="https://github.com/user-attachments/assets/f61a21b9-9a01-428f-96a9-60c13011716b" width="500">
</p>

_You can download the Control station's KiCad files [here](https://github.com/kkbroxane/2025-Team-Epibot-Docs/raw/main/docs/week2/pole-electronic/schematics/control_station_schematics.zip)_.

### c. The Cube design

For the cube design, we used Autodesk Fusion 360 (_download [here](https://www.autodesk.com/products/fusion-360/download)_) to design and model the components for this project. Fusion 360 is a comprehensive, cloud-based platform that integrates design, engineering, and manufacturing into a single tool. It offers powerful features for parametric modeling, assembly creation, simulation, and detailed rendering, making it ideal for both prototyping and final product development. To learn more about this tool, refer to its [official documentation](https://help.autodesk.com/view/fusion360/ENU/). Our cube has a side height of _7 centimeters_ with one face open to visualize the internal circuitry.

## 4. Microcontroller Logic on the I2C bus

### a. What is I2C ?

The [Inter-Integrated Circuit](https://en.wikipedia.org/wiki/I%C2%B2C) (I2C), is a communication protocol that allows multiple electronic components, like sensors and microcontrollers, to talk to each other using just two wires: one for data (SDA) and one for the clock signal (SCL). With I2C, a single device acts as the controller (or master) that manages the communication, while other devices act as followers (or slaves). Each device on the I2C bus has a unique address, so the controller can talk to each one individually, even though they share the same wires. This makes I2C a simple and efficient way to connect many devices together in a circuit, saving wiring and making communication between parts much easier.

### b. Microcontroller configuration

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
- Processing algorithms, such as filtering (e.g., moving average or complementary filter), are applied to reduce noise (_small, unwanted variations or disturbances in the sensor data that do not represent the actual movement of the cube_) and improve data stability.
- The processed sensor data is then formatted for transmission to the control station over the I2C bus and for display on the LCD.

This systematic approach to data acquisition and processing enables the system to provide real-time, accurate feedback on the cube’s speed and position, which is crucial for the core functionality of the project.

## 5. Programming the circuits

After the wiring is done, we need to program both the ATmega328P chips from the Cube and The control station.

Download the Arduino IDE using this [link](https://www.arduino.cc/). It's a software that will allow you to run and upload your code to the MCUs. Once the installation is done, we can set up by installing the necessary libraries via the **Library Manager** in the Arduino IDE (_make sure that you also install their dependencies when prompted to_). We will need the following Arduino libraries:

- The **Wire** library for I2C communication

  ```
  #include <Wire.h>
  ```
- The **LiquidCrystal** library for managing our LCD screen

  ```
  #include <LiquidCrystal.h>
  ```
- The **MPU6050** library for managing the sensor

  ```
  #include <MPU6050.h>
  ```

### Fetching data from the MPU-6050 sensor in The Cube

In the **setup()** function, we call `Serial.begin(9600);` to start the _serial monitor_ at 9600 [bauds](https://en.wikipedia.org/wiki/Baud), and `Wire.begin();` to start the I2C bus as master.

We first need to initialize the MPU-6050 sensor. For that, in the **setup()** function, we call `mpu.initialize();`. 

In the **loop()** function, we do this for reading raw data from our MPU-6050 sensor:

```
int16_t ax, ay, az;
mpu.getAcceleration(&ax, &ay, &az);
```

We then need to send this fetched data to the slave (microcontroller in the Control Station) through the I2C bus:

```
  Wire.beginTransmission(slave_address); // start i2c data transfer
  // ====== Write the data to the i2c bus ======
  Wire.write((byte)(aX >> 8));
  Wire.write((byte)aX);
  Wire.write((byte)(aY >> 8));
  Wire.write((byte)aY);
  Wire.write((byte)(aZ >> 8));
  Wire.write((byte)aZ);
  Wire.endTransmission(); // stop data transfer

```

_Download the full code for the black box [here](https://github.com/kkbroxane/2025-Team-Epibot-Docs/raw/main/docs/week2/pole-electronic/code/the_cube_box.ino)_.

### Outputting from Control Station

We first need to declare our declare our LCD display.

```
// The LCD declaration with connected pin numbers as parameters
LiquidCrystal lcd(18, 16, 23, 24, 25, 3);
```

We then initialize the LCD module in the **setup()** function by doing:
`lcd.begin(16, 2);         // Initialize LCD`. 


`Wire.onReceive(receiveEvent);` enables us to read data as it becomes available on the i2c bus using a custom function **receiveEvent**:

```
  index = 0; // Global variable to track number of bytes read (volatile uint8_t index = 0)

  // start reading and storing data as soon as it is sent
  while (Wire.available() && index < 6)
  {
    buffer[index++] = Wire.read(); // read bytes from the i2c bus
  }
  //  
  if (index == 6) {
    ax = (buffer[0] << 8) | buffer[1];
    ay = (buffer[2] << 8) | buffer[3];
    az = (buffer[4] << 8) | buffer[5];
  }
```

To print the readings from our sensor to the screen, we use `lcd.clear();` to first clear the screen buffer, then `lcd.setCursor()` to adjust the cursor's position on the screen, and finally `lcd.print();` to write to the screen.

_Download the full code for the black box [here](https://github.com/kkbroxane/2025-Team-Epibot-Docs/raw/main/docs/week2/pole-electronic/code/the_control_station.ino)_.

## 6. Assembly of components
<!-- - wiring and connections
- Integration of custom power supply
- Main steps for assembly -->

## 7. Testing and Validation
<!-- - Testing procedures
- Demonstration video // vimeo -->

## 8. helpful Ressources

- [Download KiCad](https://www.kicad.org/)
- [Download Autodesk Fusion 360](https://www.autodesk.com/products/fusion-360/download)
- https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP/?queryID=undefined
- https://www.fs-pcba.com/fr/fabrication-dun-circuit-imprime-etape-par-etape/
- https://youtu.be/uDUp4cLXFrY?si=2rA-MZsFL_sJVutS
- https://github.com/sdouze/Atmega-Standalone
