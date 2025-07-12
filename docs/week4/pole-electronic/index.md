# FINAL TEST - THE CONVEYOR: Documentation

In this documentation:

- [1. General Context](#1-general-context)
- [2. Description of components](#2-description-of-components)
- [3. Computer-Aided Design](#3-computer-aided-design)
- [4. Conveyor System Logic](#4-conveyor-system-logic)
- [5. The Arduino Code](#5-the-arduino-code)
- [6. Assembly of components](#6-assembly-of-components)
- [7. Testing the project](#7-testing-the-project)
- [8. Helpful Ressources](#8-helpful-ressources)

## 1. General Context

This test introduces us to a fun and innovative way to make a **7-segment display** using **servo motors** instead of lights (LEDs). Each segment (the bars that make up each digit) is moved by a small servo motor. The display shows the numbers from 0 to 9, then from 9 to 0, changing every second.

## 2. Description of components

### a. Arduino Nano

The [Arduino Nano](https://docs.arduino.cc/hardware/nano/) is a compact microcontroller board based on the ATmega328P, suitable for embedded applications and easy prototyping. It features 14 digital input/output pins (of which 6 can be used as PWM outputs), 8 analog inputs, a mini USB port for programming and power, and supports a voltage range of 7-12V input. Its small size (approximately 1.8 x 0.7 inches) and compatibility with the Arduino IDE make it ideal for DIY electronics, robotics, and IoT applications where space is limited and reliable performance is needed.

<p align="center">
    <img src="https://github.com/user-attachments/assets/82b01225-74db-4d1b-94b2-5a4e325868f8" width="500">
</p>

### b. Stepper Motor Nema 17

The [NEMA 17](https://pages.pbclinear.com/rs/909-BFY-775/images/Data-Sheet-Stepper-Motor-Support.pdf) stepper motor is a compact and commonly used motor in devices like 3D printers and CNC machines. It has a standard size, delivers precise movement, and is reliable for projects that need controlled motion. Its popularity comes from its balance of size, power, and ease of integration with various electronics.

<p align="center">
    <img src="https://github.com/user-attachments/assets/2af4bb93-6dbb-43fc-8ede-ca9ad9acb88e" width="500">
</p>

### c. Pololu A4988 Stepper Motor Driver

The [Pololu A4988](https://www.pololu.com/file/0j450/a4988_dmos_microstepping_driver_with_translator.pdf) is a microstepping driver for controlling bipolar stepper motors. It enables the Arduino to send signals for step and direction, translating them into precise motor motion. It features adjustable current control, over-temperature and over-current protection and also supports full, half, quarter, eighth, and sixteenth step modes.

<p align="center">
    <img src="https://github.com/user-attachments/assets/7ba5f45a-55bd-456e-ae1c-5ccd178bf0c3" width="500">
</p>

### d. KY-008 Laser Transmitter

The [KY-008](https://eclass.uth.gr/modules/document/file.php/E-CE_U_269/Sensors/Sensors_%20Datasheets/KY-008-AZ-Delivery.pdf) is a small laser emitter module used for creating a focused light beam. It features a 650nm red [laser diode](https://en.wikipedia.org/wiki/Laser_diode) that emits a focused beam, making it suitable for applications such as simple laser pointers, line detection, and light communication. Combined with a photoresistor, it can detect when an object passes through the beam (used as a tripwire sensor).

<p align="center">
    <img src="https://github.com/user-attachments/assets/0ccaf429-2387-4a03-8d5d-050d3f23fe32" width="500">
</p>

### e. Photoresistor

A [photoresistor](https://en.wikipedia.org/wiki/Photoresistor), or Light Dependent Resistor (LDR), changes its resistance based on light intensity. It’s used for detecting the presence or absence of objects, or measuring ambient light. It is typically use to detect when an object interrupts a light beam.

<p align="center">
    <img src="https://github.com/user-attachments/assets/9f7ca8d1-7d1e-4e5c-b276-5910bf914244" width="500">
</p>

### f. TCS34725 Color Sensor

The [TCS34725](https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf) is a digital color sensor that can detect RGB and clear light values. It’s used to identify the color of objects passing in front of it. It features onboard IR blocking filter, I2C interface and high sensitivity which enables for better precision.

<p align="center">
    <img src="https://github.com/user-attachments/assets/c4f0b689-3a66-4d25-8279-fc30652a6c73" width="500">
</p>

### g. Our custom power supply

To power up our setup, we should build a power supply that will provide safe and sufficient voltage to all the components. For that, we will use a pack of four 3.7V lithium batteries to supply a voltage of 14.8V.
<!-- ATTENTION ! -->

## 3. Computer-Aided Design

We used the KiCad EDA (_download [here](https://www.kicad.org/)_) to design the schematic as well as the PCB for this project. Find its official documentation [here](https://docs.kicad.org/).

To design the individual segments and model the housing that encloses all the circuitry, we used Solidworks (_download [here](https://www.solidworks.com/sw/support/downloads.htm)_). Learn more about this software [here](https://help.solidworks.com/).

### a. KiCad schematic diagram

The schematic is divided into two main blocks:

![The Schematic]()

### b. Printed Circuit Board (PCB) design

- PCB overview in the KiCad PCB editor

<p align="center">
    <img src="" width="1000">
</p>

_You can download the full KiCad project [here](https://github.com/TekBot-Robotics-Challenge/2025-Team-Epibot-Docs/raw/refs/heads/main/docs/week4/pole-electronic/designs/servo_display_kicad.zip)_.

## 4. Conveyor System Logic

#### Object Presence and Color Detection

- The KY-008 laser module emits a beam aimed at the photoresistor (LDR).
- When an object interrupts the beam, LDR detects a drop in light, signaling the Arduino that an object is present at the entry point.
- The Arduino reads color data from the TCS34725 color sensor to identify the color of the object.
- Through the serial monitor, the arduino sends a "GARBAGE COLOR" message to ROS to signal that an object is present and specify its color for tracking.

#### Positioning

- The Nema 17 stepper motor, controlled by the A4988 driver, moves the conveyor belt.
- The Arduino sends step and direction signals to the A4988 to rotate the motor and advance the belt.

#### Control Loop

- The Arduino receives input from the photoresistor (LDR) which helps to detect if an object is present/absent.
- When LDR detects an object, Arduino sends a signal to ROS specifying its color.
- ROS sends back a "MOTOR ON" message to Arduino, after which the conveyor advances the object to its end.
- ROS then sends a signal "MOTOR OFF" to signal that the conveyor should stop.
- Arduino receives this signal, the conveyor is stopped, and a "OK" confirmation message is sent back to ROS.

## 5. The Arduino Code

### Setting up

Download the Arduino IDE using this [link](https://www.arduino.cc/). It's a software that will allow you to run and upload your code to the MCUs. Once the installation is done, we can set up by installing the necessary libraries via the **Library Manager** in the Arduino IDE (_make sure that you also install their dependencies when prompted to_). We will need the following Arduino libraries:

```
/* ========= REQUIRED LIBRAIRIES ============== */
#include <Adafruit_TCS34725.h> // Library for our color sensor
#include <Wire.h>
```

### Global variables and constants declarations

- Constants representing the A4988 driver pins:

```
// === A4988 driver pins ===
#define stepPin 3 // connected to Arduino D3
#define dirPin 4 // connected to Arduino D4
```

- Constants representing the Photoresistors and Laser pins:

```
// === Photoresistors and Laser pins ===
#define laserPin 5 // KY-008 laser emitter
#define photo1 A0  // First photoresistor
#define photo2 A1  // Second photoresistor
```

- Photoresistor's resistance value threshold to detect the presence of an object

```
// Threshold to detect if an object is blocking the beam
const int detectionThreshold = 500;
```

- Declaration of TCS34725 color sensor: `Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);`

### setup() function

```
void setup()
{
  Serial.begin(115200); // start the serial monitor at 115200 bauds

  // Configuration for stepper motor
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, HIGH);

  // Configuration for Laser and photoresistors
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, HIGH); // Turn on laser

  // Initialization of our color sensor
  if (!tcs.begin()) // initialization failed
  {
    // print to the serial monitor for debugging
    Serial.println("Color sensor setup failed !");
    while (true); // Loop forever, don't proceed
  }
  if (!confirm_connect()) // ros and arduino connection failed
  {
    // print to the serial monitor for debugging
    Serial.println("Connection failure !");
    while (true); // Loop forever, don't proceed
  }
}
```

### Custom functions

#### Function to read from ROS using the serial monitor:

```
String read_from_ros()
{
  if (Serial.available()) // check if data is available for reading
  {
    return Serial.readString(); // return data as string
  }
  return ""; // return an empty string if no data is available
}
```

#### Function to confirm Arduino-ROS connection:

```
bool confirm_connect()
{
  String msg = read_from_ros(); // read from ros through serial

  // if connection with ROS is successful, send confirmation message to ros
  if (msg == "ROS CONNECTED\r\n")
  {
    // print to the serial monitor for arduino to ros communication
    Serial.println("ARDUINO CONNECTED");
    return true;
  }
  return false; // cannot confirm connection
}
```

#### Function to move the conveyor belt:

```
void startConveyor()
{
  String msg = read_from_ros(); // read from ros through serial
  int i = 0;

  // if ROS hasn't signaled to start conveyor, don't proceed
  if (msg != "MOTOR ON\r\n")
  {
    return;
  }
  while (i < 200) // Move 200 steps, partial rotation
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000); // control speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
    i++;
  }
  // Let ROS know object reached end of conveyor belt
  if (i == 200 - 1)
  {
    Serial.println("OK");
  }
}
```

#### Function to stop the conveyor belt:

```
void stopConveyor()
{
  // Just for logic
  // Motor stops itself automatically when no longer receiving pulses
}
```

#### Function for detecting an object's color through the TCS34725 color sensor:

```
// === Color detection via TCS34725 ===
String detectColor()
{
  // Read raw red, green, blue and clear values
  uint16_t red, green, blue, neutral;
  tcs.getRawData(&red, &green, &blue, &neutral);

  // Normalize color values for fair comparison
  float sum = red + green + blue;
  float R = red / sum;
  float G = green / sum;
  float B = blue / sum;

  // ======== Color dectection logic ========
  if (R > 0.35 && G > 0.35 && B < 0.25)
    return "YELLOW";
  else if (R > G && R > B)
    return "RED";
  else if (G > R && G > B)
    return "GREEN";
  else if (B > R && B > G)
    return "BLUE";
  else
    return "UNKNOWN"; // Color is not among the supported ones
}
```

### loop() function

```
void loop()
{
  // Read light levels from both photoresistors
  int value1 = analogRead(photo1); // read from first photoresistor
  int value2 = analogRead(photo2); // read from second photoresistor

  // Check if laser beam is blocked (object detected)
  if (value1 < detectionThreshold || value2 < detectionThreshold)
  {
    delay(800); // Time for object detection by color sensor
    String objectColor = detectColor(); // detect the object's color
    // print to the serial monitor for arduino to ros communication
    Serial.print("GARBAGE"); Serial.println(objectColor);
    startConveyor(); // move stepper motor
    stopConveyor();
    delay(1500); // 1.5 seconds pause before next detection
    return;
  }
  Serial.println("NONE");
}
```

_The full code is available to download [here](https://raw.githubusercontent.com/TekBot-Robotics-Challenge/2025-Team-Epibot-Docs/refs/heads/main/docs/week3/pole-electronic/code/servo_displayer.ino)_.

## 6. Assembly of components

This section outlines the assembly steps of all components for this project:

### Integrating the power supply

- Connect the four 3.7V lithium batteries in series to form a 14.8V pack.
- Use a voltage regulator to step down the voltage for the Arduino Nano (recommended: 7-12V).
- Distribute power: connect the battery pack to the A4988 Vmot/GND for the motor, and to the Arduino and sensors.
- Double-check battery polarity and connections to prevent damage.

<!-- image -->

### Wiring and soldering

- Plan the layout and cut wires to length needed for permanent installation.
- Solder the NEMA 17 stepper motor wires to the Pololu A4988 driver pins according to the datasheet.
- Connect the A4988 STEP and DIR pins to two digital pins on the Arduino Nano (in our case D3 and D4). Solder these connections directly.
- Wire the A4988 driver's logic power (VDD, GND) to the Arduino Nano's 5V and GND.
- Solder the KY-008 laser’s VCC and GND to the Arduino’s 5V and GND; if control is needed, connect its signal pin to a digital output.
- Mount and solder wires from the photoresistor to an analog input (in our case A0) and to GND and 5V as needed.
- Connect the TCS34725 color sensor’s SDA and SCL to the Arduino Nano’s I2C pins (A4, A5). Solder power and ground connections.
- Insulate all solder joints with heat-shrink tubing or electrical tape to prevent shorts.

<!-- image -->

### Mounting everything in the housing

- Secure all components to the housing using screws, standoffs, adhesive pads, or custom mounts as needed.
- Route and fix wires neatly along the housing, using cable ties, clips, or hot glue to prevent movement and interference with moving parts.
- Confirm all modules are firmly fixed and wires are clear of the conveyor mechanism.

<!-- image -->

### Component Placement

- Fix the NEMA 17 stepper motor securely to drive the conveyor belt.
- Position the Arduino Nano, Pololu A4988 driver, and voltage regulator inside the housing, ensuring accessibility for programming and maintenance.
- Mount the KY-008 laser so its beam crosses the conveyor belt, aligned with the photoresistor opposite for object detection.
- Place the TCS34725 color sensor above the conveyor belt, oriented to scan passing items.
- Ensure all components are spaced to avoid interference and allow for cooling and wire routing.

<!-- image -->

## 7. Testing and Validation

Before uploading the Arduino program, you need to do some final checks:

- Visually inspect and check all connections to prevent short circuits.
- Apply power gradually while monitoring the LEDs and voltage rails.

After performing these steps, upload [The Arduino Code](#5-the-arduino-code), and observe.

<!-- Find below a demonstration video of our own setup: -->

## 8. helpful Ressources

- [Download KiCad](https://www.kicad.org/)
- [Download SolidWorks](https://www.solidworks.com/sw/support/downloads.htm)
- https://www.youtube.com/watch?v=9qZUjEsVWts
- https://www.youtube.com/watch?v=WLVfZXxpHYI
- https://www.youtube.com/watch?v=lkyUqMVJBQ0
- https://docs.arduino.cc/learn/electronics/stepper-motors/
