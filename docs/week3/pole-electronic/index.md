# TEST OUTPUT: Documentation

In this documentation:

- [1. General Context](#1-general-context)
- [2. Description of components](#2-description-of-components)
- [3. Computer-Aided Design](#3-computer-aided-design)
- [4. 7-Segment Display Logic](#4-7-segment-display-logic)
- [5. The Arduino Code](#5-the-arduino-code)
- [6. Assembly of components](#6-assembly-of-components)
- [7. Testing the project](#7-testing-the-project)
- [8. Helpful Ressources](#8-helpful-ressources)

## 1. General Context

This test introduces us to a fun and innovative way to make a **7-segment display** using **servo motors** instead of lights (LEDs). Each segment (the bars that make up each digit) is moved by a small servo motor. The display shows the numbers from 0 to 9, then from 9 to 0, changing every second.

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

### b. A servomotor

A [servomotor](https://en.wikipedia.org/wiki/Servomotor) is a small device used to precisely control the position of objects, commonly found in robotics, model airplanes, and remote-controlled cars. It consists of a motor, a position sensor, and a control circuit, allowing it to rotate or move to a specific angle when given an electronic signal. Servo motors are popular in projects that require accurate and repeatable movements, such as animating displays or actuating robotic arms.

<p align="center">
    <img src="" width="500">
</p>

### c. The PCA-9685: servomotor controller

The [PCA-9685](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf) is an integrated circuit (IC) used as a 16-channel [Pulse Width Modulation](https://en.wikipedia.org/wiki/Pulse-width_modulation) (PWM) driver, often controlled via the I2C communication protocol. It allows you to control up to sixteen servos or LEDs independently from just two microcontroller pins. The PCA9685 is widely used in robotics and automation projects to expand the number of devices you can control, especially useful when you need to manage many servos or LEDs at the same time.  

<p align="center">
    <img src="" width="500">
</p>

### d. The Zener Diode

The [Zener Diode](https://en.wikipedia.org/wiki/Zener_diode) is a special type of semiconductor diode that is designed to allow current to flow in the reverse direction when the voltage across it reaches a specific value, called the Zener breakdown voltage. What this means is that a Zener diode can maintain a constant voltage in a circuit, even if the input voltage changes. When the voltage across the Zener diode exceeds its breakdown voltage, it starts conducting in reverse and keeps the voltage steady, making it very useful for voltage regulation and protecting sensitive electronic components from voltage spikes.

<p align="center">
    <img src="https://github.com/user-attachments/assets/ff801b0f-7cb2-41a0-b1c1-73a898b3c432" width="500">
</p>

### e. The LM-1950: a voltage regulator

The [LM-1950](https://www.alldatasheet.com/datasheet-pdf/view/125278/NSC/LM1950.html) is a [voltage regulator](https://en.wikipedia.org/wiki/Voltage_regulator). Its purpose to keep the voltage at a constant level. It uses a negative feedback system to monitor and adjust the output, ensuring stable voltage even when the input voltage or load conditions change. As an electronic component, it is built using integrated circuitry rather than mechanical parts. The LM-1950 is intended for regulating DC voltages, and can be used to supply a steady voltage to one or more DC-powered devices.

<p align="center">
    <img src="https://github.com/user-attachments/assets/27ee9377-598c-49ce-bd85-e65c67a19a62" width="500">
</p>

### f. A 16Mhz Quartz Crystal

A 16MHz [quartz crystal](https://en.wikipedia.org/wiki/Crystal_oscillator) is an electronic component used to provide a precise clock signal for microcontrollers and other digital circuits. By vibrating at a stable frequency of 16,000,000 cycles per second, it ensures accurate timing for operations such as processing instructions and generating communication signals.

<p align="center">
    <img src="" width="500">
</p>

### g. Our custom power supply

To power up our setup, we should build a power supply that will provide safe and sufficient voltage to all the components. For that, we will use:

- 3.7V lithium batteries
- a zener diode to stabilize the voltage at 5V
- a LM-1950 tension regulator to bring the voltage to 9V
- 22 uF and 5V/9V capacitors

## 3. Computer-Aided Design

### a. KiCad schematic diagrams

We used the KiCad EDA (_download [here](https://www.kicad.org/)_) to design and document the schematic for this project. KiCad is a powerful, open-source Electronic Design Automation (EDA) suite that enables users to create professional-quality schematics and printed circuit boards. It offers a comprehensive set of tools for circuit design, simulation, and layout, making it ideal for both hobbyists and professionals. For more detailed information about using KiCad and its features, you can refer to the [official documentation](https://docs.kicad.org/).

Find below, the KiCad schematic diagram for this project. It provides clear illustrations of how the components are connected to each other.

![The cube](https://github.com/user-attachments/assets/5b2bef66-c82e-46cd-beb2-565d138a38dd)

### b. Printed Circuit Board (PCB) design

We also used the KiCad EDA to design and layout the printed circuit board (PCB) for this project. Below, you will find the PCB images as designed in KiCad:

- PCB overview in the KiCad PCB editor

<p align="center">
    <img src="https://github.com/user-attachments/assets/beb9c3ef-64f3-4b76-a476-d52fb612c29d" width="500">
</p>

<!-- - PCB Interactive 3D view -->
<!-- _You can download the Cube's KiCad files [here](https://github.com/kkbroxane/2025-Team-Epibot-Docs/raw/main/docs/week2/pole-electronic/schematics/black_box_cube_schematics.zip)_. -->

### c. Design of the 7 segments

For the cube design, we used Autodesk Fusion 360 (_download [here](https://www.autodesk.com/products/fusion-360/download)_) to design and model the components for this project. Fusion 360 is a comprehensive, cloud-based platform that integrates design, engineering, and manufacturing into a single tool. It offers powerful features for parametric modeling, assembly creation, simulation, and detailed rendering, making it ideal for both prototyping and final product development. To learn more about this tool, refer to its [official documentation](https://help.autodesk.com/view/fusion360/ENU/). Our cube has a side height of _7 centimeters_ with one face open to visualize the internal circuitry.

## 4. 7-Segment Servo Display Logic

A 7-segment servo display consists of seven individual segments labeled from **A** through **G** (_held each by a servo_), arranged to form the numbers 0 through 9 by selectively rising or laying down specific segments.

The logic for controlling a 7-segment servo display is typically summarized in a truth table, where each digit from 0 to 9 corresponds to a unique combination of segments that should be activated. For example, to display the digit '8', all seven segments are **turned on**, while to display **'1'**, only segments **'b'** and **'c'** are activated.

Below is a segment code table for digits from 0 to 9:

```
| Digit | A | B | C | D | E | F | G |
|-------|---|---|---|---|---|---|---|
|   0   | 1 | 1 | 1 | 1 | 1 | 1 | 0 |
|   1   | 0 | 1 | 1 | 0 | 0 | 0 | 0 |
|   2   | 1 | 1 | 0 | 1 | 1 | 0 | 1 |
|   3   | 1 | 1 | 1 | 1 | 0 | 0 | 1 |
|   4   | 0 | 1 | 1 | 0 | 0 | 1 | 1 |
|   5   | 1 | 0 | 1 | 1 | 0 | 1 | 1 |
|   6   | 1 | 0 | 1 | 1 | 1 | 1 | 1 |
|   7   | 1 | 1 | 1 | 0 | 0 | 0 | 0 |
|   8   | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
|   9   | 1 | 1 | 1 | 1 | 0 | 1 | 1 |

```

In this project, a logical '1' means the servo should move to reveal the segment, and a '0' means it should hide the segment.

## 5. The Arduino Code

### Setting up

Download the Arduino IDE using this [link](https://www.arduino.cc/). It's a software that will allow you to run and upload your code to the MCUs. Once the installation is done, we can set up by installing the necessary libraries via the **Library Manager** in the Arduino IDE (_make sure that you also install their dependencies when prompted to_). We will need the following Arduino libraries:

```
/* ========= REQUIRED LIBRAIRIES ============== */
#include <Wire.h>                     // Enables I2C communication
#include <Adafruit_PWMServoDriver.h>  // Control the PCA-9685
```

### Global variables and constants declarations

- Creating a PWM controller for the PCA-9685 board: `Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();`

- Constants for minimum and maximum pulse width values used to control the position of a servomotor.

```
#define pulseMin 125 // 0 degree pulse
#define pulseMax 625 // 180 degrees pulse
```
- Number of servos: `#define numberOfServos 7`

- The segment code table, as shown above, will be implemented in code as an array of booleans, where each digit corresponds to a specific combination of servo positions (_0 or false for OFF, 1 or true for ON_).

```
const bool digitPositions[10][7] =
{
  {1, 1, 1, 1, 1, 1, 0}, // digit 0
  {0, 1, 1, 0, 0, 0, 0}, // digit 1
  {1, 1, 0, 1, 1, 0, 1}, // digit 2
  {1, 1, 1, 1, 0, 0, 0}, // digit 3
  {0, 1, 1, 0, 0, 1, 1}, // digit 4
  {1, 0, 1, 1, 0, 1, 1}, // digit 5
  {1, 0, 1, 1, 1, 1, 1}, // digit 6
  {1, 1, 1, 0, 0, 0, 0}, // digit 7
  {1, 1, 1, 1, 1, 1, 1}, // digit 8
  {1, 1, 1, 1, 0, 1, 1}, // digit 9
};
```
- Variables for non-blocking delay handling using `millis()`:

```
unsigned long previousMillis = 0;
const long interval = 1000; // Interval of 1 second
```
- Constants for ON/OFF servo positions

```
#define UP 90 // segment up position angle (active state)
#define FLAT 180 // segment flat position angle (non-active state)
```
- Global variable to track current digit: `unsigned int currentDigit = 0;`

### setup() function

```
void setup()
{
  Serial.begin(115200); // Start the serial monitor at 115200 bauds
  pwm.begin(); // Start the PWM controller
  pwm.setPWMFreq(60); // Set servo frequency at 60 Hz
  // put all servo in non-active mode at the start
  const bool startPos[7] = {0, 0, 0, 0, 0, 0, 0};
  displayDigit(startPos);
}
```

### Custom functions

- Function to move a servomotor to the desired angle

```
void moveServo(int servoNumber, int angle)
{
  int pulse = map(angle, UP, FLAT, pulseMin, pulseMax); // convert angle to pulse
  pwm.setPWM(servoNumber, 0, pulse); // apply pulse to servo
}
```

- Function for displaying digits based on the segment code table

```
void displayDigit(const bool array[7])
{
  for (int i = 0; i < numberOfServos; i++)
  {
    if (array[i])
      moveServo(i, UP); // move segment up
    else
      moveServo(i, FLAT); // lay segment flat
  }
}
```

### loop() function

- Variables:

```
unsigned long currentMillis = millis(); // Get elapsed time
// control digit order: if false, go from 0 to 9, else, go from 9 to 0
bool rewind = false; // display
```
- Digit displaying logic

```
  // ensure 1 second delay
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis; // Save current time
    if (currentDigit > 9)
      rewind = true; // display digits backwards
    if (!rewind)
      currentDigit++; // increment if not yet reached 9
    else
      currentDigit--; // decrement if already reached 9
    displayDigit(digitPositions[currentDigit]);
  }
```

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
- https://www.circuits-diy.com/12v-to-9v-converter-circuit-using-lm7809-regulator-ic/
- https://www.ic-components.fr/blog/zener-diode-circuit-design,techniques-and-tips.jsp
- https://fr.hwlibre.com/Guide-complet-du-contr%C3%B4leur-PCA9685-avec-Arduino-et-plus/
- https://learn.sparkfun.com/tutorials/hobby-servo-tutorial/all
- https://www.fs-pcba.com/fr/fabrication-dun-circuit-imprime-etape-par-etape/
- https://youtu.be/uDUp4cLXFrY?si=2rA-MZsFL_sJVutS
- https://github.com/sdouze/Atmega-Standalone
- https://www.instructables.com/Mastering-Servo-Control-With-PCA9685-and-Arduino/
- https://learn.sparkfun.com/tutorials/crystal-oscillators/all