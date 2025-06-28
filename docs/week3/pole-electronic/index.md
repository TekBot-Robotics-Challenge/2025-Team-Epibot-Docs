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
    <img src="https://github.com/user-attachments/assets/4577bfe3-11f7-4e74-b1ea-8e482174593a" width="500">
</p>

### c. The PCA-9685: servomotor controller

The [PCA-9685](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf) is an integrated circuit (IC) used as a 16-channel [Pulse Width Modulation](https://en.wikipedia.org/wiki/Pulse-width_modulation) (PWM) driver, often controlled via the I2C communication protocol. It allows you to control up to sixteen servos or LEDs independently from just two microcontroller pins. The PCA9685 is widely used in robotics and automation projects to expand the number of devices you can control, especially useful when you need to manage many servos or LEDs at the same time.  

<p align="center">
    <img src="https://github.com/user-attachments/assets/a77edaf7-42a2-4912-9f70-08218d011634" width="500">
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

### f. A 16Mhz Quartz Crystal

A 16MHz [quartz crystal](https://en.wikipedia.org/wiki/Crystal_oscillator) is an electronic component used to provide a precise clock signal for microcontrollers and other digital circuits. By vibrating at a stable frequency of 16,000,000 cycles per second, it ensures accurate timing for operations such as processing instructions and generating communication signals.

<p align="center">
    <img src="https://github.com/user-attachments/assets/6bd9e058-17b3-477a-9b86-afcf6be05a07" width="500">
</p>

### g. Our custom power supply

To power up our setup, we should build a power supply that will provide safe and sufficient voltage to all the components. For that, we will use:

- 3.7V lithium batteries
- a zener diode for 5V logic regulation
- a LM-7908 tension regulator to bring the voltage to 9V
- 22 uF and 5V/9V capacitors

## 3. Computer-Aided Design

We used the KiCad EDA (_download [here](https://www.kicad.org/)_) to design the schematic as well as the PCB for this project. Find its official documentation [here](https://docs.kicad.org/).

To design the individual segments and model the housing that encloses all the circuitry, we used Solidworks (_download [here](https://www.solidworks.com/sw/support/downloads.htm)_). Learn more about this software [here](https://help.solidworks.com/).

### a. KiCad schematic diagram

The schematic is divided into two main blocks:

1. **Custom Arduino-compatible microcontroller circuit**
  - The ATmega328P is powered by a regulated 5V supply, generated via:
    - The LM-7809 which receives an external 14.8V and outputs 9V
    - The Zener diode which stabilizes voltage down to 5V for logic
  - Two voltage outputs:
    - 9V (red LED monitored)
    - 5V (green LED monitored)
  - 16 MHz quartz for clock, 22 µF capacitors for stability
  - A button for manual reset

2. **PCA-9685 PWM driver with the 7 servos**
  - The PCA-9685 generates PWM for the 7 servos
  - Connections:
    - GND: system ground
    - VCC: 5V logic supply
    - V++: 9V servo supply
    - SDA: connected to ATmega328P A4
    - SCL: connected to ATmega328P A5
  - Servos: PWM signal from PCA9685, powered via V++ (9V) and GND

![The Schematic](https://github.com/user-attachments/assets/465c3744-cb84-4d03-b9ab-aaabe32dcd35)

### b. Printed Circuit Board (PCB) design

- PCB overview in the KiCad PCB editor

<p align="center">
    <img src="https://github.com/user-attachments/assets/93f0245d-49ef-4dda-88de-4c7fb8671d1c" width="700">
</p>
<!-- - PCB 3D visualization -->
<!-- _You can download the full KiCad project [here](https://github.com/kkbroxane/2025-Team-Epibot-Docs/raw/main/docs/week2/pole-electronic/schematics/black_box_cube_schematics.zip)_. -->
<!-- ### c. Segments and housing design -->

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
  {1, 1, 1, 1, 0, 0, 1}, // digit 3
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
static bool rewind = false; // display
```
- Digit displaying logic

```
  if (currentMillis - previousMillis >= interval) // ensure 1 second delay
  {
    previousMillis = currentMillis; // Save current time
    // print to the serial monitor for debugging
    Serial.print("==="); Serial.print(currentDigit); Serial.println("===");
    // display the current digit
    displayDigit(digitPositions[currentDigit]);

    if (currentDigit >= 9)
      rewind = true; // start counting backwards
    if (!rewind)
      currentDigit++;
    if (rewind && currentDigit > 0)
      currentDigit--;
    if (rewind && !currentDigit)
      rewind = false; // start counting from zero again
  }
```

_The full code is available to download [here]()_.

## 6. Assembly of components

The assembly process was carried out on a veroboard instead of a custom PCB, allowing for easy and flexible adjustments during construction. 

### Component Placement

We start by planning where each component will go on the veroboard. We then place the ATmega328P microcontroller, PCA9685 PWM module, voltage regulator (LM7809 or LM-1950), zener diode, capacitors, and LEDs in order to minimize wire crossings and make the layout neat and logical. Finally, we insert the header pins for easy connection of servomotors and for accessing the microcontroller’s I/O pins for testing or expansion.

### Integrating the power supply

- Connect your 3.7V lithium batteries to the input side of the power regulation circuit.
- Mount the LM-7809  on the veroboard and add the zener diode plus capacitors around it for filtering and stability.
- Set up two separate power rails on the veroboard: one for 5V (logic, microcontroller, PCA9685) and one for 9V (servomotors).
- Solder red and green LEDs to your rails as power indicators, so you can easily see when each voltage is present.

### Wiring and soldering

- Use a small drill bit or a sharp knife to carefully break copper traces on the veroboard where necessary, preventing unwanted connections.
- Solder wires to link the I2C lines (SDA and SCL) from the ATmega328P to the corresponding pins on the PCA9685.
- Connect each servomotor’s control wire to its dedicated PWM channel on the PCA9685, and hook up their power (to the 9V rail) and ground lines.
- Double-check all connections for accuracy before soldering to ensure everything matches your schematic.

### Mounting everything in the housing

- Place the finished veroboard and all connected modules into the housing.
- Secure the servomotors so that each one lines up with the correct segment of the display and can move freely.
- Route the servo wires and any additional connectors to be accessible from outside the housing.

## 7. Testing and Validation

Before uploading the Arduino program, you need to do some final checks:

- Visually inspect and check all connections to prevent short circuits.
- Apply power gradually while monitoring the LEDs and voltage rails.

After performing these steps, upload [The Arduino Code](#5-the-arduino-code), and observe.

<!-- Find below a demonstration video of our own setup: -->

## 8. helpful Ressources

- [Download KiCad](https://www.kicad.org/)
- [Download SolidWorks](https://www.solidworks.com/sw/support/downloads.htm)
- https://www.circuits-diy.com/12v-to-9v-converter-circuit-using-lm7809-regulator-ic/
- https://www.ic-components.fr/blog/zener-diode-circuit-design,techniques-and-tips.jsp
- https://fr.hwlibre.com/Guide-complet-du-contr%C3%B4leur-PCA9685-avec-Arduino-et-plus/
- https://learn.sparkfun.com/tutorials/hobby-servo-tutorial/all
- https://www.fs-pcba.com/fr/fabrication-dun-circuit-imprime-etape-par-etape/
- https://youtu.be/uDUp4cLXFrY?si=2rA-MZsFL_sJVutS
- https://github.com/sdouze/Atmega-Standalone
- https://www.instructables.com/Mastering-Servo-Control-With-PCA9685-and-Arduino/
- https://learn.sparkfun.com/tutorials/crystal-oscillators/all
