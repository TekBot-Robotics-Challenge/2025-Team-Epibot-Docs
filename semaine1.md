# __TEST INPUT: DOCUMENTATION__

In this documentation:

- [General context](#general-context)
- [PART 1: Description of components](#part-1-description-of-components)
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

- a 3-axis [gyroscope](https://en.wikipedia.org/wiki/Gyroscope)
- a 3-axis [accelerometer](https://en.wikipedia.org/wiki/Accelerometer)
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

The [accelerometer](https://en.wikipedia.org/wiki/Accelerometer) in the MPU-6050 _measures acceleration_: how fast an object speeds up, slows down, or changes its direction. It uses the [piezoelectric effect](https://en.wikipedia.org/wiki/Piezoelectricity) to evaluate acceleration forces, detecting the electrical charge produced by a moving object. In other terms, it works by detecting tiny forces inside the sensor when it moves.

#### The Digital Motion Processor (DMP)

The Digital Motion Processor in the MPU-6050 _processes_ the rotation data from the gyroscope and the movement data from the accelerometer through sophisticated algorithms such as [Kalman filtering](https://en.wikipedia.org/wiki/Kalman_filter). The result is [quaternions](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation), a mathematical representation that blends rotational and translational data for in-depth motion analysis. The DMP also takes a load off the main processor, making the whole system faster and more efficient.

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

```
    [9V Battery]
         |
         v
    [Tension Regulator] ------> [5V] --------------------------+
             |                                                 |
         +--------------------- GND -----------------------+----+
                                                           |    |
                                                           |    |
             +---------------------------+                 |    |
             |       [Arduino Uno]       |                 |    |
             |  [5V]---------------------+-----------------+    |
             |  [GND]--------------------+----------------------+ 
             |  [SDA] <-------+          |
             |  [SCL] <-------+          |
             +---------------------------+
                    ^           ^
                    |           |
             +------+           +------+
             |                         |
       [SSD1306 OLED]             [MPU-6050]
    (I2C: SDA, SCL, 5V, GND)  (I2C: SDA, SCL, 5V, GND)
```

## PART 2: Assembly of components

### a. The Arduino code

Download the Arduino IDE using this [link](https://www.arduino.cc/). Once the installation is done, we can set up by installing the necessary libraries via the **Library Manager** in the Arduino IDE (_make sure that you also install their dependencies when prompted to_). We will need the following Arduino libraries:

- The **Wire** library for I2C communication (required for the MPU-6050 and the SSD1306)
  ```
  #include <Wire.h>
  ```
- The **Adafruit SSD1306** library for managing our SSD1306 module
  ```
  #include <Adafruit_SSD1306.h>
  #include <Adafruit_GFX.h>  // Required for graphics
  ```
- The **Adafruit MPU6050** library for managing
  ```
  #include <Adafruit_MPU6050.h>
  #include <Adafruit_Sensor.h>  // Required for sensor data structures
  ```
Next, in the **setup()** function, we call `Serial.begin(115200);` to start the _serial monitor_ at 115200 [bauds](https://en.wikipedia.org/wiki/Baud), and `Wire.begin();` to start the I2C bus.

#### Fetching data from the MPU-6050 sensor

We first need to initialize the MPU-6050 sensor. For that, in the **setup()** function, we call `mpu.initialize();`. 

In the **loop()** function, we do this for reading raw data from our MPU-6050 sensor:

```
int16_t ax, ay, az;
mpu.getAcceleration(&ax, &ay, &az);
```
This fetched data represents the object's acceleration following the X, Y and Z axes. We will then need to perform a conversion of this data to **m/s²**. From that, we can then estimate the **total acceleration** and **rotation** by using some trigonometrical formulas.

#### Outputting the readings onto the SSD1306 screen

We first need to declare our SSD1306 display.

```
#define SCREEN_WIDTH 128           // in pixels
#define SCREEN_HEIGHT 64           // in pixels
#define SCREEN_ADDRESS 0x3C        // Standard I2C address for SSD1306
#define OLED_RESET -1              // No reset pin used with I2C

// ---------- Display declaration ----------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
```
We then initialize the SSD1306 OLED module in the **setup()** function by doing:
`display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)`. 

Although optional, we decided to display our school's logo at the screen's startup. You can choose to also display an image at startup. For that, you will need to convert your image to a 1-bit format, since the SSD1306 OLED employs the monochrome theme. You can use this [tool](https://javl.github.io/image2cpp/) to generate a 1-bit map for your image. You will only need to upload your image file and adjust the image settings. You can then use `display.drawBitmap(0, 0, myLogo, 128, 64, WHITE); /` to draw your image from your generated 1-bit map to the screen.

To print the readings from our sensor to the screen, we use `splay.clearDisplay();` to first clear the screen buffer, then `display.print()` to write to the screen and finally `display.display();` to display everything.

### b. Joining the components

#### Connecting the SSD1306 to the Arduino UNO

<p align="center">
    <img src="https://github.com/user-attachments/assets/d6634e37-e542-4dfd-8961-57af85efb327" width="600">
</p>

#### Connecting the MPU-6050 to the Arduino UNO

<p align="center">
    <img src="https://github.com/user-attachments/assets/24712601-d2ee-4b3e-91b5-27eb747b680c" width="600">
</p>

## PART 3: Testing the project

Check the wiring again first, then place the setup flat in the palm of your hand.

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
