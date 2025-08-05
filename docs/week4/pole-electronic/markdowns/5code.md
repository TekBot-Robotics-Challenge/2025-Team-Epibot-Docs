# TRC2K25 - Automated Sorting System with Arduino

This project implements an Arduino-based automated sorting system, integrating a color sensor, a photoresistor laser, an ultrasonic sensor, a Nema17 stepper motor with an A4988 driver, and serial communication with ROS.

## Code Architecture

The code is divided into namespaces to organize the components:

* `MOTOR`: Stepper motor control.
* `SENSOR`: Sensor control.
* `TRC 2K25`: Sorting logic, ROS interface.

```
/*
** PROJECT: FINAL TEST - THE CONVEYOR
** TEAM: EPIBOT
*/

// ======= Include the necessary libraries =======

#include <Adafruit_TCS34725.h> // Library to manage the TCS34725 color sensor
#include <Wire.h> // Library for I2C communication (useful for the TCS34725)
#include <NewPing.h> // Library for the HC-SR04 ultrasonic sensor

// ======= NAMESPACE FOR MOTORS =======

namespace MOTOR
{
    // Base class for the A4988 driver
    class A4988
    {
    public:
        A4988(); // Constructor
        ~A4988() = default; // Default destructor
    protected:
        size_t stepPin; // STEP Pin
        size_t dirPin; // Pin DIR
        size_t stepTime; // Time between two steps (microseconds)
    };

    // Nema17 class inheriting from A4988
    class Nema17 : public A4988
    {
    public:
        Nema17(); // Constructor
        ~Nema17() = default; // Default destructor
        void start(); // Method to start the motor
    private:
        size_t nbStep; // Number of steps to take
    };
}

// Implementation of the A4988 constructor
MOTOR::A4988::A4988()
{
    stepPin = 2; // STEP pin set to D2
    dirPin = 3; // DIR pin set to D3
    stepTime = 10000; // 10 ms between each step
    pinMode(stepPin, OUTPUT); // Set the STEP pin to output
    pinMode(dirPin, OUTPUT); // Set the DIR pin to output
    return;
}

// Nema17 constructor implementation
MOTOR::Nema17::Nema17() : MOTOR::A4988()
{
    nbStep = 3; // The motor will take 3 steps each time it is activated
    return;
}

// Method to start the motor
void MOTOR::Nema17::start()
{
    for (size_t step{0}; step < nbStep; step++)
    {
        digitalWrite(stepPin, HIGH); // Sends a HIGH signal to start a step
        delayMicroseconds(stepTime); // Wait
        digitalWrite(stepPin, LOW); // Sends a LOW signal to end the step
        delayMicroseconds(stepTime); // Wait
    }
    return;
}

// ======= SENSOR NAMESPACE =======

namespace SENSOR
{
    // Class for the TCS34725 color sensor
    class TCS34725
    {
    public:
        TCS34725(); // Constructor
        ~TCS34725() = default; // Default destructor
        String detectColor(); // Returns a detected color as a string
    private:
        Adafruit_TCS34725 tcs; // Sensor instance
        uint16_t red, green, blue, neutral; // Raw data
        float sum, r, g, b; // Normalized values
    };

    // Class for the KY-008 laser
    class KY008
    {
    public:
        KY008(); // Constructor
        ~KY008() = default; // Default destructor
    private:
        size_t laserPin; // Laser pin
    };

    // Photoresistor class
    class Photoresistor
    {
    public:
        Photoresistor(); // Constructor
        ~Photoresistor() = default;
        bool isHit(); // Checks if the laser is interrupted
    private:
        int photoValue; // Value measured by the photoresistor
        int detectionThreshold; // Interrupt threshold
        size_t photoPin; // Analog read pin
        size_t thresholdPin; // Pin for threshold adjustable via potentiometer
    };

    // Ultrasonic sensor class
    class HCSR04
    {
    public:
        HCSR04(); // Constructor
        ~HCSR04(); // Destructor
        bool isObject(); // Returns true if an object is detected within 2 cm
    private:
        size_t trigPin; // Trigger pin
        size_t echoPin; // Receive Pin
        int distance; // Measured distance
        NewPing *sensorhcsr04; // Pointer to the NewPing object
    };
}

// ======= SENSOR IMPLEMENTATION =======

SENSOR::TCS34725::TCS34725()
{
    sum = r = g = b = 0.0; // Initialization
    tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // Sensor Configuration
    if (!tcs.begin()) // Check if the sensor is detected
        while (1)
            Serial.println("Error: TCS34725 not found.");
    return;
}

String SENSOR::TCS34725::detectColor()
{ 
    tcs.getRawData(&red, &green, &blue, &neutral); // Read RGB data 
    if (!neutral) return "NONE"; // Avoid division by 0 
    r = (float)red / neutral; 
    g = (float)green / neutral;b /= sum; 
    if (r > 0.40 && g > 0.40 && b < 0.25) 
        return "YELLOW"; 
    else if (r > 0.45 && g < 0.40 && b < 0.35) 
        return "RED"; 
    else if (g > 0.45 && r < 0.40 && b < 0.35) 
        return "GREEN"; 
    else if (b > 0.45 && r < 0.35 && g < 0.40) 
        return "BLUE"; 
    else 
        return "NONE";
}

// Initializing the KY-008 laser
SENSOR::KY008::KY008()
{ 
    laserPin = 4; // Pin D4 for the laser 
    pinMode(laserPin, OUTPUT); // Declare the output pin
    digitalWrite(laserPin, HIGH); // Enables the laser
    return;
}

// Photoresistor: constructor
SENSOR::Photoresistor::Photoresistor()
{
    photoValue = 0;
    detectionThreshold = 0;
    photoPin = A0;
    thresholdPin = A1;
}

// isHit method: Checks if the light is blocked
bool SENSOR::Photoresistor::isHit()
{
    photoValue = analogRead(photoPin); // Read the received light
    detectionThreshold = analogRead(thresholdPin); // Adjustable threshold
    return photoValue > detectionThreshold; // Interrupt detection
}

// Ultrasonic sensor constructor
SENSOR::HCSR04::HCSR04()
{
    echoPin = 5;
    trigPin = 6;
    sensorhcsr04 = new NewPing(trigPin, echoPin, 10); // Max range 10 cm
    distance = 0;
    return;
}

// Destructor: frees the sensor object
SENSOR::HCSR04::~HCSR04()
{
    delete sensorhcsr04;
    return;
}

// Detect a nearby object (max 2 cm)
bool SENSOR::HCSR04::isObject()
{
    distance = sensorhcsr04->ping_cm(); // Measurement
    return distance <= 2;
}

// ======= TRC2K25 SORTING SYSTEM NAMESPACE =======

TRC2K25 namespace
{
    class conveyor
    {
    public:
        conveyor(); // Constructor
        ~conveyor() = default;
        void confirmConnection(); // Verifies the ROS connection
        void start(); // Starts sorting
        String read(); // Serial read
        void Write(String message); // Serial write
    private:
        MOTOR::Nema17 nema17; // Motor instance
        SENSOR::KY008 ky008; // Laser instance
        SENSOR::TCS34725 tcs34725; // Color sensor
        SENSOR::Photoresistor photoresistor; // Photoresistance
        SENSOR::HCSR04 hcsr04; // Ultrasonic sensor
        String buffer; // Message buffer
        String objectColor; // Color detected
        bool isRunning; // System status
        bool isConfirm; // ROS connection confirmed
        bool isRosRunning; // ROS logic execution
    };
}

// Constructor Implementation
TRC2K25::conveyor::conveyor()
{
    isConfirm = false;
    isRosRunning = false;
    return;
}

// Check that ROS is connected
void TRC2K25::conveyor::confirmConnection()
{
    if (!isConfirm)
    {
        delay(5000); // Wait a bit before listening
        isConfirm = true;
        unsigned long start = millis();
        while (millis() - start < 5000)
        {
            buffer = this->read();
            if (!strncmp("ROS CONNECTED", buffer.c_str(), 13))
            {
                this->Write("ARDUINO CONNECTED");
                isRunning = true;
                return;
            }
        }
        isRunning = false;
        this->Write("CONNECTION FAILURE"); 
        return; 
    }
}

// Start conveyor logic
void TRC2K25::conveyor::start()
{ 
    if (isRunning && !photoresistor.isHit() && !isRosRunning) 
    { 
        isRosRunning = true; 
        this->Write("START"); 
    } 
    if (isRosRunning) 
    { 
        objectColor = tcs34725.detectColor(); // Color detection 
        if (objectColor != "NONE") 
            this->Write("GARBAGE " + objectColor); 
        buffer = this->read(); 
        if (!strncmp("MOTOR ON", buffer.c_str(), 8)) 
            nema17.start(); // Activate motor
        if (hcsr04.isObject())
            this->Write("END"); // Object detected at end
    }
    return;
}

// Serial reading
String TRC2K25::conveyor::read()
{
    if (Serial.available())
        return Serial.readString();
    return "";
}

// Serial writing
void TRC2K25::conveyor::Write(String message)
{
    Serial.println(message);
    return;
}

// ======= ARDUINO MAIN PROGRAM =======

void setup()
{
    Serial.begin(115200); // Start serial communication
    return;
}

void loop()
{
    static TRC2K25::conveyor conveyorEpibot; // Static Instance
    conveyorEpibot.confirmConnection(); // Confirms ROS
    conveyorEpibot.start(); // Starts sorting
    return;
}
```

## Hardware Setup

### 1. `MOTOR` — Nema17 Stepper Motor Control

#### `A4988` (base class)

* Initializes the `step` (D2) and `dir` (D3) pins.
* Sets the time between steps (`stepTime = 10000 µs`).

#### `Nema17` (inherits from `A4988`)

* Sets the number of steps to take (`nbStep = 3`).
* `start()` method to loop the motor.

---
### 2. `SENSOR` — Sensor Management

#### `TCS34725` (color sensor)

* Initializes the sensor via I2C.
* `detectColor()` method:

* Retrieves RGB + ambient light values.
* Normalizes and interprets colors: `RED`, `GREEN`, `BLUE`, `YELLOW`, or `NONE`.

#### `KY008` (laser)

* Activates a laser on pin D4.

#### `Photoresistor` (analog photoresistor)

* Inputs: `photoPin = A0`, `thresholdPin = A1`.
* `isHit()` method:

* Compares the detected light to an adjustable threshold (via potentiometer).
* Returns true if an object crosses the laser beam.

#### `HCSR04` (ultrasound)

* Uses the `NewPing` library.
* Pins: `trigPin = D6`, `echoPin = D5`.
* `isObject()` method:

* Returns true if an object is detected within 2 cm.
* The destructor dynamically releases the `NewPing` object.

### 3. `TRC2K25` — Sorting System Logic

#### `conveyor` (main class)

##### Attributes:

* Sensors: `TCS34725`, `KY008`, `Photoresistor`, `HCSR04`
* Motor: `Nema17`
* Buffers: `buffer`, `objectColor`
* States: `isRunning`, `isConfirm`, `isRosRunning`

##### Methods:

* `confirmConnection()`:

* Waits for a `"ROS CONNECTED"` message for 5 seconds.
* Responds with `"ARDUINO CONNECTED"` or `"CONNECTION FAILURE"`.

* `start()`:

* If ready and the laser detects an object: Sends `START` to ROS.
* Detects the color and sends `GARBAGE [COLOR]`.
* If ROS sends `MOTOR ON`: Activates the motor.
* If HC-SR04 detects a nearby object: Sends `END`.
* `stop()`: Not used here.
* `read()`: Reads a serial message.
* `Write(message)`: Writes a serial message.

## Arduino Main Loop

```cpp
void setup()
{
    Serial.begin(115200);
}
```

* Initializes serial communication.

```cpp
void loop()
{
    static TRC2K25::conveyor conveyorEpibot;
    conveyorEpibot.confirmConnection();
    conveyorEpibot.start();
    conveyorEpibot.stop();
}
```

* Continuous system execution:

1. Verifies the ROS connection.
2. Executes the sorting logic.
3. (Stop not used.)

## Hardware Required

| Module | Purpose | Arduino Pins |
| --------------------- | ----------------------------------- | ---------------------- |
| Adafruit TCS34725 | Color Detection | I2C (A4/A5) |
| KY-008 Laser | Emits a light beam | D4 |
| Photoresistor + pot | Laser interruption detection | A0 (photo), A1 (threshold) |
| HC-SR04 + NewPing | Final distance measurement (end of sort) | Trig = D6, Echo = D5 |
| Nema17 + A4988 | Object Transport | Dir = D3, Step = D2 |
| Arduino (ATmega328P) | Central Control | UART for ROS, etc. |

## Upload Code to Custom Arduino Board

1. Connect your board via USB (CH340G recommended).
2. Select the correct board: **"Arduino Uno"**.
3. Select the correct serial port.
4. Click **"Upload"**.

*If you used an ATmega328P + CH340G bootloader, the upload will be performed as on a standard Uno.*

## Operation Summary

```plaintext
ROS ←→ Arduino
|
|→ Laser + Photoresistor → Object detected
|→ TCS34725 → Color detected
|→ Nema17 → Transport
|→ HC-SR04 → Limit switch
```

## Serial Exchange Example

| ROS Message | Arduino Response | Action |
| ----------------- | ----------------- | ------------------------- |
| ROS CONNECTED | ARDUINO CONNECTED | System Init |
| (after detection) | START | Input object detected |
| | GARBAGE RED | Detected object is red |
| MOTOR ON | (activates motor) | Object advance |
| | END | Object has reached the end |
