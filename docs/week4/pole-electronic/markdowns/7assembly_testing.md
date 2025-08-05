## 6. Assembly of components

This section outlines the assembly steps of all components for this project:

### Integrating the power supply

- Connect the four 3.7V lithium batteries in series to form a 14.8V pack.
- Use a voltage regulator to step down the voltage for the Arduino Nano (recommended: 7-12V).
- Distribute power: connect the battery pack to the A4988 Vmot/GND for the motor, and to the Arduino and sensors.
- Double-check battery polarity and connections to prevent damage.

<p align="center">
    <img src="https://github.com/user-attachments/assets/7fee4788-ca95-403d-85a5-5d567107520a" width="500">
</p>

### Wiring and soldering

- Plan the layout and cut wires to length needed for permanent installation.
- Solder the NEMA 17 stepper motor wires to the Pololu A4988 driver pins according to the datasheet.
- Connect the A4988 STEP and DIR pins to two digital pins on the Arduino Nano (in our case D3 and D4). Solder these connections directly.
- Wire the A4988 driver's logic power (VDD, GND) to the Arduino Nano's 5V and GND.
- Solder the KY-008 laser’s VCC and GND to the Arduino’s 5V and GND; if control is needed, connect its signal pin to a digital output.
- Mount and solder wires from the photoresistor to an analog input (in our case A0) and to GND and 5V as needed.
- Connect the TCS34725 color sensor’s SDA and SCL to the Arduino Nano’s I2C pins (A4, A5). Solder power and ground connections.
- Insulate all solder joints with heat-shrink tubing or electrical tape to prevent shorts.

<p align="center">
    <img src="https://github.com/user-attachments/assets/73e15dc2-9bbb-4cc0-9936-69c57a6d249d" width="500">
</p>

### Component Placement

- Fix the NEMA 17 stepper motor securely to drive the conveyor belt.
- Position the Arduino Nano, Pololu A4988 driver, and voltage regulator inside the housing, ensuring accessibility for programming and maintenance.
- Mount the KY-008 laser so its beam crosses the conveyor belt, aligned with the photoresistor opposite for object detection.
- Place the TCS34725 color sensor above the conveyor belt, oriented to scan passing items.
- Ensure all components are spaced to avoid interference and allow for cooling and wire routing.

### Mounting everything in the housing

- Secure all components to the housing using screws, standoffs, adhesive pads, or custom mounts as needed.
- Route and fix wires neatly along the housing, using cable ties, clips, or hot glue to prevent movement and interference with moving parts.
- Confirm all modules are firmly fixed and wires are clear of the conveyor mechanism.

<video src="/demo1.mp4" controls autoplay muted style="width: 100%; max-width: 800px; height: auto;">
  Your browser does not support the video tag.
</video>

## 7. Testing and Validation

Before uploading the Arduino program, you need to do some final checks:

- Visually inspect and check all connections to prevent short circuits.
- Apply power gradually while monitoring the LEDs and voltage rails.

After performing these steps, upload [The Arduino Code](#5-the-arduino-code), and observe.
