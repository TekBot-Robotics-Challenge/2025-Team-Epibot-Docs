## 4. Conveyor System Logic

### Object Presence and Color Detection

- The KY-008 laser module emits a beam aimed at the photoresistor (LDR).
- When an object interrupts the beam, LDR detects a drop in light, signaling the Arduino that an object is present at the entry point.
- The Arduino reads color data from the TCS34725 color sensor to identify the color of the object.
- Through the serial monitor, the arduino sends a "GARBAGE COLOR" message to ROS to signal that an object is present and specify its color for tracking.

### Positioning

- The Nema 17 stepper motor, controlled by the A4988 driver, moves the conveyor belt.
- The Arduino sends step and direction signals to the A4988 to rotate the motor and advance the belt.

### Control Loop

- The Arduino receives input from the photoresistor (LDR) which helps to detect if an object is present/absent.
- When LDR detects an object, Arduino sends a signal to ROS specifying its color.
- ROS sends back a "MOTOR ON" message to Arduino, after which the conveyor advances the object to its end.
- ROS then sends a signal "MOTOR OFF" to signal that the conveyor should stop.
- Arduino receives this signal, the conveyor is stopped, and a "OK" confirmation message is sent back to ROS.
