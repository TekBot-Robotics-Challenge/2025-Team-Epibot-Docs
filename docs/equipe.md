# Epibot Team

---

<video src="/videos/trailer.mp4" controls style="width: 100%; max-width: 350px; height: auto;">
  Votre navigateur ne supporte pas la balise vidéo.
</video>

Our multidisciplinary team brings together complementary talents to tackle this 5-week challenge. Each member contributes their unique expertise while collaborating closely across all poles.

---

## IT Pole

### Mission

The main mission of the IT sub-team is to develop intelligent and modular software solutions that enable autonomous behavior, real-time decision-making, and seamless communication between hardware components and user interfaces. These systems must evolve across the selection tests and culminate in an integrated solution for the final conveyor belt triage challenge.

<h2 style="font-size: 24px; font-weight: bold; color: #949CDF;">
  Expertise
</h2>  

The IT sub-team must demonstrate expertise in the following areas:

<p style="font-size: 14px; font-weight: bold;">
  Object-Oriented Programming (OOP)
</p>

  - Designing flexible, reusable, and extensible class structures in C++ or Python.<br>
  - Using encapsulation, inheritance, and polymorphism effectively.

<p style="font-size: 14px; font-weight: bold;">
  ROS2 (Robot Operating System 2)
</p>

  - Building modular robotic systems using nodes, topics, services, and actions.<br>
  - Implementing publishers and subscribers for sensor data evaluation.<br>
  - Writing launch files for system-level execution.

<p style="font-size: 14px; font-weight: bold;">
  Pathfinding Algorithms
</p>

  - Developing and integrating navigation algorithms such as A*, Dijkstra, RRT, etc.<br>
  - Simulating robot movement in Gazebo and visualizing paths in RViz2.<br>
  - Managing obstacle avoidance and dynamic path planning.

<p style="font-size: 14px; font-weight: bold;">
  Embedded Systems Integration
</p>

  - Communicating with microcontrollers.<br>
  - Processing sensor inputs and triggering actuators through logic decisions.

<p style="font-size: 14px; font-weight: bold;">
  Web Interface Development
</p>

  - Creating intuitive dashboards to display real-time data.<br>
  - Integrating backend logic with frontend visualization.

<p style="font-size: 14px; font-weight: bold;">
  System Documentation
</p>

  - Maintaining GitHub repositories with clear commit history.<br>
  - Providing UML diagrams, technical explanations, and simulation results.

<h2 style="font-size: 24px; font-weight: bold; color: #949CDF;">
  Challenge Goals
</h2>

### Test 1: Robot Class Management
  
#### Objective:

Create a base Robot class with two derived subclasses, each redefining a virtual move() method.

#### Deliverables:

- OOP structure with encapsulation (getters/setters)
- Inheritance and polymorphism demonstrated
- UML diagram showing class relationships
- Clean, well-documented code in C++ or Python

---

### Test 2: Introduction to ROS2
  
#### Objective:

Build a ROS2 package with a node that generates random sensor data and another that validates it.

#### Deliverables:

- ROS2 package named sensor_data_evaluation
- Publisher node sending data (temperature, humidity, pressure) every 0.5s
- Subscriber node validating data within expected ranges
- Launch file to start both nodes simultaneously
- Proper documentation and logging

### Test 3: Pathfinding Algorithm
  
#### Objective:

Implement a pathfinding algorithm (A*, Dijkstra, etc.) in a simulated environment using ROS2, Gazebo, and RViz2.

#### Deliverables:

- Autonomous path generation from point A to B
- Obstacle detection and avoidance
- Visualization in Gazebo and RViz2
- Well-documented implementation and performance analysis

---

## Mechanical Pole

### Mission  
Design and manufacture robust and efficient mechanical components that bring our project to life.

### Expertise  
- CAD and 3D modeling  
- Mechanical simulation  
- Precision manufacturing  
- Assembly and testing  

### Challenge Goals  
- Compact and ergonomic design  
- Structural reliability  
- Easy assembly and maintenance  
- Mechanical/electronic integration  

---

## Electronics Pole

### Mission

The electronics sub-team is responsible for designing, developing, and integrating robust and intelligent electronic systems to ensure the proper functioning of mechanical and software components. These systems must collect, process, and transmit data in real-time while ensuring seamless communication between all components.

Key objectives include:

Optimizing circuits for technical constraints.
Integrating sensors, actuators, and microcontrollers.
Managing power supplies and ensuring safety.

<h2 style="font-size: 24px; font-weight: bold; color: #949CDF;">
  Expertise
</h2>

<p style="font-size: 14px; font-weight: bold;">
  Sensors and Data Acquisition
</p>

- Identification and use of various sensors (gyroscope, accelerometer, color sensor, etc.).
- Conversion of environmental data into usable signals.
- Communication via protocols like I2C or SPI.

<p style="font-size: 14px; font-weight: bold;">
  Circuit Design
</p>

- Creation of schematics using tools like KiCAD.
- Design and fabrication of optimized PCBs.
- Power management and protection against short circuits.

<p style="font-size: 14px; font-weight: bold;">
  Microcontrollers and Programming
</p>

- Use of microcontrollers (e.g., ATmega328P) to drive systems.
- Development of Arduino code for data processing and actuator control.
- Implementation of complex logic for input/output management.

<p style="font-size: 14px; font-weight: bold;">
  Communication and Interfaces
</p>

- Setting up communication systems (I2C, UART, etc.).
- Real-time data transmission to web interfaces or control stations.
- Visualization on LCD screens or graphical interfaces.

<p style="font-size: 14px; font-weight: bold;">
  Technical Documentation
</p>

- Detailed documentation including schematics, source code, and test results.
- Regular updates on GitHub with clear and organized content.

<p style="font-size: 14px; font-weight: bold;">
  Challenge Goals
</p>

### Test 1: Gyroscope and Accelerometer
#### Objective :

Identify and exploit a sensor combining gyroscope and accelerometer functions to detect orientation and speed.

#### Deliverables :

- Well-documented Arduino code.
- Data displayed on an LCD screen.
- Schematic in KiCAD and custom power supply.

### Test 2: The Black Box

#### Objective :

Design a black box to record and transmit position and speed data in real-time.

#### Deliverables :

- 7 cm³ cube containing the circuit.
- Data transmission via I2C bus to a control station.
- Optimized PCB design and fabrication.

### Test 3: 7-Segment Display

#### Objective :

Build a 7-segment display using servomotors to show digits 0 to 9.

#### Deliverables :

- Circuit controlled by ATmega328P.
- Lithium battery power supply.
- Optimized Arduino code without blocking functions (e.g., delay()).

### Final Test: Conveyor System

#### Objective :

Design and build a conveyor system to sort objects representing different types of waste (colors: green, yellow, red, blue).

#### Deliverables :

- Conveyor meeting specifications (650 mm length, 100 mm height).
- Automatic detection and sorting of objects.
- Intuitive web interface displaying sorted waste quantities. 

---

## Our Collective Strength

### Cross-Pole Collaboration  
- **Daily syncs** to coordinate progress  
- **Technical brainstorms** to unlock innovation  
- **Cross-testing sessions** for system integrity  
- **Shared documentation** for transparency and continuity  

### Shared Values  
- **Engineering excellence** across disciplines  
- **Creativity** in problem-solving  
- **Team synergy** through open communication  
- **Agility** to iterate and adapt rapidly  

---

*Together, we form a cohesive unit capable of transforming complex challenges into impactful, working solutions!*

---