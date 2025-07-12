# Smart Conveyor System – IT Component Overview

The goal of this test is to implement an intelligent and reliable embedded system with real-time web visualization.

<video controls src="/week4/videos/sorting_white.mp4" title="Title" ccontrols autoplay muted></video>

<video controls src="/week4/videos/sorting_blue.mp4 " title="Title" ccontrols autoplay muted></video>

## 1. Intelligent Detection 
- Integrate a color sensor capable of detecting 4 waste types: **Red**, **Blue**, **Green**, **Yellow**
- Implement reliable object detection using

### Tasks
- Connect and configure color sensor
- Calibrate color recognition for 4 types
- Implement presence detection logic

---

## 2. Conveyor Automation 
- Conveyor activates **only** when waste is detected

### Tasks
- Develop motor control logic for conveyor
- Trigger conveyor based on presence signal

---

## 3. Web Interface – Real-Time Monitoring 
- Web interface displays **quantity per waste type**

### Tasks
- Design responsive and clean dashboard
- Display real-time waste counters by color

---

## 4. Data Processing 
- Maintain accurate counting by waste type
- Ensure real-time data updates

### Tasks
- Implement counter logic for each color
- Use pub/sub and API to push data to UI

---

## 5. System Integration 
- Ensure reliable communication between **Arduino (ATmega328P)** and the **web interface**

### Tasks
- Use serial protocol
- Parse and forward data from Arduino to web server (Arduino -> ROS -> Web site)

---