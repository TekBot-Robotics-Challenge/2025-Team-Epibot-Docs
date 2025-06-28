# **<span style="color: #2E86C1;">Documentation for the Realization of the 7-Segment Servo Display Mechanism</span>**


## **<span style="color: #FF69B4;">Introduction</span>**

This project focuses on designing and building the physical body of a **7-segment servo display**. Unlike traditional designs, we aimed for an innovative and functional approach by adopting a **vertical configuration** inspired by the principles of mechanical engineering, particularly the operation of hydraulic cylinders (or "vérins"). Our goal was to create a unique display that combines functionality with aesthetic appeal, leveraging inspiration from open-source projects such as:

- [Kinetic Digital Clock Arduino 3D Print](https://www.instructables.com/Kinetic-Digital-Clock-Arduino-3D-Print/)
- [Digital Analog Clock With Stepper Motors](https://www.instructables.com/Digital-Analog-Clock-With-Stepper-Motors/)

In the following sections, we document the entire process, including analysis, design, fabrication, assembly, and testing.

---

## **<span style="color: #2E86C1;">Step 1: Analysis</span>**

### **<span style="color: #FF69B4;">Innovative Design Choices</span>**
The first challenge we set for ourselves was to mount the display in a **`vertical format`**, deviating from the traditional horrizontal layout. This decision was driven by our desire for a compact and visually appealing design.

**Illustrating version**
![Final Design vs Original Example](/images/electro/their_exemple.png)

![](/images/electro/Face_and_segments_after_montage.jpg)

Additionally, we wanted to avoid sharp edges, opting instead for **`rounded borders`** to give the display a sleek and modern appearance.

![Rounded Borders vs Original](/images/electro/rounded_border.png)

### **<span style="color: #FF69B4;">Mechanism Design</span>**
For the internal mechanism, we drew inspiration from the open-source project [Kinetic Digital Clock Arduino 3D Print](https://www.instructables.com/Kinetic-Digital-Clock-Arduino-3D-Print/), particularly its **`gear system`**. This inspired us to recreate our own mechanism tailored to meet the specific requirements of this project.

![Mechanism Inspiration](/images/electro/mechanismes_inspiration.png)

The core idea is to use **`servo motors`** to drive gears, converting rotational motion into linear motion for each segment. Below is a screenshot of a segment in motion:

---

### **<span style="color: #FF69B4;">Etapes d'assemblage</span>**

## **<span style="color: #2E86C1;">Step 2: Component Design</span>**

Using **`SolidWorks`**, **`Fusion 360`**, and **`PrusaSlicer`**, we modeled and fabricated each component of the display. Below are the details of the key parts:

### **<span style="color: #FF69B4;">1. Bracket Assembly</span>**
The bracket serves as the backbone of the mechanism:
- It holds the servo motor vertically and connects it to the gear system.
- On one side, it is fixed at a **`90° angle`** to support the longitudinal part of the segment.
- The bracket ensures precise alignment and smooth movement of the segments.

![Bracket Components](/images/electro/differents_part_bracket.png)

### **<span style="color: #FF69B4;">2. Segment</span>**
The segment is the visible part of the display and consists of two parts:
1. **Head**: The surface seen by the user, featuring rounded edges for aesthetic appeal.
2. **Body**: The hidden, toothed portion that interacts with the gear system.

![Segment Components](/images/electro/segment_parts.png)

---

## **<span style="color: #2E86C1;">Step 3: Fabrication</span>**

### **<span style="color: #FF69B4;">3D Printing</span>**
Once the design was complete, we proceeded to fabricate the components using **3D printing**. Below are the steps involved:

1. **<span style="color: #FF69B4;">Slicing the Models</span>**
   - We used **PrusaSlicer** to prepare the models for 3D printing.
   - Settings included a layer height of 0.2 mm, 20% infill, and support structures where necessary.

   ![Slicing Process](/images/electro/slicing_1.png)
   ![Slicing Process](/images/electro/slicing_2.png)
   
2. **<span style="color: #FF69B4;">Printing</span>**
   - The components were printed using PLA filament for its ease of use and durability.
   - Total printing time for all parts was approximately **7 hours**.
  
---

## **<span style="color: #2E86C1;">Step 4: Assembly</span>**

### **<span style="color: #FF69B4;">Process Overview</span>**
The assembly process involved combining all the fabricated components into a fully functional display. Below are the key steps:

1. **<span style="color: #FF69B4;">Attaching Segments to Gears</span>**
   - Each segment was attached to its respective gear using the toothed body. This connection ensures precise control over the segment's movement.

2. **<span style="color: #FF69B4;">Mounting Servos to the Frame</span>**
   - The servos were mounted onto the bracket assembly using screws or adhesive mounts. Proper alignment was critical to ensure smooth operation.
  
   
   [Slicing Process](/images/electro/Step_1_assembly.mp4)

   ![Slicing Process](/images/electro/Step_2_assembly.jpg)

   ![Slicing Process](/images/electro/laser_cutting.jpg)

   ![Slicing Process](/images/electro/laser_cutting_2.jpg)

---

## **<span style="color: #2E86C1;">Step 5: Enclosure Design</span>**

### **<span style="color: #FF69B4;">Vision</span>**
Our vision for the enclosure was to create a **two-tiered container**:
- **Top Tier**: Houses the servo motors.
- **Bottom Tier**: Contains the electronic circuitry.

### **<span style="color: #FF69B4;">Front Face</span>**
The front face contains hollow cavities shaped like the number "8," through which the segments slide to display digits. This part is critical for ensuring proper alignment and visibility of the segments.

![Front Face Design](/images/electro/face_avant.png)

---

## **<span style="color: #2E86C1;">Conclusion</span>**

This documentation outlines the step-by-step process of designing, fabricating, and assembling the 7-segment servo display. By combining innovative design choices with precise engineering, we created a functional and visually appealing mechanism. This project demonstrates our ability to think creatively while adhering to technical constraints.
