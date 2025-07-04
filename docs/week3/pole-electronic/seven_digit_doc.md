# **<span style="color: #2E86C1;">Documentation for the Realization of the 7-Segment Servo Display Mechanism</span>**


## **<span style="color: #FF69B4;">Introduction</span>**

This project focuses on designing and building the physical body of a **7-segment servo display**. Unlike traditional designs, we aimed for an innovative and functional approach by adopting a **vertical configuration** inspired by the principles of mechanical engineering, particularly the operation of hydraulic cylinders (or "vérins"). Our goal was to create a unique display that combines functionality with aesthetic appeal, leveraging inspiration from open-source projects such as:

- [Kinetic Digital Clock Arduino 3D Print](https://www.instructables.com/Kinetic-Digital-Clock-Arduino-3D-Print/)
- [Digital Analog Clock With Stepper Motors](https://www.instructables.com/Digital-Analog-Clock-With-Stepper-Motors/)

In the following sections, we document the entire process, including analysis, design, fabrication, assembly, and testing.

---

## **<span style="color: #2E86C1;">Step 1: Analysis</span>**

### **<span style="color: #FF69B4;">Innovative Design Choices</span>**
The first challenge we set for ourselves was to mount the display in a **`vertical format`**, deviating from the illustrating horrizontal layout. This decision was driven by our desire for a compact and visually appealing design and make differently.

**Illustrating version**
![Final Design vs Original Example](/images/electro/their_exemple.png)

![](/images/electro/Face_and_segments_after_montage.jpg)

Additionally, we wanted to avoid sharp edges, opting instead for **`rounded borders`** to give the display a sleek and recpecting the subject.

![Rounded Borders vs Original](/images/electro/rounded_border.png)

### **<span style="color: #FF69B4;">Mechanism Design</span>**
For the internal mechanism, we drew inspiration from the open-source project [Kinetic Digital Clock Arduino 3D Print](https://www.instructables.com/Kinetic-Digital-Clock-Arduino-3D-Print/), particularly its **`gear system`**. This inspired us to recreate our own mechanism tailored to meet the specific requirements of this project.

![Mechanism Inspiration](/images/electro/mechanismes_inspiration.png)

The core idea is to use **`servo motors`** to drive gears, converting rotational motion into linear motion for each segment. At the end we would to have something like this:

![simulating video of what we mant](/images/electro/what_we_want_to_have.png)

---

### **<span style="color: #FF69B4;">Creation steps</span>**

To get started, we first designed and tried to represent the models and plans. The software

Using **`SolidWorks`**, **`Fusion 360`**, and **`PrusaSlicer`**, we modeled and fabricated each component of the display. You could download theses here [Download here](/images/electro/resorces.zip)
Below are the details of the key parts:

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

### **<span style="color: #FF69B4;">1. Back face </span>**
we have choice to conserve the back face of https://www.instructables.com/Kinetic-Digital-Clock-Arduino-3D-Print/ this link to avoid measurement errors.

![Back face image](/images/electro/back-face.png)

### **<span style="color: #FF69B4;">. Front face and Box area</span>**
We have design our front face and box parts using **`illustrator`** model designer. This step was verry challenging and we have repeat the process many times. Same case when segments designing. We opted for a multi-stage configuration. One is reserved for the motor system (servos, gears, support), the second is to house the circuit and the third is to contain the rear of the power supply system. We therefore have 10cm in total for the 3 stages. The design files are provide on zipped file side. 


### **<span style="color: #FF69B4;">3D Printing</span>**
Once the design was complete, we proceeded to fabricate the components using **`3D printing`**. Below are the steps involved:

1. **<span style="color: #FF69B4;">Slicing the Models</span>**
   - We used **PrusaSlicer** to prepare the models for 3D printing.
   - Settings included a layer height of 0.2 mm, 20% infill, and support structures where necessary.

   ![Slicing Process](/images/electro/slicing_1.png)

   ![Slicing Process](/images/electro/slicing_2.png)

2. **<span style="color: #FF69B4;">Printing</span>**
   - The components were printed using PLA filament for its ease of use and durability.
   - Total printing time for all parts was approximately **7 hours**.

   ![Impression In action ](/images/electro/Impression_3D.jpg)

### **<span style="color: #FF69B4;">Laser cutter</span>**
we have choose to have our box in wood and use a lazer cutter to have shaped our illustrator design. We have been inspired by box models available in box.py tools, a free and open source librairies of box models for laser cutting. This enable to us to have a very great and custom design and coast less time

![Laser cutter in action](/images/electro/laser_cutting.jpg);

---
### **At the End**

![Materials](/images/electro/materials.jpg)

![Materials](/images/electro/materials_2.jpg)

## **<span style="color: #2E86C1;">Assembly</span>**

### **<span style="color: #FF69B4;">Process Overview</span>**
The assembly process involved combining all the fabricated components into a fully functional display. Below are the key steps:

1. **<span style="color: #FF69B4;">Attaching Segments to Gears</span>**
   - Each segment was attached to its respective gear using the toothed body. This connection ensures precise control over the segment's movement.

<iframe src="https://player.vimeo.com/video/1097439176?h=1dec9cefd6&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px%;" title="segments-to-gear_moving"></iframe>

2. **<span style="color: #FF69B4;">Mounting Servos to the Frame</span>**
   The servos were mounted onto the bracket assembly using screws or adhesive mounts. Proper alignment was critical to ensure smooth operation.

   ![Slicing Process](/images/electro/Step_2_assembly.jpg)

3. Assemble and mount the box compartments
   ![Slicing Process](/images/electro/materials_2.jpg)

4. Integrate the necessary circuits

   ![Motors](/images/electro/motors.jpg)
 ----

 
## **<span style="color: #FF69B4;">Final Result</span>**


## **<span style="color: #2E86C1;">Conclusion</span>**

This documentation outlines the step-by-step process of designing, fabricating, and assembling the 7-segment servo display. By combining innovative design choices with precise engineering, we created a functional and visually appealing mechanism. This project demonstrates our ability to think creatively while adhering to technical constraints.
