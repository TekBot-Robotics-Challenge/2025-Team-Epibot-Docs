## <span style="color: #08C5D1;">## Mechanical Design Documentation – Autonomous Conveyor System</span>

### <span style="color: #9AC8EB;">Project Context</span>

The objective of this challenge is to bring together all the skills acquired during previous projects to design and build an `**autonomous, intelligent, and efficient conveyor system**`.

To achieve this, our multidisciplinary team collaborated to produce sketches and diagrams envisioning the conveyor. The conveyor’s main dimensions are:

* **Length:** 650 mm
* **Height of belt from floor:** 100 mm

The design process followed **two key phases**:

1. **Virtual Design** — conceptualizing and modeling our components in CAD.
2. **Physical Design** — preparing for real-world fabrication and assembly.

This document details the mechanical conception process, the constraints we encountered, and how we solved them.

---

## <span style="color: #08C5D1;">## 🖥️ Virtual Design</span>

<iframe src="https://player.vimeo.com/video/1100264151?badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="width:100%;height:300px;" title="vue-3d-convoyeur"></iframe>

The virtual design work primarily involved **SolidWorks** for 3D modeling, and **Illustrator** for preparing sketches intended for laser cutting.

Below is an overview of the mechanical components and the reasoning behind each design decision.

---

### <span style="color: #9AC8EB;">🔩 Vertical Support</span>

<iframe src="https://player.vimeo.com/video/1100264624?badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" width="100%" height="300px" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" title="vue-support-verticale-3d"></iframe>

* This is the **main vertical plate** supporting the entire conveyor structure.
* **Dimensions:**

  * Length: 650 mm (65 cm)
  * Height: 110 mm (11 cm)

> We deliberately designed it 10 mm higher than the required 100 mm to maintain a small safety margin.

#### <span style="color: #DB6A8F; padding-left: 8px;"> Features: </span>

* **Two rows of small cavities** to mount the horizontal supports.
* **Circular holes (30 mm diameter)** at each end to fit bearings, allowing smooth rotation of the drum shaft.

![emplacement_roulement](/images/mechanic_images/week4/emplacement_roulement.png)

> Many additional cavities serve mainly decorative purposes and to reduce weight.

**Important Note:**

* The motor mounting location was updated **after** the initial video capture. The final position was based on the actual dimensions of the Nema 17 stepper motor we procured.

**Design Process**

<iframe src="https://player.vimeo.com/video/1100264079?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style=";top:0;left:0;width:100%;height:300px;" title="support-de-face-conception"></iframe>

---

### <span style="color: #9AC8EB;">🔧 Horizontal Supports</span>

<iframe src="https://player.vimeo.com/video/1100266127?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px;" title="vue-support-horizontal-3d"></iframe>

* Two horizontal plates positioned into the rows of cavities on the vertical supports.
* They run **perpendicular** to the vertical plates and ensure the entire structure stays balanced.

#### <span style="color: #DB6A8F; padding-left: 8px;"> Purpose: </span>

* Provide stability to the conveyor belt, especially if it needs to carry heavy loads.
* Help prevent overflow or jamming of transported items.

> The horizontal supports straddle the drum, making it easier to deploy the belt over them.

#### <span style="color: #DB6A8F; padding-left: 8px;"> Dimensions:</span>

* **Spacing between plates:** 70 mm
* **Width of each plate:** 70 + (2 × 40) mm = 150 mm

  * The extra 40 mm per side accounts for the interlocking features.
* **Length:** 590 mm

**Design Process**

<iframe src="https://player.vimeo.com/video/1100264115?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style=";top:0;left:0;width:100%;height:300px;" title="support-horrizontal-design-process"></iframe>

---

### <span style="color: #9AC8EB;">🥁 Drum</span>

<iframe src="https://player.vimeo.com/video/1100264137?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style=";top:0;left:0;width:100%;height:300px;" title="tambour-vue-3d"></iframe>

* Cylindrical part directly in contact with the conveyor belt, transmitting motion.

#### <span style="color: #DB6A8F; padding-left: 8px;"> Features:</span>

* Both drum ends are designed to fit into bearings mounted in the vertical supports.
* One drum end has an extended shaft (**23 mm vs. 15 mm**) to:

  * Pass through the thicker vertical plate.
  * Accommodate a pulley that transmits rotation from the motor.

![ drum protruding through vertical plate](/images/mechanic_images/week4/port_capteurs_parts.png)

**Design Process**



---

### <span style="color: #9AC8EB;">⛓️ Pulley System</span>

<iframe src="https://player.vimeo.com/video/1100264175?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px;" title="vue-poulie-3d"></iframe>

#### <span style="color: #DB6A8F; padding-left: 8px;"> Objective:</span>

To transmit motion from the **Nema 17 stepper motor** to the drum.

* We explored several transmission methods. The following video was a valuable reference:
  [YouTube Video on Pulley Systems](https://www.youtube.com/watch?v=5Xn26kxmX5U)

#### <span style="color: #DB6A8F; padding-left: 8px;"> Solution: </span>

* We chose a **belt and pulley system.**
* Since we couldn’t source custom belts of precise lengths, we decided to use **elastic bands** as belts.

**Elastic Band Specifications:**

* Length: 100 mm
* Width: 1 mm
* Height: 1 mm

#### <span style="color: #DB6A8F; padding-left: 8px;"> Pulley Details:</span>

* Two pulleys were designed:

  * One with a 5 mm axial hole (fits the motor shaft).
  * One with a 10 mm axial hole (fits the drum shaft).

> The pulleys were specifically modeled to hold multiple elastic bands for better grip and reliability.

---

### <span style="color: #9AC8EB;">🎯 Sensor Supports</span>

<iframe src="https://player.vimeo.com/video/1100264058?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px;" title="port-capteur-vue-3d"></iframe>

We designed a dedicated structure to hold:

* A **color sensor** (TCS34725)
* A **presence detector** (KY-008)

#### <span style="color: #DB6A8F; padding-left: 8px;"> Design:</span>

* A surface parallel to the conveyor belt for mounting the color sensor.
* A vertical surface perpendicular to the belt for mounting the KY-008 presence sensor.
* Both surfaces are supported by a bent bar attached to one of the horizontal plates.

![Images of sensor components and mounts](/images/mechanic_images/week4/port_capteurs_parts.png)

**Design Process**

<iframe src="https://player.vimeo.com/video/1100264021?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:;" title="port-capteur-3D"></iframe>

---

## <span style="color: #08C5D1;">## 🛠️ Assembly in SolidWorks</span>


<iframe src="https://player.vimeo.com/video/1100263974?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px;"title="assemblage-video-solid-works"></iframe>

To create an accurate virtual assembly, we imported:

* A **Nema 17 stepper motor** model (downloaded from [GrabCAD](https://grabcad.com/)).
* A **KY-008 laser sensor.**
* A **TCS34725 color sensor.**
This ensured our virtual model closely represents the real mechanical constraints and space requirements.

[Download All SolidWorks Files and Assets](/images/mechanic_images/week4/conveyor_virtual_design.zip)

---
## <span style="color: #08C5D1;">## 📂 Download All Components</span>


# ⚙️ Physical Construction Phase

---

## <span style="color: #08C5D1;">## ✅ Obtaining the Parts</span>

### <span style="color: #9AC8EB;">3D Printing</span>

At this stage, we already had all the CAD files and parts designed. Components like the sensor bracket, the drum, and the pulleys were 3D-printed using PLA material because they are small-sized parts.  

The workflow was as follows:

- Convert SolidWorks files to `.stl` format.
- Import the `.stl` files into **PrusaSlicer**.
- Generate the G-code.
- Transfer the G-code to the 3D printer.

---

### <span style="color: #9AC8EB;">###  Laser Cutting </span>

The vertical and horizontal panels were too long to print or fit into standard laser-cutting machines. One challenge was that the working area of our laser cutter was limited to about **60 × 30 cm**, while our vertical supports were designed to be 65 cm long.

#### <span style="color: #DB6A8F; padding-left: 8px;"> Design Adaptations</span>

To overcome this, we modified our design:

- We **split the motor zone from the rest of the conveyor** and connected them horizontally with a new support plate.  

    ![Screenshot showing separation and demarcation](/images/mechanic_images/week4/separation_demarquation.png)

- We created a **new motor zone** to house the motor separately.  

<iframe src="https://player.vimeo.com/video/1100824407?badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px;" title="création de la zone motrice"></iframe>

- We designed a **basal support structure** capable of holding the entire assembly. We also made adjustments to the dimensions to ensure the final length still reached **650 mm**.

<iframe src="https://player.vimeo.com/video/1100824443?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px%;" title="support_basale"></iframe>

After these changes, we updated the assembly in SolidWorks, which resulted in the following view:

<iframe src="https://player.vimeo.com/video/1100825538?badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:;" title="vue_assembly_modificated"></iframe>

---

## <span style="color: #08C5D1;">## ✂️ Laser Cutting Workflow</span>

To prepare for laser cutting, we:

- Opened the **technical drawings (mise en plan)** of the parts.
- Exported them in **SVG format**.
- Configured the laser cutting parameters using **Adobe Illustrator**.

Here are some screenshots from this process:

- ![Mise en plan 1 screenshot](/images/mechanic_images/week4/mise_en_plan_1.png)
- ![Mise en plan 2 screenshot](/images/mechanic_images/week4/mise_en_plan_2.png)
- ![Mise en plan 3 screenshot](/images/mechanic_images/week4/mise_en_plan_3.png)

All the files used for laser cutting are available for download in the folder laser_cutting_plans.

Once the files were ready, we proceeded with the cutting and 3D printing of all the necessary parts.

![Images of obtained materials](/images/mechanic_images/week4/material.jpg)

---

## <span style="color: #08C5D1;">## 🔧 Assembly Process</span>

Once we had all the components, we started assembling the conveyor. Here’s a step-by-step summary of the process:

- **Insertion of the four bearings** into their dedicated slots.  
    ![Screenshot showing bearing insertion](/images/mechanic_images/week4/roulement_fixing.jpg)

- **Mounting the two horizontal supports** into the rows of cavities on the vertical supports.
    ![Screenshot showing horizontal supports installation](/images/mechanic_images/week4/montage_verti-horrizontal_2.jpg)

- **Inserting the drums** into the bearing axes.  
    ![Screenshot showing drum insertion](/images/mechanic_images/week4/poulie_fixing.jpg)

- **Placing the second vertical face** on top of the assembly.  
    ![Screenshot showing second vertical panel assembly](/images/mechanic_images/week4/second_face.jpg)

- **Inserting the pulleys** onto the motor and drum shafts.  
    ![Screenshot showing pulley insertion](/images/mechanic_images/week4/poulie_fixing.jpg)

    ![Screenshot showing pulley insertion](/images/mechanic_images/week4/poulie_fixing_2.jpg)

- **Fixing the NEMA 17 motor** and the basal horizontal support. 

- **Cutting the conveyor belt:**
    - We used a **1.2 × 0.065 m leather sheet** (the same material typically used in upholstery).
    - The belt was placed over the drums and its ends were glued together to form the loop.

    ![Screenshot of belt cutting](/images/mechanic_images/week4/decoupe.jpg)
  
    ![Screenshot of belt after cutting and gluing](/images/mechanic_images/week4/tapis_picking.jpg)

- **Attaching the sensors** to the sensor bracket.  
- **Fixing the sensor bracket** to the vertical support.  

---