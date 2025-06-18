# <span style="color: #2E86C1;">**Documentation: Assembly of the Mechanical Gripper**</span>

---

### <span style="color: #28B463;">**Short Demo**</span>

<video width="320" height="240" controls>
  <source src="/videos/mechanic_videos/week1/gripper_simulation.mp4" type="video/mp4">
  Votre navigateur ne supporte pas la balise vidéo.
</video>
---

### <span style="color: #28B463;">**Link to Download**</span>: <a href="/pieces/week1/assemblage_pince.SLDASM" download>**Link to Download Gripper**</a>


## <span style="color:rgb(231, 60, 137);">**Introduction**</span>

The challenge is to assemble a mechanical gripper based on precise assembly plans while respecting dimensions and geometric constraints. The assembly consists of several separate parts, each playing a specific role in the overall operation of the gripper. These parts include:

---

### <span style="color: #28B463;">**Key Components**</span>

- **<span style="color: #5DADE2;">Jack</span>**: The main actuator that allows the gripper to open and close.  
  ![](/images/mechanic_images/week1/pince/jack.png)

- **<span style="color: #5DADE2;">M4 Nut</span>**: Mechanically holds the elements together.  
  ![](/images/mechanic_images/week1/pince/nut.png)

- **<span style="color: #5DADE2;">W4 circlips</span>**: Distributes the pressure of the screws to prevent damage to the surfaces.  
  ![](/images/mechanic_images/week1/pince/circlips.png)

- **<span style="color: #5DADE2;">Body</span>**: The main structure of the gripper, serving as support for all other parts.  
  ![](/images/mechanic_images/week1/pince/body.png)

- **<span style="color: #5DADE2;">Jack End</span>**: Connector between the Jack and the links.  
  ![](/images/mechanic_images/week1/pince/jack_end.png)

- **<span style="color: #5DADE2;">M5x25 Socket Head Screw</span>**: Provides a solid fixation of the elements, particularly to secure the Jack end.  
  ![](/images/mechanic_images/week1/pince/Screw_M5*25.png)

- **<span style="color: #5DADE2;">Link Pin and Jaw Holder</span>**: Allows the rotation of the links and jaw holders.  
  ![](/images/mechanic_images/week1/pince/axes.png)

- **<span style="color: #5DADE2;">Link</span>**: Transmits the movement from the Jack to the jaws.  
  ![](/images/mechanic_images/week1/pince/connecting_rod.png)

- **<span style="color: #5DADE2;">Circlip</span>**: Axially holds the pins to prevent misalignment.  
  ![](/images/mechanic_images/week1/pince/circlip.png)

- **<span style="color: #5DADE2;">Jaws</span>**: The movable parts of the gripper, responsible for grasping objects.
  ![](/images/mechanic_images/week1/pince/Jaws.png)

> **Note:** All parts are available in the provided ZIP file. Make sure to import them into SolidWorks before starting.  
> These can be found in the various files after decompressing the zip file.

---

## <span style="color:rgb(231, 60, 137);">**Assembly Steps**</span>

### <span style="color:rgba(13, 176, 81, 0.7);">**1. Open the parts in SolidWorks**</span>

To begin the assembly, it is necessary to open the files in SolidWorks. Once the interface is launched, click on the "Open" icon and select the file “Gripper Assembly”.

![](/images/mechanic_images/week1/pince/open_solidworks.jpeg)

![](/images/mechanic_images/week1/pince/select_gripper_assembly.jpeg)

This file is already configured as an assembly (.sldasm), which means there is no need to create a new assembly manually. The Jack is already fixed to the body, but the other necessary components still need to be added.

For this, use the **"Insert Components"** option in the top toolbar, then select the available parts from the decompressed folder.

![](/images/mechanic_images/week1/pince/insertion.jpeg)

![](/images/mechanic_images/week1/pince/after_inserting.jpeg)

Geometric constraints, such as coaxiality and coincidence, will be essential for correctly aligning the elements. For example, to align two cylindrical surfaces, apply a coaxiality constraint, then use a coincidence constraint to fix the contacting faces.

---

### <span style="color: #28B463;">**Fixing the Jack**</span>

Fixing the Jack is a crucial step, as it ensures that the movement transmitted by the Jack will be correctly relayed to the other components. In the diagram, it can be seen that the rod of the Jack is connected to the end piece via an M5x25 socket head screw. To make this connection, start by aligning the axis of the screw with the cylindrical cavities of the end piece and the Jack rod.

![](/images/mechanic_images/week1/pince/Jaws_fixing_plan.jpeg)

**Methodology:**

1. **Select the Constraint tab** 

   ![](/images/mechanic_images/week1/pince/Jaws_contraint.png)

2. **Select coaxiality** and then click on the cylindrical faces of the cavity and the screw.

   ![](/images/mechanic_images/week1/pince/cavity_plane_face.png)

3. **Select the coincidence constraint** between the base face of the screw cap and the first drilled surface of the cavity.  
   ![](/images/mechanic_images/week1/pince/srew_head_base.png)

   ![](/images/mechanic_images/week1/pince/cavity_plane_face.png)

---

### <span style="color: #28B463;">**Assembly of the Links**</span>

The links play a central role in transmitting the movement from the Jack to the jaws. Each end of the Jack is connected to a pair of links, forming a sandwich-like configuration. The links are held in place by the link pins and circlips, which prevent any dislocation of the assembly.

![](/images/mechanic_images/week1/pince/assembly_jack-link_plan.jpeg)

**Steps:**

1. Fix one link on one of the axial ends of the Jack end.

   ![](/images/mechanic_images/week1/pince/axial_ends.png)

2. Apply a **coaxiality constraint** between the extreme axial cavities of the link and the end piece.

   ![](/images/mechanic_images/week1/pince/coaxiality_1.jpeg)

3. Apply a **coincidence constraint** between the opposite faces of the link and the end piece.

   ![](/images/mechanic_images/week1/pince/coincidence_1.jpeg)

4. Secure the assembly with the pins and circlips.
   ![](/images/mechanic_images/week1/pince/coincidence_2.jpeg)

   ![](/images/mechanic_images/week1/pince/coaxiality_2.jpeg)

5. Repeat the process for the second end of the Jack.

   ![](/images/mechanic_images/week1/pince/four_result.jpeg)

6. Apply a **parallelism constraint** between the opposite faces of the links.

   ![](/images/mechanic_images/week1/pince/parralellisme_contrainst.jpeg)

---

### <span style="color: #28B463;">**Mounting the Jaw Holders**</span>

The jaw holders consist of two rotation axes: let's name them A1, located at the rounded end, and A2, located at the elongated end. Axis A1 is directly fixed between the links via their pins, while A2 is fixed to the main body via jaw holder pins and circlips.

![](/images/mechanic_images/week1/pince/A1_plan_1.png)

![](/images/mechanic_images/week1/pince/A1_plan_2.png)

**Steps:**

1. Align A1 and A2 on the same axis as their respective cavities.

   ![](/images/mechanic_images/week1/pince/coaxiality_3.jpeg)

2. Apply a **coincidence constraint** between A1 and one of the faces of the links.

   ![](/images/mechanic_images/week1/pince/coincidence_3.jpeg)

3. Secure the pins and circlips.

   ![](/images/mechanic_images/week1/pince/result_2.jpeg)

---

### <span style="color: #28B463;">**Mounting the Jaws**</span>

The jaws are fixed on the truncated face of the jaw holders at the elongated end and are held in place by two pins to ensure strength and durability.

![](/images/mechanic_images/week1/pince/A1_plan.png)

![](/images/mechanic_images/week1/pince/A2_plan.png)

**Steps:**

1. Apply a **coincidence constraint** between the contact surfaces of the jaws and the jaw holders.

   ![](/images/mechanic_images/week1/pince/coaxiality_4.jpeg)

2. Use a **coaxiality constraint** to align the cylindrical cavities of the jaws with the corresponding pins.  
   ![](/images/mechanic_images/week1/pince/coaxiality_5.jpeg)

3. Secure each cavity with M5x16 socket head screws.  
   ![](/images/mechanic_images/week1/pince/coaxiality_6.jpeg)

   ![](/images/mechanic_images/week1/pince/coaxiality_7.jpeg)

---

### <span style="color: #28B463;">**Final Assembly**</span>

In the end, we obtain the sketch of a mechanical gripper capable of grasping and releasing objects.  
![](/images/mechanic_images/week1/pince/final_result_1.jpeg)

---

## <span style="color:rgb(231, 60, 137);">**Additional Tips**</span>

### <span style="color: #28B463;">**Application of the Symmetry Constraint**</span>

To ensure synchronized movement of the links or jaw holders, apply a **symmetry constraint** between the opposite faces of the links.

 
   ![](/images/mechanic_images/week1/pince/symetrie_plan.jpeg)

1. Select the **Symmetry** constraint under the advanced options.

   ![](/images/mechanic_images/week1/pince/symetry_contrainst_1.png)

2. Select the top plane as the axis of symmetry.

   ![](/images/mechanic_images/week1/pince/symetry_contrainst_2.jpeg)

3. Apply the constraint between the opposite faces.

   ![](/images/mechanic_images/week1/pince/symetry_contrainst_3.jpeg)

---

### <span style="color: #28B463;">**Determining the Center of Gravity**</span>

To determine the center of gravity, go to the **Evaluate** tab at the top left and select the **Mass Properties** option.

![](/images/mechanic_images/week1/pince/symetry_contrainst_3.png)

The coordinates of the center of gravity are:  
- **(X = -30.29, Y = 0.0, Z = 24.50)** at its maximum position.  
- **(X = -45.29, Y = 0.00, Z = 24.50)** at its minimum position.

![](/images/mechanic_images/week1/pince/Mass_finding.jpeg)

---

## <span style="color:rgb(231, 60, 137);">**Errors to Avoid During Assembly**</span>

- **Incorrect orientation of parts** can lead to unexpected effects.

  ![](/images/mechanic_images/week1/pince/error_type_1.png)

  ![](/images/mechanic_images/week1/pince/error_type_2.jpeg)

  ![](/images/mechanic_images/week1/pince/error_type_3.jpeg)

- **Confusion of screws.**
- **Ensure that you have used all the parts.**