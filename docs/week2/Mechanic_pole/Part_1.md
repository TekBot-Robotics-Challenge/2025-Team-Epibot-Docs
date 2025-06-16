# <span style="color: #5784BA;">**Documentation for Part 1 of Week 2**</span>


## <span style="color: #08C5D1;">**Challenge Overview**</span>

This challenge involves creating the part shown below using SolidWorks, adhering to the provided measurements, dimensions, and constraints. The dimensions `A`, `B`, and `C` are treated as variables, and the objective is to determine the mass of the part based on the values assigned to these variables.

![Challenge Screenshot](/images/mechanic_images/week2/label.png)

---

### <span style="color: #9AC8EB;">**Short Demo**</span>

<video width="450" height="350" ccontrols autoplay muted>
  <source src="/videos/mechanic_videos/week2/piece_demo.mp4" type="video/mp4">
  Votre navigateur ne supporte pas la balise vidéo.
</video>

---

## <span style="color: #08C5D1;">**Design Steps**</span>

### <span style="color: #9AC8EB;">**Step 1: Creating the Base Sketch**</span>

To begin, open a new part in SolidWorks by clicking the "New" icon, selecting "Part," and clicking "OK."

![Opening a New Part](/images/mechanic_images/week2/ouverture_piece.png) 

#### <span style="color: #DB6A8F; padding-left: 8px;">**Configure Units**</span>
Set the unit system to `MMGS (Millimeter, Gram, Second)`.

![Unit Configuration](/images/mechanic_images/week2/mmgs_setting.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Define Global Variables**</span>
To evaluate the variation in mass based on parameters `A`, `B`, and `C`, define these as global variables:
1. Go to **Tools > Equations**.

![Equation selecting process](/images/mechanic_images/week2/equation_selecting_process.png)

2. In the table that appears, insert your variables and their values. Start with the default values provided in the PDF.

![Equation Table](/images/mechanic_images/week2/globale_variable_setting.png)

<span style="color: #CE6A6B;">**Example:**</span>
- Variable Name: `"A"`, Value: `84`
- Variable Name: `"B"`, Value: `57`
- Variable Name: `"C"`, Value: `43`

![Exemple](/images/mechanic_images/week2/default_values_of_variables.png)

![Exemple](/images/mechanic_images/week2/globale_variables.png)

> **Tip:** Ensure correct syntax when defining variables (e.g., `"A"` for the name and `=84` for the value).

####  <span style="color: #DB6A8F; padding-left: 8px;">**Select a Drawing Plane**</span>
Choose a plane to start sketching. For this design, we’ll use the **`Front Plane`**.

![Plane Selection](/images/mechanic_images/week2/plan_selection.png)

#### <span style="color: #DB6A8F; padding-left: 8px;"> **Draw the Outline**</span>
Sketch all the edges of the figure, including the circle specified in the diagram.

![Base Sketch](/images/mechanic_images/week2/esquisse_2D.png)

![got Sketch](/images/mechanic_images/week2/essaie_de_design.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Apply Dimensions**</span>
Use the **`Smart Dimension`** tool to apply the required dimensions. Select the edges or features you want to dimension and input the corresponding values.

![Dimension Process]()

<span style="color: #08C5D1;">**Demo of a proocess**</span>

---

#### <span style="color: #DB6A8F; padding-left: 8px;">****Add Tangency Relations**</span>
Apply tangency relations to the specified areas.


![Tangency to do](/images/mechanic_images/week2/tangeante_to_apply.png)

![Tangency Relations](/images/mechanic_images/week2/application_de_la_relation_de_tangeante.png)

---


### <span style="color: #9AC8EB;">**Step 2: Extrusion**</span>

![Expected Volume](/images/mechanic_images/week2/cotation_height_C_to_do.png)

Once the base sketch is complete, extrude it to add material. Use the global variable `C` as the extrusion height.
1. Go to **Features > Extruded Boss/Base**.
2. Input the global variable `C` in the dimension field.
3. Select both the outer contour and the circular cutout to ensure simultaneous extrusion.

![Extrusion Process](/images/mechanic_images/week2/extrusion_desquisse.png)

![Extrusion Process](/images/mechanic_images/week2/extrusion_2.png)

> **Note:** Extruding the circular cutout along with the base simplifies the process and avoids additional steps later.

---

### <span style="color: #9AC8EB;">**Step 3: Assign Material**</span>

Assign the material **AISI 1020 Steel** to the part:
1. Click the **`Material`** icon in the left-hand panel.
2. Select **`AISI 1020 Steel`** from the list.

![Material Assignment](/images/mechanic_images/week2/application_materiel.png)

---

### <span style="color: #9AC8EB;">**Step 4: Evaluate Mass**</span>

To calculate the mass of the part:
1. Go to **Evaluate > Mass Properties**.
2. Review the calculated mass.

![Mass Determining Process](/images/mechanic_images/week2/mass_property.png)

<span style="color: #CE6A6B;">**Results:**</span>
- For `(A, B, C) = (81, 57, 43)`: **Mass = 939.54 g**
- For `(A, B, C) = (84, 59, 45)`: **Mass = 1032.32 g**


> **Tip:** To modify the values of `A`, `B`, or `C`, return to the **Equations** table and adjust as needed.

---

## <span style="color: #08C5D1;">**Additional Notes**</span>

- **Best Practices:**
  - Always double-check dimensions and tangency relations before extruding.
ß
  - Ensure that you don't overlook any single quotation
  
  - Always verify that you are using mmgs.

  - Maintain proper syntax for global variables.

---
