# <span style="color: #5784BA;">**Documentation for Part 2 of Week 2**</span>

## <span style="color: #08C5D1;">**Challenge Overview**</span>

The objective of this exercise is to modify the piece created in Part 1 ([link to Part 1](/week2/Mechanic_pole/Part_1)) by applying precise material removal at strategic points while respecting dimensions, tangency constraints, and other specifications.

![Label Screenshot](/images/mechanic_images/week2/label_2.png)

### <span style="color: #9AC8EB;">**Short Demo**</span>

<video width="450" height="350" ccontrols autoplay muted>
  <source src="/videos/mechanic_videos/week2/part_2_demo.mp4" type="video/mp4">
  Votre navigateur ne supporte pas la balise vidéo.
</video>

---

##  <span style="color: #08C5D1;"> **Implementation Steps** </span>

###  <span style="color: #9AC8EB;"> **Step 1: Second Cylindrical Drilling**</span>

We will begin by performing an 11 mm diameter drilling on both ends of the piece at the second extremity.

   ![Piercing To Do](/images/mechanic_images/week2/percing_to_do.png)

#### <span style="color: #DB6A8F; padding-left: 8px;"> **Process**</span>
1. Select the base sketch for the operation.

   ![Select Sketch](/images/mechanic_images/week2/select_esquisse.png)

2. Draw a circle at the desired extremity and apply the correct dimensions.

   ![Piercing Drawing](/images/mechanic_images/week2/persing_drawed.png)

3. Apply material removal using the drawn circle:
   - Click on **`Features > Extrude/Cut Material`**.
   - Select the circle and apply the extrusion.

   ![Extrusion Process](/images/mechanic_images/week2/extrusion_persing_cylinder.png)

> <span style="color: #CE6A6B;">**Note:**</span> The software defaults to "up to the next surface," but you can also adjust it manually if needed.

---

### <span style="color: #9AC8EB;">**Step 2: Create a Prismatic Cut**</span>

   ![Extrusion making](/images/mechanic_images/week2/prisme_extrusion.png)


#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
1. Divide the piece into three sections as shown below:

   ![Compartmentation Screenshot](/images/mechanic_images/week2/compatimentation.png)

2. To achieve this:

   - Draw a horizontal line and dimension it at 52 mm from the opposite extremity.

   - Draw a second horizontal line to delimit the crest.

     ![Lines for Compartmentation](/images/mechanic_images/week2/lines_to_compatimente.png)

3. Use the **`Convert Entities`** tool to select the contours of the surfaces to be extruded.

   ![Entity Conversion](/images/mechanic_images/week2/entities_convertion.png)

4. Use the **`Trim Entities`** tool to remove excess outlines.

   ![Trimming Entities](/images/mechanic_images/week2/ajuster_les_elements.png)

5. Perform an extrusion of 19 mm on these surfaces.

   ![Extrusion Process](/images/mechanic_images/week2/extrusion_1.png)

>  <span style="color: #CE6A6B;">**Warning:**</span> Always ensure the correct face orientation for the sketch. The extrusion zone is on the left when the crest faces forward.
>
> ![Correct Orientation Warning 1](/images/mechanic_images/week2/warning_1_good_sens.png)

> ![Correct Orientation Warning 2](/images/mechanic_images/week2/warning_2_good_sens.png)

---

###  <span style="color: #9AC8EB;">**Step 3: Create the Last Semi-Cylindrical Cavity**</span>

We will now focus on the final drilling step.


   ![Semi-Cylindrical Cavity](/images/mechanic_images/week2/last_persing.png)

####  <span style="color: #9AC8EB;">**Process**</span>
1. Create a plane located 12 mm from the solid face of the piece.

   ![Plane Location](/images/mechanic_images/week2/plan_location.png)

   ![Plane Drawing](/images/mechanic_images/week2/plan_drawing.png)

2. Draw a circle in the area to be drilled and dimension its radius to 41 mm.

   ![Circle Drawing](/images/mechanic_images/week2/circle_for_last_persing_drawing.png)

3. Apply a dimension of 36 mm.

   ![Cotation Application](/images/mechanic_images/week2/cotation_36_mm.png)

4. Delimit an entity in the specified zone:

   - Click on **`Convert Entities`** and select the area to be delimited.

   ![Semi-Cylindrical Cavity](/images/mechanic_images/week2/last_persing.png)

   ![Entity Conversion](/images/mechanic_images/week2/entities_convertion_2.png)

5. Use the **`Trim Entities`** tool to polish and remove excess portions.

   ![Trimming Tool Icon](/images/mechanic_images/week2/icone_ajuster_entite.png)

6. Align the center of the circle with the extreme edge:

   - Draw a vertical construction line along the extreme edge.

   - Apply a coincidence relation between the circle's center and the line.

   ![Alignment Process](/images/mechanic_images/week2/coincidence_relation.png)

7. Apply a material removal of 24 mm from the obtained surface:
   - Click on **`Extrude/Cut Material`**.
   - Select the opposite side of the solid face and input the value `24mm` in the dimension field.

   ![Extrusion Process](/images/mechanic_images/week2/extrusion_3.png)

---

###   <span style="color: #9AC8EB;">**Step 4: Determine the Mass**</span>

To calculate the mass of the piece:
1. Select Equation icon and actuate the values of global variables

   ![Equation selecting process](/images/mechanic_images/week2/equation_selecting_process.png)

   ![varaible changing](/images/mechanic_images/week2/variable_changing.png)

1. Click on the **`Evaluate`** tab, then select **`Mass Properties`**.

   ![Mass Evaluation](/images/mechanic_images/week2/mass_2_evaluation.png)

The final mass obtained for this test is   <span style="color: #CA3C66;">**`628.18 g`**</span>.

---

##  <span style="color: #08C5D1;">**Additional Notes**</span>

  - Always double-check dimensions and constraints before applying material removal.

  - Ensure proper alignment of features to avoid misplacement during extrusion.

  - Use construction lines and geometric relations (e.g., coincidence, tangency) to maintain precision.

---
