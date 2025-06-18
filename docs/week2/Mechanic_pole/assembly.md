# <span style="color: #5784BA;">**Documentation for Week 2 Assembly**</span>

---

##  <span style="color: #08C5D1;">**Challenge Overview**</span>

The objective of this exercise is to assemble a mechanical system using the provided parts while respecting the constraints and adhering to the origin of the assembly. The files required for this assembly can be downloaded via this lin. Once decompressed, you will have access to the necessary parts.

The goal is to apply different angle values (`A`, `B`, and `C`) at the rotational axes and determine the center of gravity (center of mass) for each configuration.

![Task to do](/images/mechanic_images/week2/assembly/label_3.png)

---

### <span style="color: #9AC8EB;">**Short Demo**</span>
<iframe src="https://player.vimeo.com/video/1094296418?h=5769ac1ab9&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px;" title="demo-assembly-2"></iframe>

## <span style="color: #08C5D1;">**Implementation Steps**</span>

###  <span style="color: #9AC8EB;">**Step 0: Opening and Importing Parts**</span>

Once the compressed file is extracted, open the parts in SolidWorks.

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
1. Open `SolidWorks`.
2. Click on **`New > Assembly`**.

   ![Open Assembly in SolidWorks](/images/mechanic_images/week2/assembly/open_assembly_in_solidwork.png)

3. Select the file containing the parts and open it.

   ![File and Piece Selection](/images/mechanic_images/week2/assembly/file_piece_selectioning.png)

   <span style="color: #CE6A6B;">**Uploaded**</span>

   ![Upload part](/images/mechanic_images/week2/assembly/piece_uploaded.png)

4. Duplicate the required pieces by copying (`Ctrl + C`) and pasting (`Ctrl + V`).

   - Determine how many duplicates you need based on the assembly plan.

   ![Number of Duplicates](/images/mechanic_images/week2/assembly/number_to_duplicate.png)

5. After duplication, ensure all required pieces are ready for assembly.
   ![After Duplication](/images/mechanic_images/week2/assembly/piece_duplication.png)

---

###  <span style="color: #9AC8EB;">**Step 1: Aligning the Base of a Long Pin with the Assembly Origin**</span>

This step is critical as the center of gravity will be determined relative to this reference point. The goal is to align the center of the base surface of a long pin with the origin of the assembly coordinate system.

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
1. Select one of the pins and display its entities.

2. While holding `Shift`, select both the origin of the part and the origin of the assembly.

3. Apply a **`Coincidence Constraint`** by clicking on **`Add Mate`** and selecting the relevant faces.

<iframe src="https://player.vimeo.com/video/1094297116?h=a54fde4ad3&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px;" title="coincide-origine"></iframe>


> <span style="color: #CE6A6B;">**Explanation:** </span>
> Proper alignment of the pin's base with the assembly origin ensures accurate calculations of the center of gravity later.

---

###   <span style="color: #9AC8EB;">**Step 2: Joining the Pin to the Connector**</span>

Here, we will attach a connector to the pin that was just fixed.

   ![First Joinning](/images/mechanic_images/week2/assembly/first_mounting.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
1. Align the cylindrical cavity of the connector with the pin by applying a **`Coaxiality Constraint`**.

   ![Coaxiality Process](/images/mechanic_images/week2/assembly/process_of_coaxiality_first_fixing.png)

2. Apply a **`Coincidence Constraint`** between the rear surface of the connector and the front surface of the pin.

   ![Coincidence Process](/images/mechanic_images/week2/assembly/firts_fixing_2.png)

---

###  <span style="color: #9AC8EB;">**Step 3: Connecting Two Connectors**</span>

On the assembly plan, connectors are joined together via pins through their axes (front axis of connector 1 → rear axis of connector 2).

 ![Connection to do](/images/mechanic_images/week2/assembly/pin_to_pin_fixing.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
1. Apply a **`Coaxiality Constraint`** between the cylindrical cavities of the two connectors' axes.

 ![Connectors Coaxiality](/images/mechanic_images/week2/assembly/pin_to_pin_process_video_coaxiality.png)

2. Apply a **`Coincidence Constraint`** between the contact surfaces of the two connectors.

   ![Connectors Coincidence](/images/mechanic_images/week2/assembly/pin_to_pin_process_video_coincidence.png)

3. Apply a **`Coaxiality Constraint`** between small pin (blue pin) and the shared cavity of the two connectors.

4. Apply a **`Coincidence Constraint`** between the external surfaces of the rear connector and the bases of the pin.

   ![pin coincidence](/images/mechanic_images/week2/assembly/coincidence_pin_to_pin_1.png)

   ![pin coincidence](/images/mechanic_images/week2/assembly/coincidence_pin_to_pin_2.png)

5. Repeat this process for the remaining two connectors.

   #### <span style="color: #DB6A8F; padding-left: 8px;">**Result**</span>
   ![After fixing](/images/mechanic_images/week2/assembly/all_racords_fixed.png)
---

### <span style="color: #9AC8EB;">**Step 4: Fixing the Last Pin**</span>

Fix the second pin to the last free axial end of the final connector.

   ![After fixing](/images/mechanic_images/week2/assembly/last_pin.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
Follow the same procedure as in Step 2 to attach the second pin.

<span style="color: #CE6A6B;">**Coaxiality**</span>
![After fixing](/images/mechanic_images/week2/assembly/fix_second_pin_1.png)

<span style="color: #CE6A6B;">**Coincidence**</span>
![After fixing](/images/mechanic_images/week2/assembly/fix_second_pin_2.png)

---

### <span style="color: #DB6A8F; padding-left: 8px;">**Step 5: Applying Angle Values and Determining the Center of Gravity**</span>

Now, we will apply different angle values (`A`, `B`, and `C`) at the joints between connectors and determine the center of gravity for each configuration.

![Angles to set](/images/mechanic_images/week2/assembly/angle_to_set.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>

1. Select in `Constraint`, the option **`Add Mate`**, then choose the faces forming the angles.
2. Apply an angular dimension with the specified values.

   <span style="color: #CE6A6B;">**First Angle**</span>
   ![Angulation 1](/images/mechanic_images/week2/assembly/angulation_1_1.png)

   <span style="color: #CE6A6B;">**Second Angle**</span>
   ![Angulation 2](/images/mechanic_images/week2/assembly/angulation_1_2.png)

   <span style="color: #CE6A6B;">**Third Angle**</span>
   ![Angulation 2](/images/mechanic_images/week2/assembly/angulation_1_3.png)

3. Adjust the angles as needed. Use the **`Reverse Dimension`** option if the orientation is incorrect.

   ![Adjusting Angles](/images/mechanic_images/week2/assembly/ajuster_angle.png)


> <span style="color: #CE6A6B;">**Important Note:**</span>
> Ensure the correct orientation of the connectors. A common mistake is neglecting the fact that the first connector is aligned along the X-axis.
>
> ![Aligning with X-Axis](/images/mechanic_images/week2/assembly/alignement_in_x.png)
>
> To fix this, apply a **`Colinearity Constraint`** between the front plane (already aligned along X) and the opposing face of the connector.
>   ![Aligning with X-Axis](/images/mechanic_images/week2/assembly/make_aligment_with_x.png)

---

### <span style="color: #DB6A8F; padding-left: 8px;">**Step 6: Determining the Mass**</span>

Finally, determine the center of gravity for the assembly based on the applied angle values.

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>

1. Click on **`Evaluate > Mass Properties`**.

   ![Mass Determination](/images/mechanic_images/week2/assembly/assembly_2_mass_determining.png)

2. Record the coordinates of the center of mass.

<span style="color: #CE6A6B;">**Results:**</span>
- For `(A, B, C) = (25°, 125°, 130°)`:
  - Center of Gravity: `(X = 348.66, Y = -88.48, Z = -91.40)`
- For `(A, B, C) = (30°, 115°, 135°)`:
  - Center of Gravity: `(X = 327.67, Y = -98.36, Z = -102.91)`

> <span style="color: #CE6A6B;">**Tip:**</span>
> To switch between angle configurations, delete the existing angular constraints and repeat the process with new values.
> ![change angular](/images/mechanic_images/week2/assembly/angles_values_changing.png)
---

## <span style="color: #08C5D1;">**Additional Notes**</span>

- <span style="color: #CE6A6B;">**Best Practices:**</span>
  - Always double-check constraints and alignments before proceeding to the next step.
  - Use the **`Reverse Dimension`** option to correct orientation issues.
  - Ensure all pieces are properly constrained to avoid misalignment during assembly.

- <span style="color: #CE6A6B;">**Troubleshooting:**</span>
  - If the center of gravity seems incorrect, verify the alignment of the first connector along the X-axis.
  - Ensure all angular constraints are applied correctly and consistently.
  - The first connection shoulb be aligned to X axis
  - you couldn't use global variables to set angular contraints

---
