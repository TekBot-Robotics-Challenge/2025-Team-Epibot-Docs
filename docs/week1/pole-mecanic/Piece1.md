# <span style="color: #949CDF; font-size: 24px;">**Part I Construction Documentation**</span>

![](/images/mechanic_images/week1/piece1/piece1_plan.png)

---


### <span style="color: #28B463;">**Short Demo**</span>

<video width="320" height="240" controls>
  <source src="/videos/mechanic_videos/week1/piece1_simulation.mp4" type="video/mp4">
  Votre navigateur ne supporte pas la balise vidéo.
</video>

---

### <span style="color: #28B463;">**Link to Download**</span>: <a href="/pieces/week_1_pieces.zip" download>**Pieces**</a>


## <span style="color:rgb(229, 63, 99); border-left: 4px solid rgb(229, 63, 99); padding-left: 8px;">**Method Using Successive Circles and Cuts**</span>

### <span style="color: rgb(6, 91, 249);">First Step: Creating the Three Base Circles</span>

**Description**  
This step involves creating the three main circles to establish the base structure (see following  screenshot) of the part: two lateral extensions and a central body.

![](/images/mechanic_images/week1/piece1/circles_to_draw.png)

**Actions**  
1. Select the XY plane in SolidWorks.
   ![](/images/mechanic_images/week1/piece1/plan_choisie.png)

2. Draw the **central circle** with radius 75 mm at position (0, 0):
   
   - select circle icon at the top left corner
  
   ![](/images/mechanic_images/week1/piece1/circle_icon.png)

   - design your circle from choosen position
  
   ![](/images/mechanic_images/week1/piece1/cercle.png)

   - Use the smart cotation to set a raduis
  
   ![](/images/mechanic_images/week1/piece1/cotation_intelligent.png)![](/images/mechanic_images/week1/piece1/cotation_result.png)

3. Draw the **left circle**:
   - Radius: 40 mm
   - Position: -150 mm from central circle center on X-axis
  
   ![](/images/mechanic_images/week1/piece1/Leftimage.png)
  
4. Draw the **right circle**:
   - Radius: 40 mm
   - Position: +150 mm from central circle center on X-axis 

**Result**  
Three circles positioned to form the dumbbell-shaped base geometry.  
 **Screenshot 1:**

![](/images/mechanic_images/week1/piece1/Three_circles.png)

---

### <span style="color: rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Second Step: Adding Transition Circles</span>

**Description**  
Creation of additional circles to define transition zones and rounded shapes between main elements. The circles allow application of curvature radius in transition areas.

![](/images/mechanic_images/week1/piece1/transition_to_draw.png)

**Actions**  
1. Create new transition circles:
   - Intermediate circles for smooth connections
   - Radius: 75mm
   - Strategic positioning for organic curves
2. Apply tangent properties to circles

![](/images/mechanic_images/week1/piece1/transition_circles.jpeg)

---

### <span style="color: rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Third Step: Cutting Operations to Form Transitions</span>

**Description**  
Using additional circles to cut and recreate the part's characteristic rounded shapes.

**Actions**  
1. **First series of cuts**:
   - Select circles to use as cutting tools
   - Select "Fit Entities" tool

     ![](/images/mechanic_images/week1/piece1/entity_ajusting_icon.png)

   - Remove excess circle material
  
2. **Second series of cuts**:
   - Refine transition shapes
   - Create organic curves
   - Eliminate unwanted intersections

**Result**  
Dumbbell-shaped organic form with smooth curved transitions between central body and extensions.  
 **Screenshot 3:**  
![](/images/mechanic_images/week1/piece1/circle_fitting.png)

---

### <span style="color: rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Fourth Step: Applying Symmetry Relative to Central Circle's (O,J) Axis</span>


**Description**  
Mirroring the completed portion relative to the central circle's (O,J) axis to facilitate reproduction of transition curves.

**Actions**  
1. Select portions to mirror
2. Select symmetry axis
3. Apply "Mirror Entities" function to selected elements  
![](/images/mechanic_images/week1/piece1/symetrie_icon.png)


**Result (See Screenshot 4)**  

---

### <span style="color: rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Fifth Step: Base Extrusion</span>

**Description**  
Transforming the obtained 2D part into a 3D component.

**Actions**  
1. Select the three sketched circles
2. Use extrusion function

   ![](/images/mechanic_images/week1/piece1/extrusion_icon.png)

3. Set extrusion height to 20.00 mm  

   ![](/images/mechanic_images/week1/piece1/extrusion_parameters.png)

4. Extrude perpendicular to XY plane (along Z-axis)

**Result**

Formation of extruded part.  

 **Screenshot 4**  
![](/images/mechanic_images/week1/piece1/screenshoot4.jpeg)

---

### <span style="color:rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Sixth Step: Creating Main Central Hole</span>

**Description**  
Drilling the large central circular hole that forms the part's main feature, creating the hollowed area at the central circle.

![](/images/mechanic_images/week1/piece1/central_circular_hole.png)

**Actions**  
1. Select top face of central portion
2. Draw centered circle:
   - Diameter: Ø135.6 mm
   - Position: geometric center of central portion
3. Use through-hole tool

   ![](/images/mechanic_images/week1/piece1/removing_extrusion_icon.jpeg)

4. Drill through entire thickness (10 mm)

   ![](/images/mechanic_images/week1/piece1/removing_extrusion_parameters.png)

**Measurements**  
- Depth: 10 mm (through)

**Result**  
Central circular through-hole in main part portion.  
 **Screenshot 5 Location:**  
![](/images/mechanic_images/week1/piece1/central_circular_hole.png)

![](/images/mechanic_images/week1/piece1/central_circular_hole.png)

---

### <span style="color: rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Seventh Step: Finishing and Material Properties</span>

**Description**  
Applying final finishes, verifications, material property definition, and mass determination.

**Actions**  
1. **Geometric verification**:
   - Dimension control per technical drawing 
   - Check for geometric defects
   - Create part drawing
2. **Material application**:
   - Selection: AISI 1020 Steel
   - Density: 0.0079 g/mm³
   - Automatic mass calculation
   - Final adjustments

 **Screenshots 6:**  
![](/images/mechanic_images/week1/piece1/material.jpeg)

![](/images/mechanic_images/week1/piece1/finitions.jpeg)

![](/images/mechanic_images/week1/piece1/mass_finding.png)

**Final Measurements**  
- **Material:** AISI 1020 Steel (density 0.0079 g/mm³)  
- **Obtained mass:** 2834.97 grams
   ---