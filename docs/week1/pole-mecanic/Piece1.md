# <span style="color: #949CDF; font-size: 24px;">**Part I Construction Documentation**</span>

![](/images/piece1/Pince_001.png)

---

## <span style="color:rgb(229, 63, 99); border-left: 4px solid rgb(229, 63, 99); padding-left: 8px;">**Method Using Successive Circles and Cuts**</span>

### <span style="color: rgb(6, 91, 249);">First Step: Creating the Three Base Circles</span>

**Description**  
This step involves creating the three main circles to establish the base structure (see following  screenshot) of the part: two lateral extensions and a central body.

![](/images/piece1/Pince_002.png)
![](/images/piece1/Pince_003.png)

**Actions**  
1. Select the XY plane in SolidWorks.
   ![](/images/piece1/plan_choisie.png)

2. Draw the **central circle** with radius 75 mm at position (0, 0):
   
   - select circle icon at the top left corner
  
   ![](/images/piece1/circle_icon.png)

   - design your circle from choosen position
  
   ![](/images/piece1/cercle.png)

   - Use the smart cotation to set a raduis
  
   ![](/images/piece1/cotation_intelligent.png)![](/images/piece1/cotation_result.png)

3. Draw the **left circle**:
   - Radius: 40 mm
   - Position: -150 mm from central circle center on X-axis
  
   ![](/images/piece1/Leftimage.png)
  
4. Draw the **right circle**:
   - Radius: 40 mm
   - Position: +150 mm from central circle center on X-axis 

**Result**  
Three circles positioned to form the dumbbell-shaped base geometry.  
 **Screenshot 1:**  
![](/images/piece1/Pince_004.png)
![](/images/piece1/Pince_005.png)

---

### <span style="color: rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Second Step: Adding Transition Circles</span>

**Description**  
Creation of additional circles to define transition zones and rounded shapes between main elements. The circles allow application of curvature radius in transition areas.

![](/images/piece1/Pince_006.png)
![](/images/piece1/Pince_007.png)
![](/images/piece1/Pince_008.png)

**Actions**  
1. Create new transition circles:
   - Intermediate circles for smooth connections
   - Radius: 75mm
   - Strategic positioning for organic curves
2. Apply tangent properties to circles

![](/images/piece1/Pince_009.png)
![](/images/piece1/Pince_010.jpeg)

---

### <span style="color: rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Third Step: Cutting Operations to Form Transitions</span>

**Description**  
Using additional circles to cut and recreate the part's characteristic rounded shapes.

**Actions**  
1. **First series of cuts**:
   - Select circles to use as cutting tools
   - Select "Fit Entities" tool 
     ![](/images/piece1/Pince_011.png)
   - Remove excess circle material
  
2. **Second series of cuts**:
   - Refine transition shapes
   - Create organic curves
   - Eliminate unwanted intersections

**Result**  
Dumbbell-shaped organic form with smooth curved transitions between central body and extensions.  
 **Screenshot 3:**  
![](/images/piece1/Pince_012.png)

---

### <span style="color: rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Fourth Step: Applying Symmetry Relative to Central Circle's (O,J) Axis</span>

![](/images/piece1/Pince_013.png)

**Description**  
Mirroring the completed portion relative to the central circle's (O,J) axis to facilitate reproduction of transition curves.

**Actions**  
1. Select portions to mirror
2. Select symmetry axis
3. Apply "Mirror Entities" function to selected elements  
   ![](/images/piece1/Pince_014.png)

**Result (See Screenshot 4)**  
![](/images/piece1/Pince_015.png)

---

### <span style="color: rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Fifth Step: Base Extrusion</span>

**Description**  
Transforming the obtained 2D part into a 3D component.

**Actions**  
1. Select the three sketched circles
2. Use extrusion function  
   ![](/images/piece1/Pince_016.png)
3. Set extrusion height to 20.00 mm  
   ![](/images/piece1/Pince_017.png)
4. Extrude perpendicular to XY plane (along Z-axis)

**Result**  
Formation of extruded part.  
 **Screenshot 4**  
![](/images/piece1/Pince_018.png)
![](/images/piece1/Pince_019.jpeg)

---

### <span style="color:rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Sixth Step: Creating Main Central Hole</span>

**Description**  
Drilling the large central circular hole that forms the part's main feature, creating the hollowed area at the central circle.

![](/images/piece1/Pince_020.png)

**Actions**  
1. Select top face of central portion
2. Draw centered circle:
   - Diameter: Ø135.6 mm
   - Position: geometric center of central portion
3. Use through-hole tool  
   ![](/images/piece1/Pince_021.jpeg)
4. Drill through entire thickness (10 mm)  
   ![](/images/piece1/Pince_022.png)

**Measurements**  
- Depth: 10 mm (through)

**Result**  
Central circular through-hole in main part portion.  
 **Screenshot 5 Location:**  
![](/images/piece1/Pince_023.jpeg)
![](/images/piece1/Pince_024.jpeg)

---

### <span style="color: rgb(6, 91, 249);  padding: 4px 8px; border-radius: 4px;">Seventh Step: Finishing and Material Properties</span>

![](/images/piece1/Pince_025.png)

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
![](/images/piece1/Pince_026.jpeg)
![](/images/piece1/Pince_027.jpeg)
![](/images/piece1/Pince_028.png)

**Final Measurements**  
- **Material:** AISI 1020 Steel (density 0.0079 g/mm³)  
- **Obtained mass:** 2834.97 grams  