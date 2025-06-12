# <span style="color: #949CDF; font-size: 28px; font-weight: bold;">PIECE 2 REALIZATION</span>

![Part Preview](/images/piece2/image1.png)

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">TOOLS USED AND THEIR ROLES</span>

- **<span style="color: #34A853;">Extruded Boss/Base</span>**: Creates solid volume by extruding 2D sketches  
- **<span style="color: #34A853;">Extruded Cut</span>**: Removes material from 3D solids  
- **<span style="color: #34A853;">Smart Dimension</span>**: Adds/modifies dimensions (lengths, diameters, angles)  

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">INITIAL SETUP</span>

![Setup Step 1](/images/piece2/image2.png)
![Setup Step 2](/images/piece2/image3.png)

1. Click application icon → File → Part
2. Select upper plane

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">CREATING 6 CIRCLES & 2 ARCS</span>

![Circle Creation](/images/piece2/image4.png)
![Arc Creation](/images/piece2/image5.png)
![Dimensioning](/images/piece2/image6.png)
![Result Preview](/images/piece2/image7.png)
![Tool Location](/images/piece2/image8.png)
![Final Sketch](/images/piece2/image9.png)

**Steps:**
1. Select **Circle** tool
2. Draw:
   - 4 concentric circles (shared center)
   - 2 additional concentric circles (second center)
3. Use **Smart Dimension** to:
   - Resize each circle
   - Set distance between centers
4. Create arcs with **Centerpoint Arc** tool
5. Dimension arcs per specifications

> 💡 Refer to images for visual guidance

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">2D TO 3D CONVERSION (CIRCLES)</span>

![Extrusion Interface](/images/piece2/image10.png)
![Selection](/images/piece2/image11.png)
![Completion](/images/piece2/image12.png)

1. Click **Extruded Boss**
2. Set parameters per task requirements
3. Select contours:
   - 64mm diameter circle
   - 38mm diameter circle
4. Confirm with green checkmark ✓

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">2D TO 3D CONVERSION (ARCS)</span>

![Arc Extrusion](/images/piece2/image13.png)
![Parameters](/images/piece2/image14.png)

1. Click **Extruded Boss**
2. Select arc centers
3. Enter task-specified values
4. Confirm with green checkmark ✓

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">CREATING CENTRAL VOID</span>

![Cut Setup](/images/piece2/image15.png)
![Cut Result](/images/piece2/image16.png)

1. Select **Extruded Cut**
2. Set direction: "Through both"
3. Select void contours
4. Confirm removal

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">43MM DIAMETER BOSS CREATION</span>

![Boss Setup 1](/images/piece2/image17.png)
![Boss Setup 2](/images/piece2/image18.png)
![Selection](/images/piece2/image19.png)
![Parameters](/images/piece2/image20.png)
![Result](/images/piece2/image21.png)

1. Click **Extruded Boss**
  
2. Select:
   - 43mm diameter circle
   - 34mm diameter circle
3. Change "From" setting:
   - Sketch plane → Surface/plane/face
4. Select target face
5. Confirm with ✓

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">5MM RECESS CREATION</span>

![Recess Setup 1](/images/piece2/image22.png)
![Recess Setup 2](/images/piece2/image23.png)
![Selection](/images/piece2/image24.png)
![Result](/images/piece2/image25.png)

1. Click **Extruded Cut**
2. Select contours:
   - 55mm diameter circle
   - 43mm diameter circle
3. Set depth: 5mm
4. Change "From":
   - Sketch plane → Surface/plane/face
5. Select target face
6. Confirm with ✓

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">MATERIAL ASSIGNMENT</span>

![Material Selection](/images/piece2/image26.png)
![Material Application](/images/piece2/image27.png)

1. Right-click "Material"
2. Select "Edit Material"
3. Search desired material
4. Apply selection

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">FINAL RESULT</span>

**Without Color:**  
![Final Plain](/images/piece2/image28.png)

**With Color (Optional):**  
![Final Colored](/images/piece2/image29.png)

> 🎨 To colorize: Right-click face → Select color sphere → Choose color → Confirm

---

## <span style="color: #949CDF; border-bottom: 2px solid #4285F4; padding-bottom: 4px;">MASS CALCULATION</span>

![Mass Properties](/images/piece2/image30.png)
![Mass Results](/images/piece2/image31.png)

1. Click **Mass Properties**
2. Read results (grams)
3. Convert to kg: divide by 1000  
   **Final Mass:** 0.28415 kg