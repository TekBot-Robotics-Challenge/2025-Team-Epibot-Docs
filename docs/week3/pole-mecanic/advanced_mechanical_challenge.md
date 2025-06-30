# <span style="color: #5784BA;">**Documentation for Advanced Mechanical Design Challenge**</span>

---

## <span style="color: #08C5D1;">**Challenge Overview**</span>

![Opening SolidWorks](/images/mechanic_images/week3/label_4.png)

The Tekbot Robotics Challenge 2025 (TRC2K25) is a rigorous competition designed to evaluate the technical skills of participating teams across multiple disciplines: Electronics, IT, and Mechanics. Over the course of five weeks, teams will face weekly challenges that test their expertise in these domains. Each challenge builds upon the previous one, culminating in a final multidisciplinary project where all sub-teams must collaborate to solve a complex problem.

This documentation focuses on the **`Mechanical Design Challenge`**, specifically the advanced level, which requires participants to design and model a complex mechanical part using SolidWorks. The objective is to create a functional and precise 3D model, calculate its mass for various configurations, and ensure compliance with specified dimensions and constraints.

---

### <span style="color: #28B463;">**Link to Download**</span>: [Link to download](/pieces/week_3_pieces.zip)

## <span style="color: #08C5D1;">**Implementation Steps**</span>

### <span style="color: #9AC8EB;">**Step 1: Opening SolidWorks and Setting Up Global Variables**</span>

To begin, open SolidWorks in **`Part Mode`**. This step is crucial as it sets the foundation for the entire design process.

![Opening SolidWorks](/images/mechanic_images/week3/ouvir_solidwork.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Define Global Variables**</span>
Most dimensions in this design depend on global variables, which can significantly impact the geometry and, consequently, the final mass. To define these variables:
1. Navigate to **`Tools > Equations`**.
2. In the table that appears, insert the required variables and assign them initial values based on the problem statement.
   - Example Variables: `A`, `B`, `W`, `X`, `Y`, `Z`.

   ![Global Variables Exemple](/images/mechanic_images/week3/globales_variables_exemples.png)

   - Assign default values for the first configuration:  
     - `A = 193 mm`  
     - `B = 88 mm`  
     - `W = B/2 mm`
     - `X = A/4 mm`  
     - `Y = B + 5.5 mm` 
     - `Z = B + 15 mm`

![Global Variables Table](/images/mechanic_images/week3/global_variables_process.png)

> <span style="color: #CE6A6B;">**Detailed Explanation:**</span>
> Using global variables ensures that all dimensions are dynamically linked. This means that when you update a variable (e.g., changing `A` from 193 mm to 205 mm), the entire model updates automatically. This approach not only saves time but also minimizes errors during iterative design processes.

---

### <span style="color: #9AC8EB;">**Step 2: Creating the Prismatic Sections**</span>

The design begins with the creation of prismatic sections, which serve as the foundational structure of the part. These sections are modeled as primitive shapes, such as cubes and rectangular prisms, before being refined into the final geometry.

![Prismatic parts to do](/images/mechanic_images/week3/first_part_to_do.png)

![Rectangle Dimensions](/images/mechanic_images/week3/first_cotation_exemple_1.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
1. **`Select the Top Plane`** from the left-hand sidebar.
2. Click on the **`Rectangle Tool`** in the top toolbar under "Basic Modeling Tools."

![Rectangle Dimensions](/images/mechanic_images/week3/first_prisme_drawing_step_1.png)

3. Draw a rectangle and apply the following dimensions:
   - Height: `2 * X`
   - Width: `B`

![Rectangle Dimensions](/images/mechanic_images/week3/first_prisme_drawing_step_2.png)

4. Extrude the rectangle by `B` mm:
   - Go to **`Features > Extruded Boss/Base`**.
   - Set the height to `B`.

![Extrusion Process](/images/mechanic_images/week3/first_prisme_drawing_step_3.png)

5. Draw a construction line horizontally to serve as a reference for the second prism.

![Extrusion Process](/images/mechanic_images/week3/first_prisme_drawing_step_4.png)

6. Draw a square starting from this line and dimension it to `40 mm`.


7. Apply a distance of `A` mm between the two rectangles.

![Extrusion Process](/images/mechanic_images/week3/cotation_A_between_rectangles.png)


![Extrusion Process](/images/mechanic_images/week3/cotation_A_between_rectangles_drawing.png)

8. Extrude the square by `Y` mm with an offset of `40 mm` from the sketch plane.

![Square Extrusion](/images/mechanic_images/week3/square_first_extrusion_to_do.png)

![Square Extrusion](/images/mechanic_images/week3/square_first_extrusion.png)

> <span style="color: #CE6A6B;">**Detailed Explanation:**</span>
> The use of construction lines and precise dimensioning ensures that the prismatic sections are aligned correctly. This alignment is critical for maintaining symmetry and balance in the final design.

---

### <span style="color: #9AC8EB;">**Step 3: Adding the Chamfer-Like Inclination**</span>

The design includes a slight chamfer-like inclination between the two prisms. This feature adds complexity to the model and tests the participant's ability to handle intricate geometries.

![Chamfer-Like Inclination](/images/mechanic_images/week3/faux_chanfrin_prisme.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
1. Analyze the top view to identify the key points:
   - The endpoints of the horizontal median of the small square.
   - Points located `6 mm` from the horizontal median of the large rectangle.
2. Draw a construction line connecting these points.
3. Extrude the excess material using the **"Select All"** option and orient the direction correctly.
4. Apply symmetry to replicate the inclination on the opposite face.

<span style="color: #DB6A8F; padding-left: 8px;">**Workshop**</span>
<iframe src="https://player.vimeo.com/video/1095340214?h=b21ca6bbb2&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px;" title="process-to-make-inclinaison"></iframe>

> <span style="color: #DB6A8F; padding-left: 8px;">**Detailed Explanation:**</span>
> The chamfer-like inclination demonstrates the participant's ability to manipulate geometric relationships. By carefully aligning the construction lines and applying symmetry constraints, the design achieves both aesthetic appeal and functional precision.

---

### <span style="color: #9AC8EB;">**Step 4: Connecting the Two Prisms**</span>

Next, we will create a base structure to connect the two prismatic sections. This step involves designing a bridge-like feature that ensures structural integrity while maintaining the overall symmetry of the part.

![Prism connection to do](/images/mechanic_images/week3/liaison_basale_prisme_to_do.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
1. Draw three broken lines approximating the shape of the base connection.
2. Apply the required dimensions to ensure accuracy.

![Base Connection Drawing](/images/mechanic_images/week3/linking_drawing_cotation.png)

3. Extrude the sketch:
Use a middle plane with a depth of `200 mm` and a thickness of `6 mm`.
This approach minimizes errors by allowing us to refine the extrusion later.

![Base Connection Extrusion](/images/mechanic_images/week3/linking_drawing_extrusion_1.png)

4. Select the necessary portion of the generated surface.

![Part to conserve](/images/mechanic_images/week3/part_necessary_for_base_choosen.png)

5. Extrude the selected area using **Boss/Base Extrude**.

![Chosen part conserve](/images/mechanic_images/week3/linking_drawing_extrusion_2.png)

6. Use the **`Combine Feature`** to retain only the intersection between the new extrusion and the previously generated surface.

<span style="color: #CE6A6B;">**Workshop**</span>
<iframe src="https://player.vimeo.com/video/1095340211?h=f9a5f46b85&amp;title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" width="100%" height="300px" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" title="process-to-link"></iframe>

> <span style="color: #CE6A6B;">**Detailed Explanation:**  </span>
> The base connection serves as a critical link between the two prisms. By carefully selecting and refining the extruded surfaces, we ensure that the final design is both structurally sound and visually appealing.

---

### <span style="color: #9AC8EB;">**Step 5: Building the Vertical Wall**</span>

The vertical wall is an essential component of the design, providing additional structural support and housing features such as mounting points or attachment mechanisms.

![Vertical Wall To Do](/images/mechanic_images/week3/muraille_to_draw.png)

#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
1. Draw the upper border of the wall using lines, ensuring it is parallel to the base.
2. Use the **`Rib Feature`** to create a spatial configuration with a thickness of `5 mm`.

![Wall and Fillets](/images/mechanic_images/week3/nervure_extrusion.png)

3. Apply fillets (`R = 10 mm`) to all relevant edges.

![Wall and Fillets](/images/mechanic_images/week3/conge_application_apres_muraille.png)

> <span style="color: #CE6A6B;">**Detailed Explanation:**</span>
> The vertical wall demonstrates the participant's ability to integrate multiple features into a single design. By combining ribs, fillets, and precise dimensioning, the wall achieves both functionality and aesthetic appeal.

<span style="color: #CE6A6B;">**workshop**</span>

<iframe src="https://player.vimeo.com/video/1095340201?h=cdecb44c0e&amp;title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" width="100%" height="300px"  frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" title="muraille-extrem-border-drawing"></iframe>

---

### <span style="color: #9AC8EB;">**Step 6: Creating Rectangular Cavities at the Ends**</span>

![Cavity to draw](/images/mechanic_images/week3/cavity_to_draw.png)

The design includes rectangular cavities at the ends of the prismatic sections. These cavities reduce the overall weight of the part while maintaining structural integrity.


#### <span style="color: #DB6A8F; padding-left: 8px;">**Process**</span>
1. Draw rectangles centered on the faces of each prism.
2. Extrude-cut the rectangles through the entire height of the prisms.

**workshop**
<iframe src="https://player.vimeo.com/video/1095340205?h=e624c36d52&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" style="top:0;left:0;width:100%;height:300px;" title="persing-prismique"></iframe>

> <span style="color: #CE6A6B;">**Detailed Explanation:**</span>
> The cavities are strategically placed to optimize the part's weight-to-strength ratio. By carefully dimensioning and positioning these features, the design achieves a balance between performance and efficiency.

---

### <span style="color: #9AC8EB;">**Step 7: Forming Features AA and BB**</span>

Features AA and BB are unique components of the design, requiring careful attention to detail and precise execution.

![AA and BB Part](/images/mechanic_images/week3/formations_AA_and_BB_to_done.png)

#### <span style="color: #9AC8EB;">**Process**</span>
1. Draw a rectangle with the specified dimensions.
2. Extrude it symmetrically across the wall for uniformity.
3. Apply a fillet (`R = 10 mm`) to the contoured edge.
4. Draw a circle (`D = 10 mm`) centered on the fillet and perform a cut-extrude through the entire structure.

<span style="color: #CE6A6B;">**Workshop**</span>
<iframe src="https://player.vimeo.com/video/1095340156?h=f8c9f1f1d8&amp;title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" width="100%" height="300px" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" title="BB_construct_process"></iframe>

> <span style="color: #CE6A6B;">**Detailed Explanation:** </span>
> Features AA and BB showcase the participant's ability to integrate complex geometries into the design. By combining extrusions, fillets, and cut-outs, these features demonstrate both creativity and technical proficiency.

---

### <span style="color: #9AC8EB;">**Step 8: Assigning Material**</span>

Assign the material **`Aluminum Alloy 1060`** to the part:
1. Go to **`Material > Edit Material`**.
2. Select **`Aluminum Alloy 1060`** from the list and confirm.

![Material Assignment](/images/mechanic_images/week3/masse_determining.png)

> <span style="color: #CE6A6B;">**Detailed Explanation:**</span>
> Material selection is a critical step in the design process. Aluminum Alloy 1060 was chosen for its lightweight properties and excellent strength-to-weight ratio, making it ideal for this application.

---

### <span style="color: #9AC8EB;">**Step 9: Determining the Mass**</span>

Finally, calculate the mass of the part for the given tuples of variable values:
1. Ensure the correct values are assigned to the global variables.
2. Go to **`Evaluate > Mass Properties`**.

![Mass Calculation](/images/mechanic_images/week3/masse_1_determining.png)

<span style="color: #CE6A6B;">**Results:**</span>

- For `(A, B, W, X, Y, Z) = (193, 88, B/2, A/4, B+5.5, B+15)` → **Mass = 1400.64 g**
- For `(A, B, W, X, Y, Z) = (205, 100, B/2, A/4, B+5.5, B+15)` → **Mass = 1651.40 g**
- For `(A, B, W, X, Y, Z) = (210, 105, B/2, A/4, B+5.5, B+15)` → **Mass = 1760.41 g**

> <span style="color: #CE6A6B;">**Detailed Explanation:**</span>
> The mass calculation is a key metric for evaluating the success of the design. By iterating through different configurations and comparing the results, participants can assess the impact of each variable on the final design.

---

## <span style="color: #08C5D1;"> **Additional Notes**</span>
  - Always double-check dimensions and constraints before proceeding to extrusion.
  - Use construction lines and geometric relations (e.g., symmetry, coincidence) to maintain precision.
  - Regularly save intermediate versions of your file after major steps.