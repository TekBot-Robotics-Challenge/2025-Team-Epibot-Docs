# <span style="color: #08C5D1;">Documentation of the Mechanical Design of the Conveyor System</span>

The objective of this challenge was to bring together all the skills we acquired during previous challenges to design an intelligent and efficient autonomous conveyor. To achieve this, we first brainstormed and sketched out ideas for what the conveyor would look like. The conveyor had to be 650mm long and 100mm above the ground. It also had to be modeled in SolidWorks. The process took two directions: the ideation phase, marked by virtual design, and the physical construction phase. This documentation details the process we followed during these phases, the mechanical constraints we encountered, and how we overcame them.

## <span style="color: #3498DB;">Virtual Design Phase</span>
### <span style="color: #1ABC9C;">Overview of the Virtual Design Process</span>

The virtual design focused on SolidWorks for 3D modeling and Illustrator for sketches intended for laser cutting. The conveyor had to be 650mm long, with the belt positioned 100mm above the ground. First, we had to think about how our conveyor would stand. We opted for two parallel vertical supports connected by horizontal planks to ensure balance. A final horizontal plate was added to keep the entire structure fixed and sturdy.

<iframe src="https://player.vimeo.com/video/1104886435?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" width="100%" height="300px" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" title="vue_3d_convoyeur_soilid_work" controls autoplay muted></iframe>

### <span style="color: #1ed3afff;"> Support </span>
<iframe src="https://player.vimeo.com/video/1104886485?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="vue_3d_support_convoyeur" controls autoplay muted></iframe>

### `Key Components and Design Process`:

- #### <span style="color: #DB6A8F">Vertical Support</span>

<iframe src="https://player.vimeo.com/video/1104886493?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" width="100%" height="300px" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" title="vue_3d_support_verticale" controls autoplay muted></iframe>

This is the vertical face that supports the entire assembly. It has a height of 115mm to comply with the challenge requirements. The 1mm difference is to provide a small margin. Initially, we planned to use wood, but the workspace of our laser cutter is limited to 600x300mm boards. Therefore, instead of 650mm plates, we opted for 590mm, giving us a 60mm margin.

##### <span style="color: #2fbaf1ff;"> Features: </span>
Two rows of small cavities to mount horizontal supports.
Circular holes at the ends (30mm diameter) for inserting bearings to smooth the drum's rotational movement.

- <span style="color: #DB6A8F;">Horizontal Support:</span>

<iframe src="https://player.vimeo.com/video/1104886388?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="support_horrizontal_face" controls autoplay muted></iframe>

These are two surfaces placed on the two rows of cavities carved into the vertical supports, perpendicular to them, maintaining balance. They also act as stabilizers for the conveyor belt, preventing overloads and blockages. They are positioned around the drum for easy deployment above and below the belt. We chose a spacing of 70mm between the plates, making the support measure (70 + 12 * 2) 94mm wide (including the interlocking parts) and 450mm long.


#### <span style="color: #DB6A8F;">Base Support:</span>

<iframe src="https://player.vimeo.com/video/1104885921?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="base_vue_d" controls autoplay muted></iframe>

This is the base on which the entire assembly rests. It consists of cavities to hold the vertical supports. Due to the laser cutter's workspace limitations, we segmented it into two complementary surfaces. In addition to supporting the structure, it serves as a base for components under the conveyor and allows us to extend the missing 60mm to reach the total length of 650mm.

`Assembly Process`:

<iframe src="https://player.vimeo.com/video/1104886159?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="support_assembly" controls autoplay muted></iframe>

### <span style="color: #3498DB;">Conveyor Mobility System</span>

<iframe src="https://player.vimeo.com/video/1104894020?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="vue_3d_assembly_2" controls autoplay muted></iframe>

After defining a robust and balanced structure for the conveyor, we moved on to designing a system to enable waste mobility. The key component generating movement is a NEMA 17 motor. Initially, we had two drums aligned with the bearings. A leather belt was placed over the drums, serving as the moving surface. At this stage, the drum's movement drives the belt, which, like a pulley, moves all objects it carries.

#### <span style="color: #DB6A8F;">Drum Design:</span>

<iframe src="https://player.vimeo.com/video/1100264137?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="tambour-vue-3d" controls autoplay muted></iframe>

The drum is the cylindrical body in direct contact with the belt, transmitting motion. Its longitudinal ends fit into the bearings at the ends of the vertical supports. The main cylinder is 30mm wide, flanked by two 40mm-high discs to prevent overflows. There are two types:
- `Leader Drum`: One end is longer (23/15mm) to pass through the 12mm-thick vertical surface and connect to the motor.
- `Follower Drum`: Both ends are 15mm long.


#### <span style="color: #DB6A8F;"> Process of Conception:</span>

<iframe src="https://player.vimeo.com/video/1100285370?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="tambour-design-video_AZAHWH1h" controls autoplay muted></iframe>

### <span style="color: #1ed3afff;">Energy Transmission System:</span>

This video https://www.youtube.com/watch?v=5Xn26kxmX5U helped us explore various possibilities. Initially, we opted for a pulley-belt system, but we lacked customizable belts. We experimented with elastic bands, string, and chain systems, but these failed due to mechanical constraints, friction, or lack of adhesion. Finally, we opted for a safer solution: directly connecting the motor to the drum via a custom-designed `Motor Support`.

<iframe src="https://player.vimeo.com/video/1104886392?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="support_moteur_vue_3d" controls autoplay muted></iframe>

**`Position`**
![position of support motor](/images/mechanic_images/week4/support_moteur_emplacement.png)

This structure allows us to fix the motor horizontally while maintaining the entire assembly vertically using four pillars in each direction.
The fixation of the motor and drum axles is ensured by a `junction`. It is a cylindrical body with two openings at its ends: one with a 5mm diameter for the NEMA motor shaft and the other with a 10mm diameter for the drum shaft. A tolerance of 0.1mm was later added to facilitate the insertion of each component into the two openings.
`Position`

![Jonction position](/images/mechanic_images/week4/emplacement_jonction_moteur_support.png)

`3D view`

<iframe src="https://player.vimeo.com/video/1104886453?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="Vue_3d_jonction" controls autoplay muted></iframe>

`Assembly Process`:

<iframe src="https://player.vimeo.com/video/1104886019?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="fication_tampour" controls autoplay muted></iframe>


<iframe src="https://player.vimeo.com/video/1104886105?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="fixation_support_moteur" controls autoplay muted></iframe>

## <span style="color: #3498DB;">Sensors and Object Detection</span>

![sensors positions](/images/mechanic_images/week4/emplacements_supports_laser.png)

To enable waste detection on the conveyor, we collaborated with the electronics team to design a `laser-phototransistor` system. The laser emits a beam captured by the phototransistor. When the beam is interrupted, it indicates the presence of an object on the conveyor. To protect the components, the electronics engineers designed PCBs.

Our mechanical challenge was to design a mechanism to support this system, ensuring precise alignment. We designed adjustable supports with sliding cylindrical trunks in a screwable base.

#### <span style="color: #1e6293ff;">Laser Port:</span>

<iframe src="https://player.vimeo.com/video/1104886142?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="port_laser_vue_D" controls autoplay muted></iframe>

This is a platform with a rectangular configuration at the top, featuring an open hollow face for laser diffusion and a rectangular opening opposite for wiring. The length is approximately the sum of the lengths of the PCB coupled with the laser.

#### <span style="color: #1e6293f0;"> Phototransistor Port:</span>

<iframe src="https://player.vimeo.com/video/1104886471?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="vue_3d_photo_résistance" controls autoplay muted></iframe>

This is a rectangular platform designed to house the phototransistor PCB. Since the laser sensor is fixed, a sliding platform was added to adjust the horizontal position of the phototransistor for perfect calibration. A fairly large opening was also provided for wiring.

#### <span style="color: #1e6293f0;">Base:</span>
<iframe src="https://player.vimeo.com/video/1104886475?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="vue_3d_socle" controls autoplay muted></iframe>

This is the main link allowing height adjustment, equipped with a cylindrical body to house the support and a 4mm opening for fixing the desired height.

`Wiring simulation`: 

<iframe src="https://player.vimeo.com/video/1104886150?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="reglage_hauteur_support" controls autoplay muted></iframe>

#### <span style="color: #1e6293f0;">The carpet:</span>

<iframe src="https://player.vimeo.com/video/1104898477?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="tapis_FwweSzvM" controls autoplay muted></iframe>

It's a surface that, as it rotates, pulls whatever it supports.
`Conception:`

<iframe src="https://player.vimeo.com/video/1104886397?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="video conception_tapis" controls autoplay muted></iframe>


### <span style="color: #1ed3afff;">Chassis Design</span>
In total, the electronics team implemented five PCBs, including those for the laser and phototransistor. Our challenge was to design a chassis to house these components, ensuring protection, functionality, and aesthetics.

![pcb done](/images/mechanic_images/week4/PCBs.png)


- `Motor Driver Box:`

<iframe src="https://player.vimeo.com/video/1104886502?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="vue_conteneur_driver" controls autoplay muted></iframe>

The motor driver is a PCB measuring 44mm in length and 57mm in width. We designed a small cubic box with two opposing openings on the lateral faces for wiring, a sliding lid, and small perforations for heat dissipation and compartment ventilation.

- `Central PCB Box`:
  
  <iframe src="https://player.vimeo.com/video/1104886426?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="vue_3d_conteneur_pas_driver" controls autoplay muted></iframe>

This is the brain of all operations on the conveyor (similar to an Arduino replica). It measures 870x74mm and must be powered by a set of four Li-ion batteries. Therefore, we designed a box with two compartments (one for the PCB and one for the power supply), always with a sliding lid. Perforations were added to allow heat dissipation.

### <span style="color: #1ABC9C;"> Download Resources: </span>

Here is the [link](/images/mechanic_images/week4/conveyor_virtual_design.zip) to download all the components.

> Note: We can observe that the distance between the mat and the ground in SolidWorks is approximately 95mm out of 100mm. This offset of 5mm is intentional because we noticed during assembly that the mat did not adhere perfectly to the upper horizontal support, resulting in a deviation of approximately ±5mm. Therefore, we reduced the measurement in SolidWorks to ensure that the mat is actually 100mm off the ground after assembly.

![+-5mm margin explication](/images/mechanic_images/week4/decalage.png)
--

`Solid Work Animation`

<iframe src="https://player.vimeo.com/video/1104914025?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="animation-convoyeur-solid-work" controls autoplay muted></iframe>

## <span style="color: #08C5D1;">Physical Construction Phase</span>

![Mounting Result](/images/mechanic_images/week4/Result_after_mountinf.jpg)

### <span style="color: #3498DB;">Obtaining the Components</span>
Once we had all the plans and parts ready, we moved on to the physical production phase. The sensor ports, drums, junctions, and other small components were 3D printed in PLA due to their small size. To achieve this, we first converted the SolidWorks files into STL format, then used the PrusaSlicer software to generate the G-code, which we sent to the printer (a Prusa MK4S Original).

#### <span style="color: #1ABC9C;">Laser Cutting</span>
The vertical and horizontal planks, as well as the basal supports, were too large for 3D printing, so they were laser-cut from 4mm-thick plywood.

![decoupe laser](/images/mechanic_images/week4/découpe_laser.jpg)

`Process:`
To cut the planks, we followed these steps:

- Opened the detailed drawings of the parts.
- Exported them in SVG format.
- Configured the cutting parameters using Adobe Illustrator.

![decoupe laser](/images/mechanic_images/week4/mise_en_plan_1.png)
![decoupe laser](/images/mechanic_images/week4/mise_en_plan_2.png)
![decoupe laser](/images/mechanic_images/week4/mise_en_plan_3.png)

For the central PCB chassis, we used an open-source model available at [Boxes Hackerspace Bamberg](https://boxes.hackerspace-bamberg.de/?language=en). We customized it by replacing the standard perforations with star-shaped and octagonal ones to give it a more "cute" aesthetic.

![Central PCB Box](/images/mechanic_images/week4/boxpy1.png)

![Central PCB Box](/images/mechanic_images/week4/boxpy2.png)

The files used for laser cutting are downloadable along with the other resources in the "laser_cutting_plans" folder. Once all this was done, we proceeded with the **`cutting`** and **`printing processes.`**

![Materials](/images/mechanic_images/week4/material.jpg)

### <span style="color: #3498DB;">Assembly of Components</span>
Once all the materials were ready, we began assembling the conveyor system. Below are the key steps we followed:

- Insertion of the Four Bearings into Their Dedicated Holes
 
![Mountage](/images/mechanic_images/week4/roulement_fixing.jpg)

- Mounting the Two Horizontal Supports into the Rows of Cavities on the Vertical Supports
  
![Mountage](/images/mechanic_images/week4/montage_verti-horrizontal_2.jpg)

- Insertion of the Drums into the Axis of the Bearings
  
![Mountage](/images/mechanic_images/week4/poulie_fixing.jpg)

- Superposition of the Second Vertical Face
  
![Mountage](/images/mechanic_images/week4/second_face.jpg)

  
- Fixation of the Color Sensor


![Mountage](/images/mechanic_images/week4/capteur_couleur fixation_2.jpg)
![Mountage](/images/mechanic_images/week4/support_capteurs_fixation.jpg)


- Fixation of the Motor onto Its Support and Then Onto the Conveyor
  
![Mountage](/images/mechanic_images/week4/support_moteur_fixation.jpg)

- Fixation of the Motor Driver Container Directly onto the Platform
- Fixation of the Sensor Bases

![Mountage](/images/mechanic_images/week4/support_capteurs_fixation.jpg)

- wiring all component to central pcb (Performed by the Electronics Team)
  
![Mountage](/images/mechanic_images/week4/compatiment_central_fixing.jpg)
  
- Calibration Test (Performed by the Electronics Team)
This test ensured that the laser beam successfully reached the phototransistor.

<iframe src="https://player.vimeo.com/video/1104885965?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="calibrage et teste de mobilité" controls autoplay muted></iframe>

### <span style="color: #3498DB;">Final Result</span>

![Mounting Result](/images/mechanic_images/week4/Result_after_mountinf.jpg)

## <span style="color: #3498DB;">Bonus: Automatic Sorting System</span>

<iframe src="https://player.vimeo.com/video/1104885940?title=0&amp;byline=0&amp;portrait=0&amp;badge=0&amp;autopause=0&amp;player_id=0&amp;app_id=58479" frameborder="0" allow="autoplay; fullscreen; picture-in-picture; clipboard-write; encrypted-media; web-share" referrerpolicy="strict-origin-when-cross-origin" style="top:0;left:0;width:100%;height:300px;" title="Bonus_TRC" controls autoplay muted></iframe>

Once the conveyor was complete, we implemented an automatic sorting system to make the process smart and autonomous. For this, we opted to build the "SO-101 Robot Arm" project from Hugging Face, which involved creating and training robotic arms to perform precise tasks. The parts used are available via the link: SO-ARM100 GitHub Repository , and the assembly process we followed is accessible at: LeRobot SO-101 Documentation .

The new system works as follows: When waste arrives in front of the robotic arm, the conveyor stops. In collaboration with the mechanical team, we decided to integrate an ultrasonic sensor to manage this functionality. We designed and 3D-printed a custom support for the ultrasonic sensor using an open-source file.

![Mounting Result](/images/mechanic_images/week4/ultra_song.jpg)


## <span style="color: #08C5D1;">Conclusion: A Journey of Innovation</span>
This project was a testament to our ability to combine creativity, technical skills, and teamwork. Every step of the way— from ideation to virtual design, physical construction, and troubleshooting— taught us valuable lessons. As we prepare for the Tekbot Robotics Challenges, we are proud of what we have achieved and excited to showcase our conveyor system.