# Assembly and mobile app control

We followed the official installation video carefully to assemble the Jetson Nano expansion board and mount the Dofbot components.

YouTube tutorial we followed:
- Installation video: https://youtu.be/7S8s1Oy_cts?si=ksBXUom3v1EDUcJC

Assembly checklist and photos
- Kit inventory: ../../public/images/Final_Test_IT_Pole/inventory.jpeg
- Heat sink / active radiator mounting: ../../public/images/Final_Test_IT_Pole/Jetson_nano_board_with_active_radiator.jpg
- Mounting pillars and camera support: ../../public/images/Final_Test_IT_Pole/Copper_pillars_and_section_cup.jpeg
- Expansion board installation: ../../public/images/Final_Test_IT_Pole/Expansion_board_installation.jpeg
- Antenna installation: ../../public/images/Final_Test_IT_Pole/Antenna_installation.jpeg
- Camera installation: ../../public/images/Final_Test_IT_Pole/Camera_installation%20.jpeg
- Completed assembly: ../../public/images/Final_Test_IT_Pole/Complete_installation.jpeg

Common issues we encountered and how we fixed them
- Loose screws on the expansion board: we rechecked each spacer and used thread locker on the M2 screws to avoid vibration loosening during tests.
- Radiator orientation: the active cooler must have the fan outlet clear of the camera ribbon and connectors; we rotated the cooler and shortened a cable to avoid interference.
- Camera cable seating: the CSI ribbon must be fully inserted and the locking clip closed — partial seating caused a black image. We re-seated the cable and power-cycled the Jetson.

Mobile app control (first take in hand and calibration)
- We used the vendor mobile app to test basic arm motions and to get familiar with joystick control. Steps we performed:
	1. Power the Dofbot and connect the control board to the same Wi‑Fi network as our phone.
 2. Open the app and connect to the robot; calibrate the gripper open/close limits using the in-app sliders.
 3. Perform an initial homing: move each servo to a known neutral position (we used the servo angles shown in `read_servo_positions.py`).
 4. Save the neutral/home pose in the app for quick recovery.

Notes
- Keep the assembly area clean and magnetic tools aside from PCB components.
- Document serial numbers and take photos at each major step — this helped us debug later when connectors were reversed.
