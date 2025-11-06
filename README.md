# Ball-Balancing-Robot
An educational Ball-Balancing Robot. This project introduces some of the core concepts of robotics: programming, inverse kinematics, computer vision, and PID control.

Discord: https://discord.com/invite/WJuUWsy6DJ

# Video Overview
<div align="center">
  <a href="https://www.youtube.com/watch?v=l92hJUUjWb0&t=6s">
    <img src="https://img.youtube.com/vi/l92hJUUjWb0/0.jpg" alt="Watch the video" width="640">
  </a>
</div>

# Pictures

<div align="center">
  <img src="https://github.com/user-attachments/assets/a4252720-81c4-4c18-85fc-d38fa5c8ed7a" alt="2" width="700" />
</div>
<div align="center">
  <img src="https://github.com/user-attachments/assets/8eb9a714-8835-46d8-bd33-cb98a85422c6" alt="1" width="700" />
</div>


# Build Instruction

The 3D models and print profile for a Bambu A1 printer can be found here: https://makerworld.com/en/models/1197770-ball-balancing-robot#profileId-1210633

The materials needed are:
| Part Name                     | Quantity |
|-------------------------------|----------|
| M2 x 8 Cap Head Socket Screw  | 6        |
| M2.5 x 10 Cap Head Socket Screw  | 4        |
| M3 x 5 Socket Head Screw      | 18       |
| M3 x 10 Socket Head Screw     | 3        |
| M3 x 15 Socket Head Screw     | 1        |
| M3 x 20 Socket Head Screw     | 3        |
| M4 x 20 Socket Head Screw     | 6        |
| M4 x 30 Socket Head Screw     | 6        |
| M5 x 30 Socket Head Screw     | 3        |
| M3 Hex Nut                   | 3        |
| M4 Nylock Nut                | 6        |
| M4 Hex Nut                   | 12       |
| M3 x 10 Standoff             | 9        |
| M3 x 15 Standoff             | 6        |
| M3 x 20 Standoff             | 3        |
| Standoff                     | 1        |
| 4-10 Bearing                 | 6        |
| Rubber Foot 12x9x9           | 3        |
| M5 Washer                    | 6        |

---

### Motor Angle Calibration
Each of the three motors must be calibrated. Follow these exact steps:

1. **Set all motor offsets to `0`.**  
   In `controller.py`, inside the `set_motor_angles` function under class `RobotController`, make sure it looks like this:  
   ```python
   self.s1.angle = clamp(theta1)
   self.s2.angle = clamp(theta2)
   self.s3.angle = clamp(theta3)
    ```
2. **Initialize the robot.**  
   Power on the robot. Then, in your terminal (in correct directory), run:  
   `python controller.py`  
   This sets the initial motor positions. Once the robot has reached its position, **do not touch it**. Power it off, then use a level or measurement tool to check how flat the top plate is.

3. **Tune each motor's angle offset.**  
   Based on how the plate is tilted, adjust each motor's angle by adding or subtracting an offset. Modify the code like this:
   ```python
   self.s1.angle = clamp(theta1) + OFFSET_S1
   self.s2.angle = clamp(theta2) + OFFSET_S2
   self.s3.angle = clamp(theta3) + OFFSET_S3
   ```
   Replace each `OFFSET_Sn` with the value needed to make the plate level. These values are specific to your hardware and may differ for each motor.

4. **Iterate until balanced.**  
   Repeat steps 2 and 3. Re-run the controller, check the plate, and adjust the offsets as needed. Continue this process until the top plate is completely flat and stable.

---

