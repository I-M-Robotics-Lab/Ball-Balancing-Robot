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

# Ballbot – Ball Balancing Robot (Rev 9)

A Raspberry Pi–based ball balancing robot using real-time vision feedback and PID control.

Rev 9 represents a structural milestone:
- Clean `src/` layout package structure
- Separation of HMI and runtime
- Headless-safe runtime operation
- SSH auto-launch capability
- JSON-driven configuration
- Modular subsystem architecture

---

## Getting Started

This section describes the full process from mechanical assembly to first runtime execution.

---

## 1. Mechanical Assembly

### 1.1 Obtain Components

Refer to the Bill of Materials (BOM) for required hardware:

- Raspberry Pi
- Camera module
- PCA9685 I2C servo driver
- Servos
- Power supply
- Ball and platform hardware

---

### 1.2 Print and Assemble Robot Structure

- 3D print structural components.
- Assemble platform and servo linkages.
- Ensure:
  - Platform pivots freely.
  - No binding in linkages.
  - Servo horns are mounted securely.
  - Ball rolls smoothly across platform surface.

Mechanical slop or binding significantly affects control stability.

---

## 2. Raspberry Pi Setup & Electrical Integration

### 2.1 Flash Raspberry Pi OS

- Use Raspberry Pi Imager.
- Install Raspberry Pi OS (Bookworm recommended).
- Enable SSH.
- Configure WiFi if required.
- Set username and password.

Boot the Pi and confirm SSH access.

---

### 2.2 Install Electrical Components

Wire components according to wiring instructions:

- Camera → CSI connector
- PCA9685 → I2C (SDA/SCL)
- Servo power isolated from Pi 5V rail (recommended)
- Common ground between servo supply and Pi

Enable I2C:

```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

Reboot.

Confirm I2C device is detected:

```bash
i2cdetect -y 1
```

You should see the PCA9685 address (typically `0x40`).

---

### 2.3 Install Software

Run installer:

```bash
bash scripts/install.sh
```

This installs:
- Python dependencies
- System libraries
- OpenCV
- libcamera stack

---

## 3. Initial Calibration via HMI

Move Ballbot HMI.desktop to desktop

Launch the HMI:

```bash
hmi
```

The HMI runs over SSH using curses and provides jog controls and calibration tools.

---

### 3.1 Zero Pose Capture (Mechanical Reference)

Before applying offsets, establish a mechanical zero pose.

Procedure:

1. Power the system.
2. Arm the servos.
3. Manually jog the platform until it is visually level.
4. Trigger **Zero Pose Capture** from the HMI.
5. Confirm the pose is recorded.

This step defines the neutral mechanical reference position for the platform. The zero pose is stored and used as the baseline for subsequent offset adjustments.

---

### 3.2 Offset Calibration

After zero pose capture:

1. Fine-adjust X and Y offsets using jog controls.
2. Observe platform level and ball behavior.
3. Save calibration values.
4. Disarm and re-arm to confirm repeatability.

Calibration values are written to:

```
config/calibration.json
```

---

### 3.3 Verification

After calibration:

- Platform should return to level at neutral command.
- No servo drift at idle.
- No bias in ball roll direction.
- Servo sounds should be symmetrical (no constant correction hum).

Accurate zero pose capture is critical for stable PID performance.

---

## 4. Run Runtime (Control Loop)

Start runtime from the HMI or directly:

```bash
bash scripts/run_runtime.sh
```

Runtime performs:
- Frame capture (~60 Hz)
- Vision processing (~50 Hz)
- PID control
- Servo actuation

If running over SSH (headless):
- Video preview is disabled automatically.
- Control loop remains fully functional.

Press `q` to exit runtime safely.

On shutdown:
- Servos disarm.
- Camera terminates cleanly.

---

## First Successful Bring-Up Checklist

Before declaring success:

- Ball remains near center without oscillation.
- No mechanical binding.
- Servos respond smoothly.
- No runaway tilt on startup.
- PID gains stable at low disturbance.

