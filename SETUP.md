# PICO VR Teleoperation for Lebai Robot — Setup Guide

## Overview

This system enables teleoperation of a Lebai robot arm using a PICO VR headset. The VR controller acts as the robot's end effector — move the controller and the robot tip follows.

```
PICO VR Headset ──USB──> PC (XRoboToolkit PC Service) ──WiFi──> Lebai Robot
```

---

## Hardware Requirements

- **PICO 4 Ultra** VR headset with controllers
- **Lebai robot arm** (6 DOF) with WiFi enabled
- **Control PC** (Ubuntu Linux, x86_64) with USB port and WiFi

---

## Software Requirements

### Python packages
```bash
pip install numpy lebai_sdk
```

### Native libraries (included in project directory)
- `xrobotoolkit_sdk.cpython-312-x86_64-linux-gnu.so` — XRoboToolkit Python bindings
- `libPXREARobotSDK.so` — native SDK library (loaded via `LD_LIBRARY_PATH`)

### System tools
- `adb` (Android Debug Bridge) — for USB communication with PICO
- XRoboToolkit PC Service — installed at `/opt/apps/roboticsservice/`

---

## Network Architecture

| Device | Connection | IP Address |
|--------|-----------|------------|
| PICO VR | USB (ADB) to PC | localhost (via ADB reverse) |
| Lebai Robot | WiFi hotspot | `10.20.17.1` |
| Control PC | WiFi to Lebai, USB to PICO | connects to both |

The PC connects to the **Lebai WiFi hotspot** (the robot creates its own WiFi network). The PICO connects to the PC via **USB cable** using ADB reverse port forwarding.

---

## Step-by-Step Setup

### 1. Start the XRoboToolkit PC Service

The PC Service bridges USB communication between the PICO headset and the Python SDK.

```bash
bash /opt/apps/roboticsservice/runService.sh
```

Verify it's running:
```bash
ps aux | grep RoboticsService
# Should show: ./RoboticsServiceProcess
```

Check listening ports:
```bash
ss -tlnp | grep Robotics
# Should show ports 60061 and 63901
```

### 2. Connect PICO VR headset via USB

1. Plug the PICO into the PC with a **data-capable USB cable**
2. Launch the **XRoboToolkit Client** app on the PICO

Verify ADB sees the device:
```bash
adb devices
# Should show something like: PA94Y0MGK8150360G  device
```

### 3. Set up ADB reverse port forwarding

This allows the PICO to reach the PC Service through the USB connection:

```bash
adb reverse --remove-all
adb reverse tcp:60061 tcp:60061
adb reverse tcp:63901 tcp:63901
```

Verify:
```bash
adb reverse --list
# Should show:
# UsbFfs tcp:60061 tcp:60061
# UsbFfs tcp:63901 tcp:63901
```

**Note:** This is done automatically when you click "Connect XRT" in the GUI, but you may need to run it manually if the PICO was restarted.

### 4. Connect to the Lebai Robot WiFi

1. Power on the Lebai robot
2. On the PC, connect to the Lebai's WiFi network (look for a network like `lebai-XXXX`)
3. The robot will be reachable at `10.20.17.1`

### 5. Launch the Application

```bash
cd "/home/cvlab/Desktop/LebAI Teleop"
python3 main.py
```

Or

```bash
cd "/home/cvlab/Desktop/LebAI Teleop"
./run.sh
```

Or with custom parameters:
```bash
./run.sh --ip 10.20.17.1 --scale 0.5 --hz 50 --max-speed 0.5
```

---

## Using the GUI

### Connection (do these in order)

1. **Click "Connect XRT SDK"** — connects to PICO via PC Service
   - Status turns green: "Connected — PICO streaming"
   - If orange ("waiting for PICO data"), pick up and move the controller
2. **Click "Connect Lebai"** — connects to the robot
   - Status turns green: "Connected"
3. **Click "Start Teleop"** — begins the control loop

### Controller Mapping

| Controller | Button | Action |
|-----------|--------|--------|
| **Right** | Trigger (top, index finger) | **Hold** to move robot arm |
| **Left** | Trigger (top, index finger) | Close gripper (incremental) |
| **Left** | Grip (side, squeeze) | Open gripper (incremental) |
| **Right** | Y button | Emergency stop |
| **Right** | A button | Toggle suction on/off |

### How Arm Control Works

1. **Press and hold** the right trigger (past 70% threshold)
2. The system syncs the controller position to the robot's current TCP
3. **Move the controller** — the robot tip follows your hand 1:1
4. **Release the trigger** — the robot stops immediately
5. **Press again** — re-syncs from the new positions

The controller acts as the robot's end effector. Move it left, the robot goes left. Move it up, the robot goes up. Rotate your wrist, the robot rotates.

### Recalibration

If the directions feel wrong (e.g., moving left makes the robot go forward):

1. Click **"Start Calibration"**
2. **Step 1:** Hold the controller still at a reference point, click "Capture Origin"
3. **Step 2:** Move the controller ~20cm in the robot's forward (+X) direction, click "Capture +X"
4. **Step 3:** Move the controller ~20cm upward (+Z), click "Capture +Z"

This computes the rotation matrix that maps VR space to robot space.

### Adjustable Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| Scale | 0.5 | Movement ratio (0.5 = robot moves half as far as controller) |
| Max Speed | 0.5 m/s | Maximum linear speed cap |
| Max Angular Speed | 1.0 rad/s | Maximum rotation speed cap |
| Control Hz | 50 | Control loop frequency |
| Acceleration | 0.8 m/s^2 | Robot acceleration |

### Emergency Stop

- Press **Y button** on the right controller, OR
- Click the **"EMERGENCY STOP"** button in the GUI
- To recover: click **"Reset E-Stop"**, then "Start Teleop" again

---

## Troubleshooting

### PICO not connecting

1. Check USB cable is data-capable (not charge-only)
2. Run `adb devices` — should show the device
3. Ensure XRoboToolkit PC Service is running: `ps aux | grep RoboticsService`
4. Re-run ADB reverse ports:
   ```bash
   adb reverse --remove-all
   adb reverse tcp:60061 tcp:60061
   adb reverse tcp:63901 tcp:63901
   ```
5. Restart the XRoboToolkit app on the PICO headset

### "SDK ready — waiting for PICO data..." (orange status)

The SDK connected but no VR data is coming through. Pick up and move the right controller. If it persists, the XRoboToolkit app on the PICO may not be running — launch it from the PICO's app library.

### Lebai connection refused

- Ensure the robot is powered on
- Ensure PC is connected to the Lebai's WiFi network
- Try: `ping 10.20.17.1`
- The robot's controller software may need time to boot after power-on

### Robot moves are jerky

- The system uses `move_pvt` (position-velocity-time streaming) with `towardj` fallback
- Reduce scale factor for smaller, smoother movements
- Reduce max speed if the robot is overshooting

### Robot doesn't respond to controller

- Check that teleop state shows "running" (green)
- Check that the right trigger value is changing in the VR data display
- Ensure `_is_controlling` shows as active when trigger is held

---

## File Structure

```
LebAI Teleop/
  main.py                    — Entry point, CLI args, Tkinter window setup
  pico_teleop_controller.py  — Core controller: VR input, IK, motion commands
  pico_teleop_widget.py      — Tkinter GUI with live data display
  frame_utils.py             — Math: quaternions, rotations, coordinate transforms
  run.sh                     — Launch script (sets LD_LIBRARY_PATH)
  xrobotoolkit_sdk.*.so      — Native XRoboToolkit Python bindings
  SETUP.md                   — This file
```
