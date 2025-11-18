# Betaflight - ESP32-S3 CAM + Lidar Fork

This fork of Betaflight adds support for GPS-free indoor flight capabilities using optical flow and lidar-based position and altitude control.

## Overview

This fork extends Betaflight with integrated support for the **ESP32-S3 CAM** module combined with **TFMini lidar rangefinder** to enable GPS-free flight modes including:

- **Altitude Hold (ALTHOLD)** - using lidar rangefinder for precise altitude measurement
- **Position Hold (POSHOLD)** - using optical flow for horizontal position stabilization

These capabilities make it possible to fly indoors or in GPS-denied environments with stable altitude and position hold.

## Key Features

### ESP32-S3 CAM Integration
- Optical flow sensor for horizontal velocity and position estimation
- Gyroscopic compensation for accurate motion tracking
- Configurable sensor alignment (0°, 90°, 180°, 270° with flip options)
- MAVLink OPTICAL_FLOW_RAD protocol compatibility
- Adjustable flow and rangefinder scaling factors
- Configurable altitude range for valid optical flow data
- Low-pass filtering for smooth velocity estimates

### TFMini Lidar Support
- High-precision distance measurement for altitude hold
- Centimeter-level accuracy for indoor flight
- Automatic sensor detection and initialization
- Compatible with TFMini and TFMini Plus models
- Serial communication interface (UART)

### Position Hold Flight Mode
- Body-frame position estimation (X: forward, Y: left)
- PID-based position controller with configurable gains
- Automatic position hold activation via flight mode switch
- Stick deadband with position hold deactivation on manual input
- Real-time position and velocity estimation
- Maximum autopilot angle limiting for safety
- Smooth velocity filtering for stable control

### Altitude Hold Flight Mode
- Lidar-based altitude measurement
- Superior performance compared to barometer indoors
- Fast response time for precise altitude control
- Works in conjunction with optical flow for full 3D position hold

## Hardware Requirements

### Required Components
1. **Flight Controller** - STM32 F4, F7, or H7 based Betaflight-compatible FC
2. **ESP32-S3 CAM Module** - with optical flow sensor capability
3. **TFMini Lidar** - or TFMini Plus rangefinder
4. **UART Connection** - between FC and ESP32-S3 CAM/TFMini

### Wiring
- Connect ESP32-S3 CAM to flight controller UART (TX, RX, GND, 5V)
- Connect TFMini lidar to flight controller UART or pass-through ESP32
- Ensure proper power supply (5V) for both modules
- Mount sensors facing downward for ground distance measurement

## Configuration

### CLI Commands

Configure the ESP32-S3 CAM and TFMini sensor:

```
# Enable the sensor
set esp32cam_tfmini_hardware = ESP32CAM

# Set sensor alignment (0-7)
set esp32cam_tfmini_alignment = 0

# Optical flow scale factor (0-200%, default 100)
set esp32cam_tfmini_optical_flow_scale = 100

# Rangefinder scale factor (0-200%, default 100)
set esp32cam_tfmini_rangefinder_scale = 100

# Minimum altitude for valid optical flow (cm)
set esp32cam_tfmini_min_altitude_cm = 10

# Maximum altitude for valid optical flow (cm, 0 = use sensor max)
set esp32cam_tfmini_max_altitude_cm = 120

# Invert flow axes if needed
set esp32cam_tfmini_flow_invert_x = OFF
set esp32cam_tfmini_flow_invert_y = OFF
```

### Position Hold PID Configuration

```
# Maximum autopilot angle (degrees)
set optical_flow_poshold_max_angle = 10

# PID gains (stored as * 100)
set optical_flow_poshold_pid_p = 50    # P = 0.5
set optical_flow_poshold_pid_i = 10    # I = 0.1
set optical_flow_poshold_pid_d = 5     # D = 0.05

# Maximum I term (stored as * 10)
set optical_flow_poshold_pid_i_max = 100  # I_MAX = 10.0

# Stick deadband (default 12)
set optical_flow_poshold_stick_deadband = 12

# Save configuration
save
```

### Rangefinder Configuration

Configure the rangefinder hardware:

```
# Enable TFMini lidar
set rangefinder_hardware = TFMINI

# Save configuration
save
```

## Flight Modes

### Altitude Hold (ALTHOLD)
- Activate via flight mode switch in Betaflight Configurator
- Uses lidar for altitude measurement instead of barometer
- Maintains stable altitude based on lidar readings
- Works from 10cm to ~12m depending on sensor configuration

### Position Hold (POSHOLD)
- Activate via flight mode switch in Betaflight Configurator
- Uses optical flow + lidar for 3D position estimation
- Maintains horizontal position (forward/back, left/right)
- Automatically deactivates when stick input exceeds deadband
- Re-activates at current position when sticks return to center
- Best performance on textured surfaces with good lighting

## Usage Tips

1. **Surface Requirements** - Optical flow works best on textured surfaces. Avoid plain, featureless floors.

2. **Lighting** - Ensure adequate lighting for optical flow sensor. Avoid direct sunlight or complete darkness.

3. **Altitude Range** - Position hold works best between 0.5m and 3m altitude. Configure min/max altitude accordingly.

4. **Tuning** - Start with default PID values and adjust based on your drone's response:
   - Increase P gain if position drift is too slow to correct
   - Increase I gain if there's steady-state position error
   - Increase D gain if there are oscillations

5. **Testing** - Always test in a safe, open indoor space first. Start with altitude hold before trying position hold.

6. **Stick Control** - Small stick movements will deactivate position hold. Return sticks to center to re-engage.

## Technical Details

### Coordinate System
- **Body Frame**: X (forward), Y (left), Z (up)
- **Optical Flow**: MAVLink OPTICAL_FLOW_RAD convention
  - flowX: rotation around X (roll) + translation along Y (sideways)
  - flowY: rotation around Y (pitch) + translation along X (forward)
- After gyroscopic compensation:
  - flowX → left/right body motion (inverted per MAVLink spec)
  - flowY → forward/back body motion

### Update Rates
- Position estimation: 50 Hz
- Velocity filtering: 5 Hz low-pass filter cutoff
- PID controller: runs at main loop rate

### Implementation Files
Key implementation files for reference:
- [esp32cam_tfmini_config.h](src/main/pg/esp32cam_tfmini_config.h) - Sensor configuration
- [optical_flow_poshold.c](src/main/flight/optical_flow_poshold.c) - Position hold controller
- [opticalflow.c](src/main/sensors/opticalflow.c) - Optical flow sensor driver
- [rangefinder_lidartf.c](src/main/drivers/rangefinder/rangefinder_lidartf.c) - TFMini lidar driver

## Future Enhancements

This fork is under active development. Planned features include:

- Support for additional optical flow sensors (PMW3901, etc.)
- Support for additional lidar models (VL53L0X, VL53L1X, etc.)
- Advanced position estimation with Kalman filtering
- Way point navigation for autonomous indoor flight
- Obstacle avoidance using multiple rangefinders
- Integration with additional sensors (ultrasonic, stereo cameras)

## Compilation

Build with support for ESP32-CAM and TFMini:

```bash
make TARGET=<your_target> USE_RANGEFINDER_ESP32CAM_TFMINI=yes
```

Or enable in `target.h`:
```c
#define USE_RANGEFINDER_ESP32CAM_TFMINI
```

## Support & Contributions

This is an experimental fork. Use at your own risk. Always follow safe flying practices and local regulations.

Contributions, bug reports, and feature requests are welcome via GitHub issues and pull requests.

## License

This project inherits the Betaflight license (GNU General Public License v3.0). See [LICENSE](LICENSE) for details.

## Credits

Based on [Betaflight](https://github.com/betaflight/betaflight) - the leading flight controller firmware for multi-rotor and fixed-wing aircraft.

Original Betaflight documentation: [README_betaflight.md](README_betaflight.md)
