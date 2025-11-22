# Betaflight - GPS-Free Indoor Flight Fork

This fork of Betaflight adds support for GPS-free indoor flight capabilities using optical flow and lidar-based position and altitude control.

## Overview

This fork extends Betaflight with integrated support for hybrid optical flow + rangefinder sensors to enable GPS-free flight modes including:

- **Altitude Hold (ALTHOLD)** - using lidar rangefinder for precise altitude measurement
- **Position Hold (POSHOLD)** - using optical flow for horizontal position stabilization

These capabilities make it possible to fly indoors or in GPS-denied environments with stable altitude and position hold.

## Key Features

### Supported Sensors

#### ESP32-S3 CAM + TFMini (Hybrid System)
- **ESP32-S3 CAM** for optical flow
- **TFMini/TFMini Plus** for rangefinding
- Separate UART connections for each component
- Configurable sensor alignment (0°, 90°, 180°, 270° with flip options)
- MAVLink OPTICAL_FLOW_RAD protocol compatibility
- Adjustable flow and rangefinder scaling factors
- Configurable altitude range for valid optical flow data

#### MTF-02 Hybrid Sensor (Integrated Solution)
- **All-in-one** integrated rangefinder + optical flow sensor
- Single UART connection (115200 baud)
- Built-in optical flow processor
- Distance measurement: 10-1200 cm
- Flow data includes quality indicator
- Configurable flow scaling and axis inversion
- Lower power consumption than separate sensors
- Compact form factor ideal for small drones

### Common Features
- Optical flow sensor for horizontal velocity and position estimation
- Gyroscopic compensation for accurate motion tracking
- Low-pass filtering for smooth velocity estimates
- High-precision distance measurement for altitude hold
- Centimeter-level accuracy for indoor flight
- Automatic sensor detection and initialization
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

**Option 1: ESP32-S3 CAM + TFMini**
1. **Flight Controller** - STM32 F4, F7, or H7 based Betaflight-compatible FC
2. **ESP32-S3 CAM Module** - with optical flow sensor capability
3. **TFMini Lidar** - or TFMini Plus rangefinder
4. **UART Connection** - between FC and ESP32-S3 CAM (one UART port required)

**Option 2: MTF-02 Hybrid Sensor **
1. **Flight Controller** - STM32 F4, F7, or H7 based Betaflight-compatible FC
2. **MTF-02 Sensor** - integrated rangefinder + optical flow module
3. **UART Connection** - between FC and MTF-02 (one UART port required)

### Wiring

**ESP32-S3 CAM + TFMini:**
- Connect ESP32-S3 CAM to flight controller UART (TX, RX, GND, 5V)
- Connect TFMini lidar to flight controller UART or pass-through ESP32
- Ensure proper power supply (5V) for both modules
- Mount sensors facing downward for ground distance measurement

**MTF-02 Hybrid Sensor:**
- Connect MTF-02 TX to FC RX pin
- Connect MTF-02 RX to FC TX pin
- Connect GND to GND
- Connect VCC to 5V
- Mount sensor facing downward for ground distance measurement
- Only one UART port needed (simpler wiring than ESP32 option)

## Configuration

### CLI Commands

#### Option 1: ESP32-S3 CAM + TFMini Configuration

```bash
# Set rangefinder hardware
set rangefinder_hardware = ESP32

# Set optical flow hardware
set opticalflow_hardware = ESP32_HYBRID

# Set sensor alignment (0-7)
set esp32_hybrid_alignment = 0

# Optical flow scale factor (0-200%, default 100)
set esp32_hybrid_flow_scale = 100

# Rangefinder scale factor (0-200%, default 100)
set esp32_hybrid_range_scale = 100

# Minimum altitude for valid optical flow (cm)
set esp32_hybrid_min_alt_cm = 10

# Maximum altitude for valid optical flow (cm, 0 = use sensor max)
set esp32_hybrid_max_alt_cm = 120

# Invert flow axes if needed
set esp32_hybrid_flow_invert_x = OFF
set esp32_hybrid_flow_invert_y = OFF

# Save configuration
save
```

#### Option 2: MTF-02 Hybrid Sensor Configuration

```bash
# Set rangefinder hardware to MTF-02
set rangefinder_hardware = MTF02_HYBRID

# Set optical flow hardware to MTF-02
set opticalflow_hardware = MTF02_HYBRID

# Minimum valid range (cm, default 10)
set mtf02_min_range_cm = 10

# Maximum valid range (cm, default 1200)
set mtf02_max_range_cm = 1200

# Minimum flow quality threshold (0-255, default 20)
set mtf02_min_quality = 20

# Flow scale factor (0-200%, default 100)
set mtf02_flow_scale = 100

# Invert flow axes if needed
set mtf02_flow_invert_x = OFF
set mtf02_flow_invert_y = OFF

# Save configuration
save
```

#### Serial Port Configuration

For ESP32 hybrid sensor, configure the serial port:
```bash
# Example for UART3 (serial port 2)
serial 2 115200 524288    # 524288 = FUNCTION_ESP32CAM_TFMINI
```

For MTF-02 hybrid sensor, configure the serial port:
```bash
# Example for UART3 (serial port 2)
serial 2 115200 1048576   # 1048576 = FUNCTION_HYBRID_MTF02
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

### General Tips

1. **Surface Requirements** - Optical flow works best on textured surfaces. Avoid plain, featureless floors.

2. **Lighting** - Ensure adequate lighting for optical flow sensor. Avoid direct sunlight or complete darkness.

3. **Altitude Range** - Position hold works best between 0.5m and 3m altitude. Configure min/max altitude accordingly.

4. **Tuning** - Start with default PID values and adjust based on your drone's response:
   - Increase P gain if position drift is too slow to correct
   - Increase I gain if there's steady-state position error
   - Increase D gain if there are oscillations

5. **Testing** - Always test in a safe, open indoor space first. Start with altitude hold before trying position hold.

6. **Stick Control** - Small stick movements will deactivate position hold. Return sticks to center to re-engage.

### MTF-02 Specific Tips

1. **Quality Threshold** - If experiencing drift, increase `mtf02_min_quality` (default 20). Higher values require better surface texture.

2. **Flow Scaling** - If position hold is too aggressive or too weak, adjust `mtf02_flow_scale`:
   - Decrease (< 100) if corrections are too strong
   - Increase (> 100) if corrections are too weak

3. **Axis Inversion** - If drone moves in wrong direction during position hold:
   - Set `mtf02_flow_invert_x = ON` if left/right movement is inverted
   - Set `mtf02_flow_invert_y = ON` if forward/back movement is inverted

4. **Range Limits** - Adjust `mtf02_min_range_cm` and `mtf02_max_range_cm` based on your flying environment:
   - Increase min range if getting false readings near ground
   - Decrease max range if experiencing noise at higher altitudes

5. **Sensor Mounting** - Ensure MTF-02 is mounted:
   - Facing straight down (perpendicular to ground)
   - Away from propeller wash and vibration sources
   - With clear view of ground (no obstructions)

## Technical Details

### MTF-02 Protocol Specification

The MTF-02 uses a binary serial protocol at 115200 baud (8N1):

**Frame Structure (16 bytes total):**
```
[0x59][0x59][DIST_LSB][DIST_MSB][STR_LSB][STR_MSB]
[FLOW_X_LSB][FLOW_X_MSB][FLOW_Y_LSB][FLOW_Y_MSB][QUALITY]
[RESERVED1][RESERVED2][RESERVED3][CHECKSUM]
```

**Field Descriptions:**
- **Sync Bytes**: `0x59 0x59` (header)
- **Distance**: 16-bit little-endian, distance in centimeters
- **Strength**: 16-bit little-endian, signal strength (> 100 for valid)
- **Flow X/Y**: 16-bit signed little-endian, velocity in cm/s at 1m height
- **Quality**: 8-bit flow quality indicator (0-255)
- **Reserved**: 3 bytes for future use
- **Checksum**: Sum of sync bytes + all payload bytes

**Flow Velocity Conversion:**
```
actual_velocity (m/s) = (flow_velocity_cm/s * actual_height_m) / 100
flow_rate (rad/s) = actual_velocity / actual_height_m
                  = flow_velocity / 100  // Simplified
```

### Coordinate System
- **Body Frame**: X (forward), Y (left), Z (up)
- **Optical Flow**: MAVLink OPTICAL_FLOW_RAD convention
  - flowX: rotation around X (roll) + translation along Y (sideways)
  - flowY: rotation around Y (pitch) + translation along X (forward)
- After gyroscopic compensation:
  - flowX → left/right body motion (inverted per MAVLink spec)
  - flowY → forward/back body motion

### Update Rates
- MTF-02 sensor: 100 Hz (10ms update period)
- ESP32 sensor: 20-50 Hz (adjustable)
- Position estimation: 50 Hz
- Velocity filtering: 5 Hz low-pass filter cutoff
- PID controller: runs at main loop rate

### Implementation Files

Key implementation files for reference:

**ESP32 Hybrid Sensor:**
- [esp32.c](src/main/drivers/hybrid/esp32/esp32.c) - ESP32 hybrid sensor driver
- [esp32.h](src/main/drivers/hybrid/esp32/esp32.h) - ESP32 protocol definitions
- [esp32_config.c](src/main/drivers/hybrid/esp32/esp32_config.c) - ESP32 configuration

**MTF-02 Hybrid Sensor:**
- [mtf02.c](src/main/drivers/hybrid/mtf02/mtf02.c) - MTF-02 hybrid sensor driver
- [mtf02.h](src/main/drivers/hybrid/mtf02/mtf02.h) - MTF-02 protocol definitions
- [mtf02_config.c](src/main/drivers/hybrid/mtf02/mtf02_config.c) - MTF-02 configuration
- [hybrid_mtf02.h](src/main/pg/hybrid_mtf02.h) - MTF-02 parameter group

**Common:**
- [optical_flow_poshold.c](src/main/flight/optical_flow_poshold.c) - Position hold controller
- [alt_hold_rangefinder.c](src/main/flight/alt_hold_rangefinder.c) - Altitude hold controller
- [opticalflow.c](src/main/sensors/opticalflow.c) - Optical flow sensor abstraction
- [rangefinder.c](src/main/sensors/rangefinder.c) - Rangefinder sensor abstraction

## Future Enhancements

This fork is under active development. Planned features include:

- Obstacle avoidance using multiple rangefinders
- Integration with additional sensors (ultrasonic, stereo cameras)

## Compilation

### Option 1: Build with ESP32 Hybrid Sensor Support

**Using Makefile variable:**
```bash
make TARGET=<your_target> USE_HYBRID_ESP32=yes
```

**Or using EXTRA_FLAGS:**
```bash
make TARGET=<your_target> EXTRA_FLAGS="-DUSE_HYBRID_ESP32"
```

**Or enable in `target.h`:**
```c
#define USE_HYBRID_ESP32
```

### Option 2: Build with MTF-02 Hybrid Sensor Support

**Using Makefile variable:**
```bash
make TARGET=<your_target> USE_HYBRID_MTF02=yes
```

**Or using EXTRA_FLAGS:**
```bash
make TARGET=<your_target> EXTRA_FLAGS="-DUSE_HYBRID_MTF02"
```

**Or enable in `target.h`:**
```c
#define USE_HYBRID_MTF02
```




## Support & Contributions

This is an experimental fork. Use at your own risk. Always follow safe flying practices and local regulations.

Contributions, bug reports, and feature requests are welcome via GitHub issues and pull requests.

## License

This project inherits the Betaflight license (GNU General Public License v3.0). See [LICENSE](LICENSE) for details.

## Credits

Based on [Betaflight](https://github.com/betaflight/betaflight) - the leading flight controller firmware for multi-rotor and fixed-wing aircraft.

Original Betaflight documentation: [README_betaflight.md](README_betaflight.md)
