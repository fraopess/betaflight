/*
 * Integration of optical flow data into position estimation
 *
 * This module integrates optical flow data from any optical flow sensor
 * into the position hold (POSHOLD) and altitude hold (ALTHOLD) flight modes.
 *
 * The optical flow data is fused with gyroscope and altitude data.
 *
 * OPTICAL FLOW AXIS CONVENTION (MAVLink OPTICAL_FLOW_RAD standard):
 *   - flowX: rotation around X (roll) + translation along Y (sideways)
 *   - flowY: rotation around Y (pitch) + translation along X (forward)
 *
 * After gyroscopic compensation, flow values map to body velocities:
 *   - velBodyX (forward/back) ← flowY
 *   - velBodyY (left/right) ← -flowX (inverted per MAVLink spec)
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef USE_OPTICALFLOW

// Position estimation structure (body frame)
// X: forward, Y: left (ESP32CAM convention)
typedef struct {
    float positionX;        // Estimated X position in body frame (m) - forward
    float positionY;        // Estimated Y position in body frame (m) - left
    float velocityX;        // Estimated X velocity in body frame (m/s) - forward
    float velocityY;        // Estimated Y velocity in body frame (m/s) - left
    float altitude;         // Estimated altitude (m)
    uint32_t lastUpdate;    // Last update timestamp (ms)
    bool valid;             // Position estimate valid
} positionEstimate_t;

// Optical flow integration functions
void opticalFlowInit(void);
void opticalFlowUpdate(void);
bool opticalFlowGetPosition(positionEstimate_t *estimate);

// Position control functions
void opticalFlowResetPosition(void);
void opticalFlowSetPositionTarget(float targetX, float targetY);
void opticalFlowActivatePositionHold(bool activate);
bool opticalFlowIsPositionValid(void);

// Flow fusion with IMU
void fuseOpticalFlowWithIMU(float flowX, float flowY, float altitude, float dt);

// Position controller interface - returns autopilot angles
float getPositionHoldPitchAngle(void);   // Returns pitch angle in decidegrees
float getPositionHoldRollAngle(void);    // Returns roll angle in decidegrees
bool isPositionHoldActive(void);

#endif // USE_OPTICALFLOW
