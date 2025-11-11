/*
 * Integration of ESP32Cam-TFMini optical flow data into position estimation
 * 
 * This file shows how to integrate the optical flow data from ESP32Cam-TFMini
 * into the position hold (POSHOLD) and altitude hold (ALTHOLD) flight modes.
 * 
 * The optical flow data will be fused with gyroscope and altitude data.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef USE_RANGEFINDER_ESP32CAM_TFMINI

// Position estimation structure
typedef struct {
    float positionX;        // Estimated X position (m)
    float positionY;        // Estimated Y position (m)
    float velocityX;        // Estimated X velocity (m/s)
    float velocityY;        // Estimated Y velocity (m/s)
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

// Position controller interface
float getPositionHoldVelocityX(void);
float getPositionHoldVelocityY(void);

#endif // USE_RANGEFINDER_ESP32CAM_TFMINI
