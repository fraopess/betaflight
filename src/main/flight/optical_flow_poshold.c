/*
 * Implementation of optical flow position estimation and control
 * 
 * This integrates ESP32Cam-TFMini optical flow with IMU data for
 * position hold and altitude hold flight modes.
 */

#include "platform.h"

#ifdef USE_RANGEFINDER_ESP32CAM_TFMINI

#include "optical_flow_poshold.h"
#include "drivers/rangefinder/rangefinder_esp32cam_tfmini.h"
#include "drivers/rangefinder/rangefinder.h"
#include "sensors/rangefinder.h"
#include "sensors/gyro.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "common/maths.h"
#include "common/filter.h"
#include "build/debug.h"
#include "drivers/time.h"

// Position estimation state
static positionEstimate_t posEstimate = {
    .positionX = 0.0f,
    .positionY = 0.0f,
    .velocityX = 0.0f,
    .velocityY = 0.0f,
    .altitude = 0.0f,
    .lastUpdate = 0,
    .valid = false
};

// Velocity filters (low-pass for smoothing)
static pt1Filter_t velocityXFilter;
static pt1Filter_t velocityYFilter;

// Position hold setpoints
static float posHoldSetpointX = 0.0f;
static float posHoldSetpointY = 0.0f;
static bool posHoldActive = false;

// Configuration
#define VELOCITY_FILTER_CUTOFF_HZ   5.0f
#define POSITION_UPDATE_RATE_HZ     50.0f
#define POSITION_UPDATE_DT          (1.0f / POSITION_UPDATE_RATE_HZ)

// Initialize optical flow position estimation
void opticalFlowInit(void)
{
    // Initialize velocity filters
    pt1FilterInit(&velocityXFilter, pt1FilterGain(VELOCITY_FILTER_CUTOFF_HZ, POSITION_UPDATE_DT));
    pt1FilterInit(&velocityYFilter, pt1FilterGain(VELOCITY_FILTER_CUTOFF_HZ, POSITION_UPDATE_DT));
    
    // Reset position estimate
    posEstimate.positionX = 0.0f;
    posEstimate.positionY = 0.0f;
    posEstimate.velocityX = 0.0f;
    posEstimate.velocityY = 0.0f;
    posEstimate.altitude = 0.0f;
    posEstimate.valid = false;
}

// Update position estimation with optical flow and IMU data
void opticalFlowUpdate(void)
{
    // Get optical flow data
    if (!esp32camTfminiIsFlowValid()) {
        posEstimate.valid = false;
        return;
    }
    
    // Get flow rates (rad/s)
    float flowRateX = esp32camTfminiGetFlowRateX();
    float flowRateY = esp32camTfminiGetFlowRateY();
    
    // Get altitude from rangefinder
    float altitude = rangefinderGetLatestAltitude() / 100.0f;  // Convert cm to m
    
    // Check altitude validity
    if (altitude < 0.1f || altitude > 12.0f) {
        posEstimate.valid = false;
        return;
    }
    
    // Convert flow rates to velocities (m/s)
    // velocity = flowRate (rad/s) * altitude (m)
    float velX = flowRateX * altitude;
    float velY = flowRateY * altitude;
    
    // Apply rotation based on IMU orientation
    // Get current attitude
    float roll = attitude.values.roll / 10.0f * RAD;   // Convert decidegrees to radians
    float pitch = attitude.values.pitch / 10.0f * RAD;
    float yaw = attitude.values.yaw / 10.0f * RAD;
    
    // Rotate velocities to body frame
    // This compensates for aircraft tilt
    float cosYaw = cos_approx(yaw);
    float sinYaw = sin_approx(yaw);
    
    float velBodyX = velX * cosYaw - velY * sinYaw;
    float velBodyY = velX * sinYaw + velY * cosYaw;
    
    // Apply tilt compensation
    float cosPitch = cos_approx(pitch);
    float cosRoll = cos_approx(roll);
    
    if (cosPitch > 0.1f && cosRoll > 0.1f) {
        velBodyX /= cosPitch;
        velBodyY /= cosRoll;
    }
    
    // Filter velocities
    posEstimate.velocityX = pt1FilterApply(&velocityXFilter, velBodyX);
    posEstimate.velocityY = pt1FilterApply(&velocityYFilter, velBodyY);
    
    // Integrate velocities to get position (simple integration)
    posEstimate.positionX += posEstimate.velocityX * POSITION_UPDATE_DT;
    posEstimate.positionY += posEstimate.velocityY * POSITION_UPDATE_DT;
    
    // Store altitude
    posEstimate.altitude = altitude;
    posEstimate.lastUpdate = millis();
    posEstimate.valid = true;
    
    // Update DEBUG values
    DEBUG_SET(DEBUG_OPTICAL_FLOW, 0, (int)(posEstimate.velocityX * 100));  // cm/s
    DEBUG_SET(DEBUG_OPTICAL_FLOW, 1, (int)(posEstimate.velocityY * 100));
    DEBUG_SET(DEBUG_OPTICAL_FLOW, 2, (int)(posEstimate.positionX * 100));  // cm
    DEBUG_SET(DEBUG_OPTICAL_FLOW, 3, (int)(posEstimate.positionY * 100));
}

// Get current position estimate
bool opticalFlowGetPosition(positionEstimate_t *estimate)
{
    if (!posEstimate.valid) {
        return false;
    }
    
    *estimate = posEstimate;
    return true;
}

// Fuse optical flow with IMU (called from flight controller)
void fuseOpticalFlowWithIMU(float flowX, float flowY, float altitude, float dt)
{
    // This function can be expanded to implement more sophisticated
    // sensor fusion algorithms like Extended Kalman Filter (EKF)
    // For now, we use the simpler integration in opticalFlowUpdate()
    
    // Future: Implement EKF fusion with:
    // - Optical flow velocity
    // - IMU acceleration
    // - Barometer altitude
    // - GPS position (if available)
}

// Position hold controller - returns desired velocity in X direction
float getPositionHoldVelocityX(void)
{
    if (!posEstimate.valid || !posHoldActive) {
        return 0.0f;
    }
    
    // Simple P controller for position hold
    const float kP = 0.5f;  // Position gain (adjust via PID tuning)
    const float maxVelocity = 2.0f;  // Maximum velocity (m/s)
    
    float positionError = posHoldSetpointX - posEstimate.positionX;
    float desiredVelocity = positionError * kP;
    
    // Limit velocity
    desiredVelocity = constrainf(desiredVelocity, -maxVelocity, maxVelocity);
    
    return desiredVelocity;
}

// Position hold controller - returns desired velocity in Y direction
float getPositionHoldVelocityY(void)
{
    if (!posEstimate.valid || !posHoldActive) {
        return 0.0f;
    }
    
    // Simple P controller for position hold
    const float kP = 0.5f;  // Position gain (adjust via PID tuning)
    const float maxVelocity = 2.0f;  // Maximum velocity (m/s)
    
    float positionError = posHoldSetpointY - posEstimate.positionY;
    float desiredVelocity = positionError * kP;
    
    // Limit velocity
    desiredVelocity = constrainf(desiredVelocity, -maxVelocity, maxVelocity);
    
    return desiredVelocity;
}

#endif // USE_RANGEFINDER_ESP32CAM_TFMINI
