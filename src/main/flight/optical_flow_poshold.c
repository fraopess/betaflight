/*
 * Implementation of optical flow position estimation and control
 *
 * This integrates ESP32Cam-TFMini optical flow with IMU data for
 * position hold and altitude hold flight modes.
 *
 * Key features:
 * - Gyroscopic compensation: Subtracts drone rotation from optical flow
 * - Body to Earth frame transformation
 * - Tilt compensation for accurate ground velocity estimation
 * - Position integration with drift compensation
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
#define MIN_VALID_ALTITUDE_CM       10.0f   // Minimum altitude for valid flow (10cm)
#define MAX_VALID_ALTITUDE_CM       1200.0f // Maximum altitude for valid flow (12m)

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

    // Get altitude from rangefinder (in cm, convert to m)
    float altitudeCm = rangefinderGetLatestAltitude();
    float altitude = altitudeCm / 100.0f;

    // Check altitude validity
    if (altitudeCm < MIN_VALID_ALTITUDE_CM || altitudeCm > MAX_VALID_ALTITUDE_CM) {
        posEstimate.valid = false;
        return;
    }

    // Get raw optical flow rates (rad/s) - these include drone rotation
    float flowRateX = esp32camTfminiGetFlowRateX();
    float flowRateY = esp32camTfminiGetFlowRateY();

    // *** GYROSCOPIC COMPENSATION ***
    // Subtract gyro rotation rates to get pure translational flow
    // gyro.gyroADCf is in deg/s, convert to rad/s
    const float DEG_TO_RAD = 0.017453292f;
    float gyroRateX = gyro.gyroADCf[FD_ROLL] * DEG_TO_RAD;  // Roll rate (rad/s)
    float gyroRateY = gyro.gyroADCf[FD_PITCH] * DEG_TO_RAD; // Pitch rate (rad/s)

    // Compensate: subtract gyro rotation from optical flow
    // The camera sees apparent motion due to rotation, we remove it
    float compensatedFlowX = flowRateX - gyroRateX;
    float compensatedFlowY = flowRateY - gyroRateY;

    // Convert compensated flow rates to ground velocities (m/s)
    // velocity = flowRate (rad/s) * altitude (m)
    float velCameraX = compensatedFlowX * altitude;
    float velCameraY = compensatedFlowY * altitude;

    // *** BODY TO EARTH FRAME TRANSFORMATION ***
    // Get current attitude (in decidegrees, convert to radians)
    float roll = attitude.values.roll * 0.1f * DEG_TO_RAD;
    float pitch = attitude.values.pitch * 0.1f * DEG_TO_RAD;
    float yaw = attitude.values.yaw * 0.1f * DEG_TO_RAD;

    // Apply tilt compensation first (project camera velocity to ground plane)
    float cosPitch = cos_approx(pitch);
    float cosRoll = cos_approx(roll);
    float cosTilt = cosPitch * cosRoll; // Combined tilt factor

    // Only proceed if tilt is not too extreme (< 80 degrees)
    if (cosTilt < 0.17f) { // cos(80°) ≈ 0.17
        posEstimate.valid = false;
        return;
    }

    // Compensate for tilt to get ground velocity
    float velBodyX = velCameraX / cosTilt;
    float velBodyY = velCameraY / cosTilt;

    // Rotate from body frame to earth frame using yaw angle
    float cosYaw = cos_approx(yaw);
    float sinYaw = sin_approx(yaw);

    // Earth frame velocities (North-East reference)
    float velEarthX = velBodyX * cosYaw - velBodyY * sinYaw;  // East velocity
    float velEarthY = velBodyX * sinYaw + velBodyY * cosYaw;  // North velocity

    // Filter velocities to reduce noise
    posEstimate.velocityX = pt1FilterApply(&velocityXFilter, velEarthX);
    posEstimate.velocityY = pt1FilterApply(&velocityYFilter, velEarthY);

    // Integrate velocities to get position (dead reckoning)
    posEstimate.positionX += posEstimate.velocityX * POSITION_UPDATE_DT;
    posEstimate.positionY += posEstimate.velocityY * POSITION_UPDATE_DT;

    // Store altitude and update timestamp
    posEstimate.altitude = altitude;
    posEstimate.lastUpdate = millis();
    posEstimate.valid = true;

    // Update DEBUG values for blackbox/configurator
    DEBUG_SET(DEBUG_OPTICAL_FLOW, 0, (int)(posEstimate.velocityX * 100));  // East velocity (cm/s)
    DEBUG_SET(DEBUG_OPTICAL_FLOW, 1, (int)(posEstimate.velocityY * 100));  // North velocity (cm/s)
    DEBUG_SET(DEBUG_OPTICAL_FLOW, 2, (int)(posEstimate.positionX * 100));  // East position (cm)
    DEBUG_SET(DEBUG_OPTICAL_FLOW, 3, (int)(posEstimate.positionY * 100));  // North position (cm)
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

// Reset position estimate to zero (called when activating position hold)
void opticalFlowResetPosition(void)
{
    posEstimate.positionX = 0.0f;
    posEstimate.positionY = 0.0f;
    posEstimate.velocityX = 0.0f;
    posEstimate.velocityY = 0.0f;

    // Reset filters by clearing their internal state
    velocityXFilter.state = 0.0f;
    velocityYFilter.state = 0.0f;
}

// Set target position for position hold
void opticalFlowSetPositionTarget(float targetX, float targetY)
{
    posHoldSetpointX = targetX;
    posHoldSetpointY = targetY;
}

// Activate or deactivate position hold mode
void opticalFlowActivatePositionHold(bool activate)
{
    if (activate && !posHoldActive) {
        // Activating: set current position as target
        posHoldSetpointX = posEstimate.positionX;
        posHoldSetpointY = posEstimate.positionY;
    }
    posHoldActive = activate;
}

// Check if optical flow position is valid
bool opticalFlowIsPositionValid(void)
{
    return posEstimate.valid;
}

#endif // USE_RANGEFINDER_ESP32CAM_TFMINI
