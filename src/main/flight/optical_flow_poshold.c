/*
 * Implementation of optical flow position estimation and control
 *
 * This integrates optical flow from any optical flow sensor with IMU data
 * for position hold flight mode.
 *
 * Key features:
 * - Body frame only (X: forward, Y: left)
 * - Gyroscopic compensation after transformation
 * - PID controller outputs pitch and roll angles directly
 * - Stick movement deactivates position hold
 */

#include "platform.h"

#ifdef USE_OPTICALFLOW

#include "optical_flow_poshold.h"
#include "sensors/opticalflow.h"
#include "drivers/rangefinder/rangefinder.h"
#include "sensors/rangefinder.h"
#include "sensors/gyro.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "fc/rc_controls.h"
#include "common/maths.h"
#include "common/filter.h"
#include "build/debug.h"
#include "drivers/time.h"
#include "pg/optical_flow_poshold.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

// Register parameter group for position hold configuration
PG_REGISTER_WITH_RESET_FN(opticalFlowPosHoldConfig_t, opticalFlowPosHoldConfig, PG_OF_POSHOLD_CONFIG, 0);

void pgResetFn_opticalFlowPosHoldConfig(opticalFlowPosHoldConfig_t *config)
{
    config->max_angle = 10;         // Default 10 degrees max autopilot angle
    config->pid_p = 50;             // Default P = 0.5 (stored as * 100)
    config->pid_i = 10;             // Default I = 0.1 (stored as * 100)
    config->pid_d = 5;              // Default D = 0.05 (stored as * 100)
    config->pid_i_max = 100;        // Default I_MAX = 10.0 (stored as * 10)
    config->stick_deadband = 12;    // Default stick deadband
}

// Position estimation state (body frame)
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

// Configuration
#define VELOCITY_FILTER_CUTOFF_HZ   5.0f
#define POSITION_UPDATE_RATE_HZ     50.0f
#define POSITION_UPDATE_DT          (1.0f / POSITION_UPDATE_RATE_HZ)
#define MIN_VALID_ALTITUDE_CM       10.0f   // Minimum altitude for valid flow (10cm)
#define MAX_VALID_ALTITUDE_CM       1200.0f // Maximum altitude for valid flow (12m)

// PID gains and stick deadband are now read from opticalFlowPosHoldConfig()
// Default values are set in pgResetFn_opticalFlowPosHoldConfig()

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
    if (!opticalflowIsValid()) {
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

    // Get raw optical flow rates (rad/s)
    // MAVLink convention: X is forward, Y is left
    float flowRateX = opticalflowGetFlowRateX();
    float flowRateY = opticalflowGetFlowRateY();

    // Convert flow rates to body-frame velocities (m/s)
    // velocity = flowRate (rad/s) * altitude (m)
    // IMPORTANT: Apply transformation first, THEN do gyro compensation
    float velBodyX_raw = flowRateX * altitude;   // Forward velocity (X axis)
    float velBodyY_raw = flowRateY * altitude;   // Left velocity (Y axis)

    // *** GYROSCOPIC COMPENSATION ***
    // Compensate gyro rotation AFTER transformation, on corresponding axes
    // gyro.gyroADCf is in deg/s, convert to rad/s
    const float DEG_TO_RAD = 0.017453292f;
    float gyroRatePitch = gyro.gyroADCf[FD_PITCH] * DEG_TO_RAD; // Pitch rate (rad/s)
    float gyroRateRoll = gyro.gyroADCf[FD_ROLL] * DEG_TO_RAD;   // Roll rate (rad/s)

    // Subtract gyro-induced apparent motion from velocities
    // Pitch rotation affects X velocity (forward/back)
    // Roll rotation affects Y velocity (left/right)
    float velBodyX = velBodyX_raw - (gyroRatePitch * altitude);
    float velBodyY = velBodyY_raw - (gyroRateRoll * altitude);

    // Filter velocities to reduce noise
    posEstimate.velocityX = pt1FilterApply(&velocityXFilter, velBodyX);
    posEstimate.velocityY = pt1FilterApply(&velocityYFilter, velBodyY);

    // Integrate velocities to get position (dead reckoning in body frame)
    posEstimate.positionX += posEstimate.velocityX * POSITION_UPDATE_DT;
    posEstimate.positionY += posEstimate.velocityY * POSITION_UPDATE_DT;

    // Store altitude and update timestamp
    posEstimate.altitude = altitude;
    posEstimate.lastUpdate = millis();
    posEstimate.valid = true;

    // Update DEBUG values for blackbox/configurator
    DEBUG_SET(DEBUG_OPTICALFLOW, 0, (int)(posEstimate.velocityX * 100));  // Body X velocity (cm/s)
    DEBUG_SET(DEBUG_OPTICALFLOW, 1, (int)(posEstimate.velocityY * 100));  // Body Y velocity (cm/s)
    DEBUG_SET(DEBUG_OPTICALFLOW, 2, (int)(posEstimate.positionX * 100));  // Body X position (cm)
    DEBUG_SET(DEBUG_OPTICALFLOW, 3, (int)(posEstimate.positionY * 100));  // Body Y position (cm)
    DEBUG_SET(DEBUG_OPTICALFLOW, 4, (int)(velBodyX_raw * 100));            // Raw body X velocity before gyro comp (cm/s)
    DEBUG_SET(DEBUG_OPTICALFLOW, 5, (int)(velBodyY_raw * 100));            // Raw body Y velocity before gyro comp (cm/s)
    DEBUG_SET(DEBUG_OPTICALFLOW, 6, (int)(gyroRatePitch * 1000));          // Pitch gyro rate (mrad/s)
    DEBUG_SET(DEBUG_OPTICALFLOW, 7, (int)(gyroRateRoll * 1000));           // Roll gyro rate (mrad/s)
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

// Check if optical flow position is valid
bool opticalFlowIsPositionValid(void)
{
    return posEstimate.valid;
}

#endif // USE_OPTICALFLOW
