/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

#ifndef USE_WING

#include "build/debug.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/vector.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "pg/autopilot.h"
#include "autopilot.h"

#ifdef USE_OPTICALFLOW
#include "flight/optical_flow_poshold.h"
#include "sensors/opticalflow.h"
#include "sensors/rangefinder.h"
#endif

#define ALTITUDE_P_SCALE  0.01f
#define ALTITUDE_I_SCALE  0.003f
#define ALTITUDE_D_SCALE  0.01f
#define ALTITUDE_F_SCALE  0.01f
#define POSITION_P_SCALE  0.0012f
#define POSITION_I_SCALE  0.0001f
#define POSITION_D_SCALE  0.0015f
#define POSITION_A_SCALE  0.0008f
#define UPSAMPLING_CUTOFF_HZ 5.0f

static pidCoefficient_t altitudePidCoeffs;
static pidCoefficient_t positionPidCoeffs;

static float altitudeI = 0.0f;
static float throttleOut = 0.0f;
static float altitudePidP = 0.0f;
static float altitudePidD = 0.0f;

typedef struct efPidAxis_s {
    bool isStopping;
    float previousDistance;
    float previousVelocity;
    float integral;
    pt1Filter_t velocityLpf;
    pt1Filter_t accelerationLpf;
} efPidAxis_t;

typedef enum {
    // axes are for ENU system; it is different from current Betaflight code
    LON = 0,   // X, east
    LAT        // Y, north
} axisEF_e;

typedef struct autopilotState_s {
    gpsLocation_t targetLocation;       // active / current target
    float sanityCheckDistance;
    float upsampleLpfGain;              // for the Body Frame upsample filter for pitch and roll
    float vaLpfCutoff;                  // velocity + acceleration lowpass filter cutoff
    bool sticksActive;
    float maxAngle;
    vector2_t pidSumBF;                 // pid output, updated on each GPS update, rotated to body frame
    pt3Filter_t upsampleLpfBF[RP_AXIS_COUNT];    // upsampling filter
    efPidAxis_t efAxis[EF_AXIS_COUNT];
} autopilotState_t;

static autopilotState_t ap = {
    .sanityCheckDistance = 1000.0f,
    .upsampleLpfGain = 1.0f,
    .vaLpfCutoff = 1.0f,
    .sticksActive = false,
};

float autopilotAngle[RP_AXIS_COUNT];

static void resetEFAxisFilters(efPidAxis_t* efAxis, const float vaGain)
{
    pt1FilterInit(&efAxis->velocityLpf, vaGain);
    pt1FilterInit(&efAxis->accelerationLpf, vaGain);
}

static void resetEFAxisParams(efPidAxis_t *efAxis, const float vaGain)
{
    // at start only
    resetEFAxisFilters(efAxis, vaGain);
    efAxis->isStopping = true; // Enter starting (deceleration) 'phase'
    efAxis->integral = 0.0f;
}

static void resetUpsampleFilters(void)
{
    for (unsigned i = 0; i < ARRAYLEN(ap.upsampleLpfBF); i++) {
        pt3FilterInit(&ap.upsampleLpfBF[i], ap.upsampleLpfGain);
    }
}

// get sanity distance based on speed
static inline float sanityCheckDistance(const float gpsGroundSpeedCmS)
{
    return fmaxf(1000.0f, gpsGroundSpeedCmS * 2.0f);
    // distance flown in 2s at current speed. with minimum of 10m
}

void resetPositionControl(const gpsLocation_t *initialTargetLocation, unsigned taskRateHz)
{
    // from pos_hold.c (or other client) when initiating position hold at target location
    ap.targetLocation = *initialTargetLocation;
    ap.sticksActive = false;
    // set sanity check distance according to groundspeed at start, minimum of 10m
    ap.sanityCheckDistance = sanityCheckDistance(gpsSol.groundSpeed);
    for (unsigned i = 0; i < ARRAYLEN(ap.efAxis); i++) {
        // clear anything stored in the filter at first call
        resetEFAxisParams(&ap.efAxis[i], 1.0f);
    }
    const float taskInterval = 1.0f / taskRateHz;
    ap.upsampleLpfGain = pt3FilterGain(UPSAMPLING_CUTOFF_HZ, taskInterval); // 5Hz; normally at 100Hz task rate
    resetUpsampleFilters(); // clear accumlator from previous iterations
}

void autopilotInit(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    ap.sticksActive = false;
    ap.maxAngle = cfg->maxAngle;
    altitudePidCoeffs.Kp = cfg->altitudeP * ALTITUDE_P_SCALE;
    altitudePidCoeffs.Ki = cfg->altitudeI * ALTITUDE_I_SCALE;
    altitudePidCoeffs.Kd = cfg->altitudeD * ALTITUDE_D_SCALE;
    altitudePidCoeffs.Kf = cfg->altitudeF * ALTITUDE_F_SCALE;
    positionPidCoeffs.Kp = cfg->positionP * POSITION_P_SCALE;
    positionPidCoeffs.Ki = cfg->positionI * POSITION_I_SCALE;
    positionPidCoeffs.Kd = cfg->positionD * POSITION_D_SCALE;
    positionPidCoeffs.Kf = cfg->positionA * POSITION_A_SCALE; // Kf used for acceleration
    // initialise filters with approximate filter gains; location isn't used at this point.
    ap.upsampleLpfGain = pt3FilterGain(UPSAMPLING_CUTOFF_HZ, 0.01f); // 5Hz, assuming 100Hz task rate at init
    resetUpsampleFilters();
    // Initialise PT1 filters for velocity and acceleration in earth frame axes
    ap.vaLpfCutoff = cfg->positionCutoff * 0.01f;
    const float vaGain = pt1FilterGain(ap.vaLpfCutoff,  0.1f); // assume 10Hz GPS connection at start; value is overwritten before first filter use
    for (unsigned i = 0; i < ARRAYLEN(ap.efAxis); i++) {
        resetEFAxisFilters(&ap.efAxis[i], vaGain);
    }
}

void resetAltitudeControl (void) {
    altitudeI = 0.0f;
}

void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeStep)
{
    // Use lidar-aware functions for altitude hold to prioritize rangefinder when available
    const float verticalVelocityCmS = getAltitudeDerivativeForAltHold();
    const float altitudeErrorCm = targetAltitudeCm - getAltitudeCmForAltHold();
    altitudePidP = altitudeErrorCm * altitudePidCoeffs.Kp;

    // reduce the iTerm gain for errors greater than 200cm (2m), otherwise it winds up too much
    const float itermRelax = (fabsf(altitudeErrorCm) < 200.0f) ? 1.0f : 0.1f;
    altitudeI += altitudeErrorCm * altitudePidCoeffs.Ki * itermRelax * taskIntervalS;
    // limit iTerm to not more than 200 throttle units
    altitudeI = constrainf(altitudeI, -200.0f, 200.0f);

    // increase D when velocity is high, typically when initiating hold at high vertical speeds
    // 1.0 when less than 5 m/s, 2x at 10m/s, 2.5 at 20 m/s, 2.8 at 50 m/s, asymptotes towards max 3.0.
    float dBoost = 1.0f;
    {
        const float startValue = 500.0f; // velocity at which D should start to increase
        const float altDeriv = fabsf(verticalVelocityCmS);
        if (altDeriv > startValue) {
            const float ratio = altDeriv / startValue;
            dBoost = (3.0f * ratio - 2.0f) / ratio;
        }
    }

    altitudePidD = verticalVelocityCmS * dBoost * altitudePidCoeffs.Kd;

    const float altitudeF = targetAltitudeStep * altitudePidCoeffs.Kf;

    const float minThrottle = MAX(rxConfig()->mincheck, PWM_RANGE_MIN);
    const float hoverOffset = autopilotConfig()->hoverThrottle - minThrottle;
    float throttleOffset = altitudePidP + altitudeI - altitudePidD + altitudeF + hoverOffset;

    const float tiltMultiplier = 1.0f / fmaxf(getCosTiltAngle(), 0.5f);
    // 1 = flat, 1.3 at 40 degrees, 1.56 at 50 deg, max 2.0 at 60 degrees or higher
    // note: the default limit of Angle Mode is 60 degrees

    throttleOffset *= tiltMultiplier;

    float newThrottle = minThrottle + throttleOffset;
    newThrottle = constrainf(newThrottle, autopilotConfig()->throttleMin, autopilotConfig()->throttleMax);
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 0, lrintf(newThrottle)); // normal range 1000-2000 but is after constraint

    newThrottle = scaleRangef(newThrottle, minThrottle, PWM_RANGE_MAX, 0.0f, 1.0f);

    throttleOut = constrainf(newThrottle, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 1, lrintf(tiltMultiplier * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 3, lrintf(targetAltitudeCm));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 4, lrintf(altitudePidP));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 5, lrintf(altitudeI));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 6, lrintf(-altitudePidD));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 7, lrintf(altitudeF));

    // DEBUG_ALT_HOLD_RF mode for rangefinder altitude hold diagnostics
    DEBUG_SET(DEBUG_ALT_HOLD_RF, 0, lrintf(rangefinderGetLatestAltitude()));  // Rangefinder altitude (cm)
    DEBUG_SET(DEBUG_ALT_HOLD_RF, 1, rangefinderIsSurfaceAltitudeValid() ? 100 : 0);  // Validity (100=valid, 0=invalid)
    DEBUG_SET(DEBUG_ALT_HOLD_RF, 2, lrintf(targetAltitudeCm));  // Altitude setpoint (cm)
    DEBUG_SET(DEBUG_ALT_HOLD_RF, 3, lrintf(altitudeErrorCm));  // Altitude error (cm)
    DEBUG_SET(DEBUG_ALT_HOLD_RF, 4, lrintf(altitudePidP));  // P component
    DEBUG_SET(DEBUG_ALT_HOLD_RF, 5, lrintf(altitudeI));  // I component
    DEBUG_SET(DEBUG_ALT_HOLD_RF, 6, lrintf(-altitudePidD));  // D component (negated for display)
    DEBUG_SET(DEBUG_ALT_HOLD_RF, 7, lrintf(altitudeF));  // F component
}

void setSticksActiveStatus(bool areSticksActive)
{
    ap.sticksActive = areSticksActive;
}

static void setTargetLocationByAxis(const gpsLocation_t* newTargetLocation, axisEF_e efAxisIdx)
// not used at present but needed by upcoming GPS code
{
    if (efAxisIdx == LON) {
        ap.targetLocation.lon = newTargetLocation->lon; // update East-West / / longitude position
    } else {
        ap.targetLocation.lat = newTargetLocation->lat; // update North-South / latitude position
    }
}

bool positionControl(void)
{
    unsigned debugAxis = gyroConfig()->gyro_filter_debug_axis;
    static vector2_t debugGpsDistance = { 0 };     // keep last calculated distance for DEBUG
    static vector2_t debugPidSumEF = { 0 };        // and last pidsum in EF
    static uint16_t gpsStamp = 0;
    if (gpsHasNewData(&gpsStamp)) {
        const float gpsDataInterval = getGpsDataIntervalSeconds(); // interval for current GPS data value 0.05 - 2.5s
        const float gpsDataFreq = getGpsDataFrequencyHz();

        // get lat and long distances from current location (gpsSol.llh) to target location
        vector2_t gpsDistance;
        GPS_distance2d(&gpsSol.llh, &ap.targetLocation, &gpsDistance); // X is EW/lon, Y is NS/lat
        debugGpsDistance = gpsDistance;
        const float distanceNormCm = vector2Norm(&gpsDistance);

        // ** Sanity check **
        // primarily to detect flyaway from no Mag or badly oriented Mag
        // must accept some overshoot at the start, especially if entering at high speed
        if (distanceNormCm > ap.sanityCheckDistance) {
            return false;
        }

        // update filters according to current GPS update rate
        const float vaGain = pt1FilterGain(ap.vaLpfCutoff, gpsDataInterval);
        const float iTermLeakGain = 1.0f - pt1FilterGainFromDelay(2.5f, gpsDataInterval);   // 2.5s time constant
        vector2_t pidSum = { 0 };       // P+I in loop, D+A added after the axis loop (after limiting it)
        vector2_t pidDA;                // D+A

        for (axisEF_e efAxisIdx = LON; efAxisIdx <= LAT; efAxisIdx++) {
            efPidAxis_t *efAxis = &ap.efAxis[efAxisIdx];
            // separate PID controllers for longitude (EastWest or EW, X) and latitude (NorthSouth or NS, Y)
            const float axisDistance = gpsDistance.v[efAxisIdx];

            // ** P **
            const float pidP = axisDistance * positionPidCoeffs.Kp;
            pidSum.v[efAxisIdx] += pidP;

            // ** I **
            // only add to iTerm while in hold phase
            efAxis->integral += efAxis->isStopping ? 0.0f : axisDistance * gpsDataInterval;
            const float pidI = efAxis->integral * positionPidCoeffs.Ki;
            pidSum.v[efAxisIdx] += pidI;

            // ** D ** //
            // Velocity derived from GPS position works better than module supplied GPS Speed and Heading information

            const float velocity = (axisDistance - efAxis->previousDistance) * gpsDataFreq; // cm/s
            efAxis->previousDistance = axisDistance;
            pt1FilterUpdateCutoff(&efAxis->velocityLpf, vaGain);
            const float velocityFiltered = pt1FilterApply(&efAxis->velocityLpf, velocity);
            float pidD = velocityFiltered * positionPidCoeffs.Kd;

            // differentiate velocity another time to get acceleration
            float acceleration = (velocityFiltered - efAxis->previousVelocity) * gpsDataFreq;
            efAxis->previousVelocity = velocityFiltered;
            // apply second filter to acceleration (acc is filtered twice)
            pt1FilterUpdateCutoff(&efAxis->accelerationLpf, vaGain);
            const float accelerationFiltered = pt1FilterApply(&efAxis->accelerationLpf, acceleration);
            const float pidA = accelerationFiltered * positionPidCoeffs.Kf;

            if (ap.sticksActive) {
                // sticks active 'phase', prepare to enter stopping
                efAxis->isStopping = true;
                // slowly leak iTerm away
                efAxis->integral *= iTermLeakGain;
                efAxis->previousDistance = 0.0f; // avoid D and A spikes
                // rest is handled after axis loop
            } else if (efAxis->isStopping) {
                // 'phase' after sticks are centered, but before craft has stopped; in given Earth axis
                pidD *= 1.6f; // aribitrary D boost to stop more quickly than usual
                // detect when axis has nearly stopped by sign reversal of velocity (comparing sign of velocityFiltered, which is delayed, to velocity)
                if (velocity * velocityFiltered < 0.0f) {
                    setTargetLocationByAxis(&gpsSol.llh, efAxisIdx);  // reset target location for this axis, forcing P to zero
                    efAxis->previousDistance = 0.0f;                  // ensure minimal D jump from the updated location
                    efAxis->isStopping = false;                       // end the 'stopping' phase
                    if (ap.efAxis[LAT].isStopping == ap.efAxis[LON].isStopping) {
                        // when both axes have stopped moving, reset the sanity distance to 10m default
                        ap.sanityCheckDistance = sanityCheckDistance(1000);
                    }
                }
            }
            pidDA.v[efAxisIdx] = pidD + pidA;    // save DA here, processed after axis loop
            if (debugAxis == efAxisIdx) {
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(distanceNormCm));   // same for both axes
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(pidP * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(pidI * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(pidD * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(pidA * 10));
            }
        } // end for loop

        {
            // limit sum of D and A per axis based on total DA vector length, otherwise can be too aggressive when starting at speed
            // limit is 35 degrees from D and A alone, arbitrary value.  20 is a bit too low, allows a lot of overshoot
            // note: an angle of more than 35 degrees can still be achieved as P and I grow
            const float maxDAAngle = 35.0f; // D+A limit in degrees; arbitrary angle
            const float mag = vector2Norm(&pidDA);
            if (mag > maxDAAngle) {
                vector2Scale(&pidDA, &pidDA, maxDAAngle / mag);
            }
        }

        // add constrained DA to sum
        vector2Add(&pidSum, &pidSum, &pidDA);
        debugPidSumEF = pidSum;
        vector2_t anglesBF;

        if (ap.sticksActive) {
            // if a Position Hold deadband is set, and sticks are outside deadband, allow pilot control in angle mode
            anglesBF = (vector2_t){{0, 0}};             // set output PIDS to 0; upsampling filter will smooth this
            // reset target location each cycle (and set previousDistance to zero in for loop), to keep D current, and avoid a spike when stopping
            ap.targetLocation = gpsSol.llh;
            // keep updating sanity check distance while sticks are out because speed may get high
            ap.sanityCheckDistance = sanityCheckDistance(gpsSol.groundSpeed);
        } else {
            // ** Rotate pid Sum to body frame, and convert it into pitch and roll **
            // attitude.values.yaw increases clockwise from north
            // PID is running in ENU, adapt angle (to 0deg = EAST);
            //  rotation is from EarthFrame to BodyFrame, no change of sign from heading
            const float angle = DECIDEGREES_TO_RADIANS(attitude.values.yaw - 900);
            vector2_t pidBodyFrame;   // pid output in body frame; X is forward, Y is left
            vector2Rotate(&pidBodyFrame, &pidSum, angle);         // rotate by angle counterclockwise
            anglesBF.v[AI_ROLL] = -pidBodyFrame.y;         // negative roll to fly left
            anglesBF.v[AI_PITCH] = pidBodyFrame.x;         // positive pitch for forward
             // limit angle vector to maxAngle
            const float mag = vector2Norm(&anglesBF);
            if (mag > ap.maxAngle && mag > 0.0f) {
                vector2Scale(&anglesBF, &anglesBF, ap.maxAngle / mag);
            }
        }
        ap.pidSumBF = anglesBF;    // this value will be upsampled
    }

    // Final output to pid.c Angle Mode at 100Hz with PT3 upsampling
    for (unsigned i = 0; i < RP_AXIS_COUNT; i++) {
        // note: upsampling should really be done in earth frame, to avoid 10Hz wobbles if pilot yaws and the controller is applying significant pitch or roll
        autopilotAngle[i] = pt3FilterApply(&ap.upsampleLpfBF[i], ap.pidSumBF.v[i]);
    }

    if (debugAxis < 2) {
        // this is different from @ctzsnooze version
        // debugAxis = 0: store longitude + roll
        // debugAxis = 1: store latitude + pitch
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(debugGpsDistance.v[debugAxis]));    // cm
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(debugPidSumEF.v[debugAxis] * 10));  // deg * 10
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(autopilotAngle[debugAxis] * 10));   // deg * 10
    }
    return true;
}

bool isBelowLandingAltitude(void)
{
    return getAltitudeCm() < 100.0f * autopilotConfig()->landingAltitudeM;
}

float getAutopilotThrottle(void)
{
    return throttleOut;
}

bool isAutopilotInControl(void)
{
    return !ap.sticksActive;
}

float getAltitudePidP(void)
{
    return altitudePidP;
}

float getAltitudePidI(void)
{
    return altitudeI;
}

float getAltitudePidD(void)
{
    return altitudePidD;
}

#ifdef USE_OPTICALFLOW
/*
 * Position control using optical flow instead of GPS
 * This is used as a fallback when GPS is not available or has insufficient satellites
 */

typedef struct opticalFlowAutopilotState_s {
    vector2_t targetPosition;       // Target position in cm (X=East, Y=North)
    vector2_t integral;             // Integral term for each axis
    float maxAngle;
    vector2_t pidSumBF;             // PID output in body frame
    pt3Filter_t upsampleLpfBF[RP_AXIS_COUNT];
    float upsampleLpfGain;
    bool initialized;
} opticalFlowAutopilotState_t;

static opticalFlowAutopilotState_t ofAp = {
    .maxAngle = 30.0f,
    .upsampleLpfGain = 1.0f,
    .initialized = false,
    .integral = {{0.0f, 0.0f}},
};

// Reset optical flow position control (called when entering POS_HOLD with optical flow)
void resetPositionControlOpticalFlow(unsigned taskRateHz)
{
    // Reset optical flow position to zero
    opticalFlowResetPosition();

    // Set target to current position (0, 0 since we just reset)
    ofAp.targetPosition = (vector2_t){{0.0f, 0.0f}};

    // Reset integral terms
    ofAp.integral = (vector2_t){{0.0f, 0.0f}};

    // Initialize upsampling filters
    const float taskInterval = 1.0f / taskRateHz;
    ofAp.upsampleLpfGain = pt3FilterGain(UPSAMPLING_CUTOFF_HZ, taskInterval);
    for (unsigned i = 0; i < ARRAYLEN(ofAp.upsampleLpfBF); i++) {
        pt3FilterInit(&ofAp.upsampleLpfBF[i], ofAp.upsampleLpfGain);
    }

    ofAp.maxAngle = autopilotConfig()->maxAngle;
    ofAp.initialized = true;
}

// Position control using optical flow
bool positionControlOpticalFlow(void)
{
    if (!ofAp.initialized) {
        return false;
    }

    // Check if optical flow position is valid
    if (!opticalFlowIsPositionValid()) {
        return false;
    }

    // Get current position estimate from optical flow (in m, convert to cm)
    positionEstimate_t posEstimate;
    if (!opticalFlowGetPositionEstimate(&posEstimate)) {
        return false;
    }

    // Calculate position error in cm (body frame: X=forward, Y=left)
    vector2_t positionError;
    positionError.x = ofAp.targetPosition.x - (posEstimate.positionX * 100.0f); // Forward error (cm)
    positionError.y = ofAp.targetPosition.y - (posEstimate.positionY * 100.0f); // Left error (cm)

    // Calculate velocity in cm/s (body frame)
    vector2_t velocity;
    velocity.x = posEstimate.velocityX * 100.0f; // Forward velocity (cm/s)
    velocity.y = posEstimate.velocityY * 100.0f; // Left velocity (cm/s)

    // Update rate: assume 10Hz optical flow updates (100ms)
    const float dataInterval = 0.1f;
    const float iTermLeakGain = 1.0f - pt1FilterGainFromDelay(2.5f, dataInterval); // 2.5s time constant

    // PID controller in body frame - no rotation needed!
    // X axis (forward) controls PITCH, Y axis (left) controls ROLL

    // --- X axis (forward → pitch) ---
    const float pidPx = positionError.x * positionPidCoeffs.Kp;

    if (!ap.sticksActive) {
        ofAp.integral.x += positionError.x * dataInterval;
        ofAp.integral.x = constrainf(ofAp.integral.x, -10000.0f, 10000.0f);
    } else {
        ofAp.integral.x *= iTermLeakGain;
    }
    const float pidIx = ofAp.integral.x * positionPidCoeffs.Ki;
    const float pidDx = velocity.x * positionPidCoeffs.Kd;
    const float pitchAngle = pidPx + pidIx - pidDx; // Forward error → positive pitch

    // --- Y axis (left → roll) ---
    const float pidPy = positionError.y * positionPidCoeffs.Kp;

    if (!ap.sticksActive) {
        ofAp.integral.y += positionError.y * dataInterval;
        ofAp.integral.y = constrainf(ofAp.integral.y, -10000.0f, 10000.0f);
    } else {
        ofAp.integral.y *= iTermLeakGain;
    }
    const float pidIy = ofAp.integral.y * positionPidCoeffs.Ki;
    const float pidDy = velocity.y * positionPidCoeffs.Kd;
    const float rollAngle = -(pidPy + pidIy - pidDy); // Left error → negative roll

    vector2_t anglesBF;

    if (ap.sticksActive) {
        // Sticks are active: zero autopilot output, give full control to pilot (match GPS behavior)
        anglesBF = (vector2_t){{0, 0}};
        // Update target to current position so position hold resumes smoothly when sticks released
        ofAp.targetPosition.x = posEstimate.positionX * 100.0f;
        ofAp.targetPosition.y = posEstimate.positionY * 100.0f;
    } else {
        // Apply PID output
        anglesBF.v[AI_PITCH] = pitchAngle;
        anglesBF.v[AI_ROLL] = rollAngle;

        // Limit angle vector to maxAngle
        const float mag = vector2Norm(&anglesBF);
        if (mag > ofAp.maxAngle && mag > 0.0f) {
            vector2Scale(&anglesBF, &anglesBF, ofAp.maxAngle / mag);
        }
    }

    ofAp.pidSumBF = anglesBF;

    // Apply upsampling filter and output to autopilotAngle
    for (unsigned i = 0; i < RP_AXIS_COUNT; i++) {
        autopilotAngle[i] = pt3FilterApply(&ofAp.upsampleLpfBF[i], ofAp.pidSumBF.v[i]);
    }

    // Debug output for DEBUG_AUTOPILOT_POSITION
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(vector2Norm(&positionError)));     // Distance to target (cm)
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(positionError.x));                 // X forward error (cm)
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(pitchAngle * 10));                 // Pitch angle output (deg × 10)
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(autopilotAngle[AI_PITCH] * 10));  // Pitch angle after upsampling (deg × 10)

    // Debug mode for optical flow position hold (X-axis body frame only)
    // TEMPORARILY COMMENTED - Debug moved to optical_flow_poshold.c to show velocity processing stages
    // DEBUG_SET(DEBUG_POS_HOLD_OF, 0, lrintf(velocity.x));                         // Velocity X forward (cm/s)
    // DEBUG_SET(DEBUG_POS_HOLD_OF, 1, lrintf(posEstimate.positionX * 100.0f));    // Estimated position X forward (cm)
    // DEBUG_SET(DEBUG_POS_HOLD_OF, 2, lrintf(positionError.x));                    // Position error X forward (cm)
    // DEBUG_SET(DEBUG_POS_HOLD_OF, 3, lrintf(pidPx * 10));                         // P component (deg × 10)
    // DEBUG_SET(DEBUG_POS_HOLD_OF, 4, lrintf(pidIx * 10));                         // I component (deg × 10)
    // DEBUG_SET(DEBUG_POS_HOLD_OF, 5, lrintf(pidDx * 10));                         // D component (deg × 10)
    // DEBUG_SET(DEBUG_POS_HOLD_OF, 6, lrintf(ofAp.integral.x));                    // Integral accumulator X (cm·s)
    // DEBUG_SET(DEBUG_POS_HOLD_OF, 7, ap.sticksActive);                            // Sticks active flag

    return true;
}

// Check if optical flow is available and healthy
bool isOpticalFlowAvailable(void)
{
    // Clear separation of concerns: data available AND in range
    // Immediate response to data state changes - NO OR masking for immediate recovery

    // Rangefinder: must have data AND be in valid range (configurable via CLI)
    // Uses centralized validation - single source of truth
    bool rangefinderOk = sensors(SENSOR_RANGEFINDER) && rangefinderIsValid();

    // Optical flow: must have recent data (100ms timeout) with quality > 0
    // Uses centralized validation hierarchy
    bool opticalFlowOk = sensors(SENSOR_OPTICALFLOW) && opticalflowIsValid();

    return rangefinderOk && opticalFlowOk;
}
#endif // USE_OPTICALFLOW

#endif // !USE_WING
