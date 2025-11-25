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

#include "platform.h"

#ifndef USE_WING

#ifdef USE_POSITION_HOLD

#include "math.h"
#include "build/debug.h"
#include "common/maths.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/compass.h"

#include "pg/pos_hold.h"
#include "pos_hold.h"

#ifdef USE_OPTICALFLOW
#include "flight/optical_flow_poshold.h"
#include "sensors/rangefinder.h"
#endif

typedef enum {
    POS_HOLD_SOURCE_NONE = 0,
    POS_HOLD_SOURCE_GPS,
    POS_HOLD_SOURCE_OPTICAL_FLOW
} posHoldSource_e;

typedef struct posHoldState_s {
    bool isEnabled;
    bool isControlOk;
    bool areSensorsOk;
    float deadband;
    bool useStickAdjustment;
    posHoldSource_e source;  // Which sensor is being used for position hold
} posHoldState_t;

static posHoldState_t posHold;

void posHoldInit(void)
{
    posHold.deadband = posHoldConfig()->deadband * 0.01f;
    posHold.useStickAdjustment = posHoldConfig()->deadband;
}

static void posHoldCheckSticks(void)
{
    // if failsafe is active, eg landing mode, don't update the original start point
    if (!failsafeIsActive() && posHold.useStickAdjustment) {
        const bool sticksDeflected = (getRcDeflectionAbs(FD_ROLL) > posHold.deadband) || (getRcDeflectionAbs(FD_PITCH) > posHold.deadband);
        setSticksActiveStatus(sticksDeflected);
    }
}

static bool sensorsOk(void)
{
    // First priority: GPS with sufficient satellites and compass
    if (STATE(GPS_FIX) && gpsSol.numSat >= GPS_MIN_SAT_COUNT) {
        // Check compass requirement for GPS mode
#ifdef USE_MAG
        const bool compassOk = compassIsHealthy();
#else
        const bool compassOk = true; // No compass compiled in
#endif
        const bool canBypassCompass = posHoldConfig()->posHoldWithoutMag && canUseGPSHeading;

        if (compassOk || canBypassCompass) {
            // GPS is available with either compass or GPS heading
            posHold.source = POS_HOLD_SOURCE_GPS;
            return true;
        }
        // GPS available but compass requirement not met - fall through to try optical flow
    }

#ifdef USE_OPTICALFLOW
    // Second priority: Optical flow (independent of compass/mag)
    if (isOpticalFlowAvailable()) {
        posHold.source = POS_HOLD_SOURCE_OPTICAL_FLOW;
        return true;
    }
#endif

    // No valid position source available
    posHold.source = POS_HOLD_SOURCE_NONE;
    return false;
}

void updatePosHold(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!posHold.isEnabled) {
            // Check which sensor source is available
            posHold.areSensorsOk = sensorsOk();

            if (posHold.areSensorsOk) {
                // Initialize the appropriate controller
                if (posHold.source == POS_HOLD_SOURCE_GPS) {
                    resetPositionControl(&gpsSol.llh, POSHOLD_TASK_RATE_HZ);
                }
#ifdef USE_OPTICALFLOW
                else if (posHold.source == POS_HOLD_SOURCE_OPTICAL_FLOW) {
                    resetPositionControlOpticalFlow(POSHOLD_TASK_RATE_HZ);
                }
#endif
                posHold.isControlOk = true;
                posHold.isEnabled = true;
            }
        }
    } else {
        posHold.isEnabled = false;
        posHold.source = POS_HOLD_SOURCE_NONE;
    }

    if (posHold.isEnabled && posHold.isControlOk) {
        // CRITICAL: Always check sticks first, even if sensors fail
        // This ensures pilot can ALWAYS regain manual control
        posHoldCheckSticks();

        // Update sensor health status continuously, not just at initialization
        posHold.areSensorsOk = sensorsOk();

        if (posHold.areSensorsOk) {
            // Use the appropriate position controller based on source
            if (posHold.source == POS_HOLD_SOURCE_GPS) {
                posHold.isControlOk = positionControl();
            }
#ifdef USE_OPTICALFLOW
            else if (posHold.source == POS_HOLD_SOURCE_OPTICAL_FLOW) {
                // Update position estimate from optical flow sensor data
                opticalFlowEstimatePosition();
                // Run position control using the updated estimate
                posHold.isControlOk = positionControlOpticalFlow();
            }
#endif
            else {
                posHold.isControlOk = false;
            }
        }
    }
}

bool posHoldFailure(void) {
    // used only to display warning in OSD if requested but failing
    return FLIGHT_MODE(POS_HOLD_MODE) && (!posHold.isControlOk || !posHold.areSensorsOk);
}

#endif // USE_POSITION_HOLD

#endif // !USE_WING
