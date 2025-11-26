/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Rangefinder-based Altitude Hold Implementation
 *
 * This module provides generic rangefinder support for altitude hold.
 * It works with any rangefinder type, including hybrid sensors.
 */

#include "platform.h"

#ifdef USE_RANGEFINDER

#include "alt_hold_rangefinder.h"

#include "sensors/rangefinder.h"
#include "sensors/sensors.h"
#include "common/filter.h"
#include "common/maths.h"
#include "fc/runtime_config.h"

// Rangefinder altitude hold state
typedef struct {
    float previousAltitudeCm;
    float altitudeDerivative;
    pt2Filter_t derivativeLpf;
    bool filterInitialized;
} rangefinderAltHoldState_t;

static rangefinderAltHoldState_t state = {
    .previousAltitudeCm = 0.0f,
    .altitudeDerivative = 0.0f,
    .filterInitialized = false
};

// Initialize rangefinder altitude hold
void rangefinderAltHoldInit(void)
{
    state.previousAltitudeCm = 0.0f;
    state.altitudeDerivative = 0.0f;
    state.filterInitialized = false;
}

// Check if rangefinder is valid for altitude hold
// Uses centralized validation for consistency with position hold
bool rangefinderAltHoldIsValid(void)
{
    // Delegate to centralized validation
    // This ensures consistency across altitude hold and position hold
    // and uses CLI-configurable range limits
    return sensors(SENSOR_RANGEFINDER) && rangefinderIsValid();
}

// Get rangefinder altitude in centimeters
float rangefinderAltHoldGetAltitudeCm(void)
{
    // Use centralized validation - ensures consistency
    if (!rangefinderIsValid()) {
        return -1.0f;  // Invalid: no data or out of range
    }

    // Try to get the processed altitude (with tilt correction, etc.)
    if (rangefinderIsHealthy() && rangefinderIsSurfaceAltitudeValid()) {
        const int32_t calculatedAlt = rangefinderGetLatestAltitude();
        // Validate processed altitude is also in valid range (uses CLI config)
        if (calculatedAlt >= rangefinderConfig()->min_valid_altitude_cm &&
            calculatedAlt <= rangefinderConfig()->max_valid_altitude_cm) {
            return (float)calculatedAlt;
        }
    }

    // Fall back to raw altitude (already validated by rangefinderIsValid())
    return (float)rangefinderGetLatestRawAltitude();
}

// Get rangefinder altitude derivative (velocity) in cm/s
float rangefinderAltHoldGetDerivativeCmS(float dt)
{
    if (!rangefinderAltHoldIsValid()) {
        return 0.0f;
    }

    const float currentAltitudeCm = rangefinderAltHoldGetAltitudeCm();
    if (currentAltitudeCm < 0.0f) {
        return 0.0f;
    }

    // Initialize filter on first use
    if (!state.filterInitialized) {
        const float lidarDerivativeFilterCutoffHz = 2.0f;
        const float lidarDerivativeFilterGain = pt2FilterGain(lidarDerivativeFilterCutoffHz, dt);
        pt2FilterInit(&state.derivativeLpf, lidarDerivativeFilterGain);
        state.previousAltitudeCm = currentAltitudeCm;
        state.filterInitialized = true;
        return 0.0f;
    }

    // Calculate derivative (velocity)
    const float rawDerivative = (currentAltitudeCm - state.previousAltitudeCm) / dt;
    state.previousAltitudeCm = currentAltitudeCm;

    // Apply filtering to smooth out noise
    state.altitudeDerivative = pt2FilterApply(&state.derivativeLpf, rawDerivative);

    return state.altitudeDerivative;
}

#endif // USE_RANGEFINDER
