/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "drivers/rangefinder/rangefinder.h"

#include "pg/pg.h"

typedef enum {
    RANGEFINDER_NONE        = 0,
    RANGEFINDER_HCSR04      = 1,
    RANGEFINDER_TFMINI      = 2,
    RANGEFINDER_TF02        = 3,
    RANGEFINDER_MTF01       = 4,
    RANGEFINDER_MTF02       = 5,
    RANGEFINDER_MTF01P      = 6,
    RANGEFINDER_MTF02P      = 7,
    RANGEFINDER_TFNOVA      = 8,
    RANGEFINDER_ESP32CAM_TFMINI = 9,
    RANGEFINDER_MTF02_HYBRID = 10,
} rangefinderType_e;

typedef struct rangefinderConfig_s {
    uint8_t rangefinder_hardware;
    uint16_t min_valid_altitude_cm;  // Minimum valid altitude for position hold (default: 10cm)
    uint16_t max_valid_altitude_cm;  // Maximum valid altitude for position hold (default: 1200cm)
} rangefinderConfig_t;

PG_DECLARE(rangefinderConfig_t, rangefinderConfig);

typedef struct rangefinder_s {
    rangefinderDev_t dev;
    float maxTiltCos;
    int32_t rawAltitude;
    int32_t calculatedAltitude;
    timeMs_t lastValidResponseTimeMs;

    bool snrThresholdReached;
    int32_t dynamicDistanceThreshold;
    int16_t snr;
    rangefinderType_e sensorType;  // Store sensor type to avoid reading detectedSensors repeatedly
} rangefinder_t;

void rangefinderResetDynamicThreshold(void);
bool rangefinderInit(void);

int32_t rangefinderGetLatestAltitude(void);
int32_t rangefinderGetLatestRawAltitude(void);

void rangefinderUpdate(void);
bool rangefinderProcess(float cosTiltAngle);
bool rangefinderIsHealthy(void);
bool rangefinderIsSurfaceAltitudeValid(void);

// New validation functions for clear hierarchy
bool rangefinderHasData(void);
bool rangefinderIsInValidRange(void);
bool rangefinderIsValid(void);

// Diagnostic struct for debugging/blackbox analysis
typedef struct {
    bool hardwareResponding;   // Sensor within 500ms timeout
    bool hasData;              // rawAltitude > 0
    bool inValidRange;         // Within configured min/max
    bool fullyValid;           // All checks pass
    int32_t rawAltitude;       // Current reading
} rangefinderState_t;

void rangefinderGetState(rangefinderState_t *state);
