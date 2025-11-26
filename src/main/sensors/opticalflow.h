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
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "drivers/opticalflow/opticalflow.h"

#include "pg/pg.h"

typedef enum {
    OPTICALFLOW_NONE = 0,
    OPTICALFLOW_MT = 1,
    OPTICALFLOW_HYBRID_ESP32 = 2,    // ESP32 hybrid sensor (rangefinder + optical flow)
    OPTICALFLOW_HYBRID_MTF02 = 3,    // MTF-02 hybrid sensor (rangefinder + optical flow)
} opticalflowType_e;

typedef struct opticalflowConfig_s {
    uint8_t  opticalflow_hardware;
    uint16_t rotation;
    uint8_t  flip_x;
    uint16_t flow_lpf;
} opticalflowConfig_t;

PG_DECLARE(opticalflowConfig_t, opticalflowConfig);

typedef struct opticalflow_s {
    opticalflowDev_t dev;
    int16_t quality;
    vector2_t rawFlowRates;
    vector2_t processedFlowRates;
    uint32_t timeStampUs;
} opticalflow_t;

bool opticalflowInit(void);

void opticalflowReadSensor(void);
bool isOpticalflowHealthy(void);
void opticalflowProcess(void);

// Generic optical flow data access for position hold
float opticalflowGetFlowRateX(void);
float opticalflowGetFlowRateY(void);

// Validation functions with clear hierarchy
bool opticalflowHasData(void);
bool opticalflowIsValid(void);

// Diagnostic struct for debugging/blackbox analysis
typedef struct {
    bool hardwareResponding;   // Within 100ms timeout
    bool hasData;              // Quality > 0
    bool fullyValid;           // Both checks pass
    int16_t quality;           // Current quality value
    uint32_t lastUpdateUs;     // Timestamp of last update
} opticalflowState_t;

void opticalflowGetState(opticalflowState_t *state);
