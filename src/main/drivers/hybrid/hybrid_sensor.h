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
 * Generic interface for hybrid sensors
 *
 * Hybrid sensors are devices that provide multiple sensor capabilities
 * in a single physical package. Common combinations include:
 * - Rangefinder + Optical Flow (e.g., ESP32, MTF-02)
 * - IMU + Magnetometer
 * - Any other multi-sensor fusion device
 *
 * This interface allows hybrid sensors to provide data to multiple
 * sensor subsystems (rangefinder, optical flow, etc.) from a single
 * hardware driver and communication interface.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Forward declarations
struct rangefinderDev_s;
struct opticalFlowDev_s;

// Hybrid sensor capability flags
typedef enum {
    HYBRID_CAPABILITY_RANGEFINDER = (1 << 0),
    HYBRID_CAPABILITY_OPTICAL_FLOW = (1 << 1),
    // Future capabilities can be added here:
    // HYBRID_CAPABILITY_IMU = (1 << 2),
    // HYBRID_CAPABILITY_MAGNETOMETER = (1 << 3),
} hybridSensorCapability_e;

// Hybrid sensor types
typedef enum {
    HYBRID_SENSOR_NONE = 0,
    HYBRID_SENSOR_ESP32,        // ESP32-based hybrid sensor (rangefinder + optical flow)
    HYBRID_SENSOR_MTF02,        // MTF-02 hybrid sensor (rangefinder + optical flow)
    // Add more hybrid sensor types here
    HYBRID_SENSOR_COUNT
} hybridSensorType_e;

// Hybrid sensor device structure
typedef struct hybridSensorDev_s {
    hybridSensorType_e type;
    uint32_t capabilities;      // Bitmask of hybridSensorCapability_e

    // Pointers to capability-specific device structures
    struct rangefinderDev_s *rangefinder;
    struct opticalFlowDev_s *opticalFlow;

    // Generic sensor status
    bool initialized;
    uint32_t lastUpdateTime;
} hybridSensorDev_t;

// Hybrid sensor detection and initialization
bool hybridSensorDetect(hybridSensorDev_t *dev);
bool hybridSensorInit(hybridSensorDev_t *dev);
void hybridSensorUpdate(hybridSensorDev_t *dev);

// Get the current hybrid sensor (if any)
hybridSensorDev_t* getHybridSensor(void);
bool isHybridSensorAvailable(void);
