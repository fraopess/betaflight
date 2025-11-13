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
 * Configuration and CLI parameters for ESP32Cam-TFMini sensor
 * 
 * This file contains the parameter group definition and CLI commands
 * for configuring the ESP32Cam-TFMini hybrid sensor.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"

// Sensor orientation for mounting flexibility
typedef enum {
    ESP32CAM_TFMINI_ALIGN_DEFAULT = 0,      // CW0 - Default orientation (forward)
    ESP32CAM_TFMINI_ALIGN_CW90,             // CW90
    ESP32CAM_TFMINI_ALIGN_CW180,            // CW180
    ESP32CAM_TFMINI_ALIGN_CW270,            // CW270
    ESP32CAM_TFMINI_ALIGN_CW0_FLIP,         // CW0 with flip
    ESP32CAM_TFMINI_ALIGN_CW90_FLIP,        // CW90 with flip
    ESP32CAM_TFMINI_ALIGN_CW180_FLIP,       // CW180 with flip
    ESP32CAM_TFMINI_ALIGN_CW270_FLIP,       // CW270 with flip
} esp32camTfminiAlignment_e;

// All distance/altitude values are in CENTIMETERS (consistent with baro/GPS)
//
// AXIS CONVENTION: ESP32Cam follows MAVLink OPTICAL_FLOW_RAD standard where:
//   - flowX: rotation around X (roll) + translation along Y (sideways)
//   - flowY: rotation around Y (pitch) + translation along X (forward)
//
// After gyro compensation, flow maps to body-frame motion:
//   - flowX → left/right motion (with sign inversion)
//   - flowY → forward/back motion
//
typedef struct esp32camTfminiConfig_s {
    uint8_t alignment;                      // Sensor orientation
    uint8_t opticalFlowScale;               // Scale factor for optical flow (0-200%, default 100)
    uint8_t rangefinderScale;               // Scale factor for rangefinder (0-200%, default 100)
    uint8_t minAltitudeCm;                  // Minimum altitude for valid optical flow (CENTIMETERS)
    uint8_t maxAltitudeCm;                  // Maximum altitude for valid optical flow (CENTIMETERS, 0 = use sensor max)
    bool flowInvertX;                       // Invert X axis flow (affects left/right motion after compensation)
    bool flowInvertY;                       // Invert Y axis flow (affects forward/back motion after compensation)
} esp32camTfminiConfig_t;

PG_DECLARE(esp32camTfminiConfig_t, esp32camTfminiConfig);
