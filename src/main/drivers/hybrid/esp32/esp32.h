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
 * ESP32 Hybrid Sensor Driver
 *
 * Generic ESP32-based hybrid sensor providing:
 * - Rangefinder capability (from TFMini or similar LiDAR)
 * - Optical flow capability (from ESP32-CAM or similar camera)
 *
 * Communication: Binary protocol over UART
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "drivers/rangefinder/rangefinder.h"
#include "pg/pg.h"

// Protocol definitions for ESP32 hybrid sensor
#define ESP32_HYBRID_HEADER1         0xAA
#define ESP32_HYBRID_HEADER2         0x55
#define ESP32_HYBRID_PACKET_SIZE     13      // Total packet size in bytes

// Packet structure (binary protocol)
// [0xAA][0x55][timestamp(4)][velocity_x(2)][velocity_y(2)][distance(2)][checksum(1)]
// IMPORTANT: All distance/altitude values are in CENTIMETERS
typedef struct __attribute__((packed)) {
    uint8_t header1;                    // 0xAA
    uint8_t header2;                    // 0x55
    uint32_t timestamp_ms;              // Timestamp in milliseconds
    int16_t velocity_x;                 // X velocity in mm/s * 1000
    int16_t velocity_y;                 // Y velocity in mm/s * 1000
    uint16_t distance;                  // Distance in CENTIMETERS (cm)
    uint8_t checksum;                   // XOR checksum
} esp32HybridPacket_t;

// Detection thresholds
#define ESP32_HYBRID_DETECTION_CONE_DECIDEGREES    300  // 30 degrees
#define ESP32_HYBRID_DETECTION_CONE_EXTENDED_DECIDEGREES 450  // 45 degrees

// Sensor orientation for mounting flexibility
typedef enum {
    ESP32_HYBRID_ALIGN_DEFAULT = 0,      // CW0 - Default orientation (forward)
    ESP32_HYBRID_ALIGN_CW90,             // CW90
    ESP32_HYBRID_ALIGN_CW180,            // CW180
    ESP32_HYBRID_ALIGN_CW270,            // CW270
    ESP32_HYBRID_ALIGN_CW0_FLIP,         // CW0 with flip
    ESP32_HYBRID_ALIGN_CW90_FLIP,        // CW90 with flip
    ESP32_HYBRID_ALIGN_CW180_FLIP,       // CW180 with flip
    ESP32_HYBRID_ALIGN_CW270_FLIP,       // CW270 with flip
} esp32HybridAlignment_e;

// Configuration structure
// All distance/altitude values are in CENTIMETERS (consistent with baro/GPS)
typedef struct esp32HybridConfig_s {
    uint8_t alignment;                      // Sensor orientation
    uint8_t opticalFlowScale;               // Scale factor for optical flow (0-200%, default 100)
    uint8_t rangefinderScale;               // Scale factor for rangefinder (0-200%, default 100)
    uint8_t minAltitudeCm;                  // Minimum altitude for valid optical flow (CENTIMETERS)
    uint8_t maxAltitudeCm;                  // Maximum altitude for valid optical flow (CENTIMETERS, 0 = use sensor max)
    bool flowInvertX;                       // Invert X axis flow
    bool flowInvertY;                       // Invert Y axis flow
} esp32HybridConfig_t;

PG_DECLARE(esp32HybridConfig_t, esp32HybridConfig);

// Rangefinder interface functions
bool esp32HybridDetect(rangefinderDev_t *rangefinder);
void esp32HybridInit(rangefinderDev_t *rangefinder);
void esp32HybridUpdate(rangefinderDev_t *rangefinder);
int32_t esp32HybridRead(rangefinderDev_t *rangefinder);

// Optical flow data access functions
float esp32HybridGetFlowRateX(void);
float esp32HybridGetFlowRateY(void);
uint32_t esp32HybridGetLastUpdateTime(void);
bool esp32HybridIsFlowValid(void);

// Optical flow interface functions (for sensors/opticalflow.c integration)
struct opticalflowDev_s;
struct opticalflowData_s;
bool esp32OpticalflowDetect(struct opticalflowDev_s *dev);
void esp32OpticalflowInit(struct opticalflowDev_s *dev);
void esp32OpticalflowUpdate(struct opticalflowDev_s *dev);
void esp32OpticalflowRead(struct opticalflowDev_s *dev, struct opticalflowData_s *data);
