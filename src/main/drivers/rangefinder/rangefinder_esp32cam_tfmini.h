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

#include "drivers/rangefinder/rangefinder.h"

// Protocol definitions for ESP32Cam-TFMini hybrid sensor
#define ESP32CAM_TFMINI_HEADER1         0xAA
#define ESP32CAM_TFMINI_HEADER2         0x55
#define ESP32CAM_TFMINI_PACKET_SIZE     13      // Total packet size in bytes

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
} esp32camTfminiPacket_t;

// Detection thresholds
#define ESP32CAM_TFMINI_DETECTION_CONE_DECIDEGREES    300  // 30 degrees
#define ESP32CAM_TFMINI_DETECTION_CONE_EXTENDED_DECIDEGREES 450  // 45 degrees

bool esp32camTfminiDetect(rangefinderDev_t *rangefinder);
void esp32camTfminiInit(rangefinderDev_t *rangefinder);
void esp32camTfminiUpdate(rangefinderDev_t *rangefinder);
int32_t esp32camTfminiRead(rangefinderDev_t *rangefinder);

// Optical flow data access functions
float esp32camTfminiGetFlowRateX(void);
float esp32camTfminiGetFlowRateY(void);
uint32_t esp32camTfminiGetLastUpdateTime(void);
bool esp32camTfminiIsFlowValid(void);
