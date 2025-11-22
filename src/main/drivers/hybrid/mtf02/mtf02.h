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
 * MTF-02 Hybrid Sensor Driver
 *
 * The MTF-02 is a hybrid sensor providing:
 * - Rangefinder capability (distance measurement)
 * - Optical flow capability (velocity measurement)
 *
 * Communication: UART at 115200 baud, 8N1
 * Protocol: Binary frames with header, distance, strength, flow data, and checksum
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "drivers/rangefinder/rangefinder.h"

// MTF-02 Protocol Specifications
#define MTF02_FRAME_SYNC_BYTE1      0x59    // First sync byte
#define MTF02_FRAME_SYNC_BYTE2      0x59    // Second sync byte
#define MTF02_FRAME_LENGTH          13      // Payload length (excluding sync bytes and checksum)
#define MTF02_TIMEOUT_MS            200     // Timeout for reconfiguration
#define MTF02_TASK_PERIOD_MS        10      // Update period (100 Hz)
#define MTF02_BAUDRATE              115200  // Serial baud rate
#define MTF02_DETECTION_CONE        300     // 30 degrees (decidegrees)
#define MTF02_MAX_RANGE_CM          1200    // Maximum range in cm

// MTF-02 frame structure (16 bytes total):
// [SYNC1(0x59)][SYNC2(0x59)][DIST_LSB][DIST_MSB][STR_LSB][STR_MSB]
// [FLOW_X_LSB][FLOW_X_MSB][FLOW_Y_LSB][FLOW_Y_MSB][QUALITY][RESERVED1]
// [RESERVED2][RESERVED3][CHECKSUM]

// MTF-02 Data Structure
typedef struct {
    uint16_t distance;      // Distance in cm
    uint16_t strength;      // Signal strength
    int16_t flow_vel_x;     // Optical flow velocity X (cm/s at 1m height)
    int16_t flow_vel_y;     // Optical flow velocity Y (cm/s at 1m height)
    uint8_t quality;        // Flow quality (0-255)
    bool valid;             // Distance data validity
    bool flow_valid;        // Flow data validity
    uint32_t timestamp;     // Last update timestamp (microseconds)
} mtf02_data_t;

// MTF-02 Configuration
typedef struct {
    uint8_t min_quality;        // Minimum quality threshold
    uint16_t min_range_cm;      // Minimum valid range
    uint16_t max_range_cm;      // Maximum valid range
    uint8_t flow_scale;         // Flow scale factor (percentage, default 100)
    bool flow_invert_x;         // Invert X flow
    bool flow_invert_y;         // Invert Y flow
} mtf02_config_t;

// Rangefinder interface functions
bool mtf02Detect(rangefinderDev_t *rangefinder);
void mtf02Init(rangefinderDev_t *rangefinder);
void mtf02Update(rangefinderDev_t *rangefinder);
int32_t mtf02Read(rangefinderDev_t *rangefinder);

// Optical flow data access functions
float mtf02GetFlowRateX(void);
float mtf02GetFlowRateY(void);
uint32_t mtf02GetLastUpdateTime(void);
bool mtf02IsFlowValid(void);
uint8_t mtf02GetFlowQuality(void);

// Optical flow interface functions
struct opticalflowDev_s;
struct opticalflowData_s;
bool mtf02OpticalflowDetect(struct opticalflowDev_s *dev);
void mtf02OpticalflowInit(struct opticalflowDev_s *dev);
void mtf02OpticalflowUpdate(struct opticalflowDev_s *dev);
void mtf02OpticalflowRead(struct opticalflowDev_s *dev, struct opticalflowData_s *data);
