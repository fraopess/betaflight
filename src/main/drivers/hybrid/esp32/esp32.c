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
 * Generic ESP32-based hybrid sensor combining:
 * - Rangefinder from TFMini (or similar LiDAR)
 * - Optical flow from ESP32-CAM (or similar camera)
 *
 * Protocol: Binary packet format
 * [0xAA][0x55][timestamp(4)][velocity_x(2)][velocity_y(2)][distance(2)][checksum(1)]
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_RANGEFINDER) && defined(USE_HYBRID_ESP32)

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/hybrid/esp32/esp32.h"

#include "io/serial.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

// Parser state machine
typedef enum {
    STATE_WAIT_HEADER1,
    STATE_WAIT_HEADER2,
    STATE_READ_DATA
} parserState_e;

// Serial port handle
static serialPort_t *esp32HybridSerialPort = NULL;

// Sensor state
typedef struct {
    parserState_e state;
    esp32HybridPacket_t packet;
    uint8_t dataIndex;
    uint32_t lastUpdateTime;

    // Rangefinder data (units: centimeters)
    int32_t distance;           // Distance in centimeters
    bool distanceValid;

    // Optical flow data
    float flowRateX;        // rad/s (after transformations)
    float flowRateY;        // rad/s (after transformations)
    bool flowValid;

    // Statistics
    uint32_t goodPackets;
    uint32_t badPackets;
} esp32HybridState_t;

static esp32HybridState_t sensorState = {
    .state = STATE_WAIT_HEADER1,
    .dataIndex = 0,
    .lastUpdateTime = 0,
    .distance = RANGEFINDER_NO_NEW_DATA,
    .distanceValid = false,
    .flowRateX = 0.0f,
    .flowRateY = 0.0f,
    .flowValid = false,
    .goodPackets = 0,
    .badPackets = 0
};

// Calculate XOR checksum
static uint8_t calculateChecksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Apply configuration transformations to flow data
// altitude_cm: altitude in centimeters (for validity checks)
//
// NOTE: This function applies sensor-level transformations (scale, invert, rotate).
// The raw sensor data is assumed to follow MAVLink OPTICAL_FLOW_RAD convention.
// High-level axis mapping (flowX→sideways, flowY→forward) happens in optical_flow_poshold.c
//
static void applyFlowTransformations(float *flowX, float *flowY, float altitude_cm)
{
    const esp32HybridConfig_t *config = esp32HybridConfig();

    // Check altitude limits for flow validity
    if (config->minAltitudeCm > 0 && altitude_cm < config->minAltitudeCm) {
        *flowX = 0.0f;
        *flowY = 0.0f;
        return;
    }
    if (config->maxAltitudeCm > 0 && altitude_cm > config->maxAltitudeCm) {
        *flowX = 0.0f;
        *flowY = 0.0f;
        return;
    }

    // Apply scaling factor (stored as percentage 0-200, default 100)
    float scaleFactor = config->opticalFlowScale / 100.0f;
    *flowX *= scaleFactor;
    *flowY *= scaleFactor;

    // Apply axis inversions
    if (config->flowInvertX) {
        *flowX = -*flowX;
    }
    if (config->flowInvertY) {
        *flowY = -*flowY;
    }

    // Apply rotation based on alignment
    float rotatedX = *flowX;
    float rotatedY = *flowY;

    switch (config->alignment) {
        case ESP32_HYBRID_ALIGN_DEFAULT:
        case ESP32_HYBRID_ALIGN_CW0_FLIP:
            // No rotation needed (flip already handled by invert flags)
            break;

        case ESP32_HYBRID_ALIGN_CW90:
        case ESP32_HYBRID_ALIGN_CW90_FLIP:
            // 90° clockwise rotation: X' = -Y, Y' = X
            *flowX = -rotatedY;
            *flowY = rotatedX;
            break;

        case ESP32_HYBRID_ALIGN_CW180:
        case ESP32_HYBRID_ALIGN_CW180_FLIP:
            // 180° rotation: X' = -X, Y' = -Y
            *flowX = -rotatedX;
            *flowY = -rotatedY;
            break;

        case ESP32_HYBRID_ALIGN_CW270:
        case ESP32_HYBRID_ALIGN_CW270_FLIP:
            // 270° clockwise rotation: X' = Y, Y' = -X
            *flowX = rotatedY;
            *flowY = -rotatedX;
            break;
    }
}

// Process received data packet
static void processPacket(esp32HybridPacket_t *packet)
{
    // Verify checksum
    const uint8_t *dataPtr = (const uint8_t *)&packet->timestamp_ms;
    const size_t dataLen = sizeof(packet->timestamp_ms) +
                          sizeof(packet->velocity_x) +
                          sizeof(packet->velocity_y) +
                          sizeof(packet->distance);

    uint8_t calculatedChecksum = calculateChecksum(dataPtr, dataLen);

    if (calculatedChecksum != packet->checksum) {
        sensorState.badPackets++;
        return;
    }

    sensorState.goodPackets++;
    sensorState.lastUpdateTime = millis();

    // Update rangefinder distance with scaling
    // packet->distance is in cm, sensorState.distance is in cm
    const esp32HybridConfig_t *config = esp32HybridConfig();
    float rangeScale = config->rangefinderScale / 100.0f;
    sensorState.distance = (int32_t)(packet->distance * rangeScale);  // Result in cm
    sensorState.distanceValid = (sensorState.distance > 0 && sensorState.distance < 1200);  // Max 1200cm = 12m

    // Update optical flow data
    // Convert from mm/s * 1000 to m/s, then to rad/s
    // velocity_x and velocity_y are in mm/s * 1000
    float velX_m_s = (float)packet->velocity_x / 1000000.0f;  // Convert to m/s
    float velY_m_s = (float)packet->velocity_y / 1000000.0f;  // Convert to m/s

    // Convert linear velocity to angular rate using distance
    // flowRate (rad/s) = velocity (m/s) / distance (m)
    float rawFlowX = 0.0f;
    float rawFlowY = 0.0f;

    if (sensorState.distanceValid && packet->distance > 10) {  // packet->distance is in cm
        float distance_m = packet->distance / 100.0f;  // Convert from cm to m
        rawFlowX = velX_m_s / distance_m;
        rawFlowY = velY_m_s / distance_m;

        // Apply all transformations (rotation, inversion, scaling, altitude limits)
        sensorState.flowRateX = rawFlowX;
        sensorState.flowRateY = rawFlowY;
        applyFlowTransformations(&sensorState.flowRateX, &sensorState.flowRateY,
                                 (float)packet->distance);  // Pass distance in cm

        // Flow is valid if at least one axis is non-zero after transformations
        sensorState.flowValid = (sensorState.flowRateX != 0.0f || sensorState.flowRateY != 0.0f);
    } else {
        sensorState.flowRateX = 0.0f;
        sensorState.flowRateY = 0.0f;
        sensorState.flowValid = false;
    }

    // Note: DEBUG values removed - will be handled separately in refactored code
}

// Parse incoming serial data
static void esp32HybridParse(uint8_t c)
{
    switch (sensorState.state) {
        case STATE_WAIT_HEADER1:
            if (c == ESP32_HYBRID_HEADER1) {
                sensorState.state = STATE_WAIT_HEADER2;
            }
            break;

        case STATE_WAIT_HEADER2:
            if (c == ESP32_HYBRID_HEADER2) {
                sensorState.state = STATE_READ_DATA;
                sensorState.dataIndex = 0;
            } else {
                sensorState.state = STATE_WAIT_HEADER1;
            }
            break;

        case STATE_READ_DATA:
            ((uint8_t*)&sensorState.packet.timestamp_ms)[sensorState.dataIndex++] = c;

            if (sensorState.dataIndex >= (sizeof(sensorState.packet) - 2)) { // -2 for headers
                processPacket(&sensorState.packet);
                sensorState.state = STATE_WAIT_HEADER1;
            }
            break;
    }
}

// Detection interface for rangefinder system
bool esp32HybridDetect(rangefinderDev_t *rangefinder)
{
    // Find and open the serial port for ESP32 hybrid sensor
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESP32CAM_TFMINI);
    if (!portConfig) {
        return false;
    }

    esp32HybridSerialPort = openSerialPort(portConfig->identifier, FUNCTION_ESP32CAM_TFMINI, NULL, NULL, 115200, MODE_RXTX, 0);
    if (esp32HybridSerialPort == NULL) {
        return false;
    }

    // Set detection cone
    rangefinder->delayMs = 20;  // 50Hz update rate = 20ms, use 10ms for scheduler
    rangefinder->maxRangeCm = 1200;  // TFMini max range
    rangefinder->detectionConeDeciDegrees = ESP32_HYBRID_DETECTION_CONE_DECIDEGREES;
    rangefinder->detectionConeExtendedDeciDegrees = ESP32_HYBRID_DETECTION_CONE_EXTENDED_DECIDEGREES;

    rangefinder->init = &esp32HybridInit;
    rangefinder->update = &esp32HybridUpdate;
    rangefinder->read = &esp32HybridRead;

    return true;
}

// Initialization function
void esp32HybridInit(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);

    // Reset state
    memset(&sensorState, 0, sizeof(sensorState));
    sensorState.state = STATE_WAIT_HEADER1;
    sensorState.distance = RANGEFINDER_NO_NEW_DATA;
}

// Update function called by scheduler
void esp32HybridUpdate(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);

    // Process all available serial data
    if (esp32HybridSerialPort) {
        while (serialRxBytesWaiting(esp32HybridSerialPort) > 0) {
            uint8_t c = serialRead(esp32HybridSerialPort);
            esp32HybridParse(c);
        }
    }

    // Check for timeout (no data for 200ms)
    if ((millis() - sensorState.lastUpdateTime) > 200) {
        sensorState.distance = RANGEFINDER_OUT_OF_RANGE;
        sensorState.distanceValid = false;
        sensorState.flowValid = false;
    }
}

// Read function - returns the most recent distance measurement in centimeters
int32_t esp32HybridRead(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);
    return sensorState.distance;  // Returns distance in cm
}

// Optical flow data access functions
float esp32HybridGetFlowRateX(void)
{
    return sensorState.flowRateX;
}

float esp32HybridGetFlowRateY(void)
{
    return sensorState.flowRateY;
}

uint32_t esp32HybridGetLastUpdateTime(void)
{
    return sensorState.lastUpdateTime;
}

bool esp32HybridIsFlowValid(void)
{
    return sensorState.flowValid && sensorState.distanceValid;
}

// ============================================================================
// Optical Flow Interface (for sensors/opticalflow.c integration)
// ============================================================================

#ifdef USE_OPTICALFLOW

#include "drivers/opticalflow/opticalflow.h"

// Optical flow interface: detection
bool esp32OpticalflowDetect(opticalflowDev_t *dev)
{
    // The ESP32 hybrid sensor is detected via rangefinder detection
    // Only proceed if the rangefinder part was already detected
    if (esp32HybridSerialPort == NULL) {
        return false;
    }

    dev->delayMs = 20;  // 50Hz update rate
    dev->minRangeCm = 10;
    dev->minQualityThreshold = 0;  // No quality threshold for optical flow

    dev->init = &esp32OpticalflowInit;
    dev->update = &esp32OpticalflowUpdate;
    dev->read = &esp32OpticalflowRead;

    return true;
}

// Optical flow interface: initialization
void esp32OpticalflowInit(opticalflowDev_t *dev)
{
    UNUSED(dev);
    // Initialization is already done by esp32HybridInit
}

// Optical flow interface: update
void esp32OpticalflowUpdate(opticalflowDev_t *dev)
{
    UNUSED(dev);
    // Update is handled by esp32HybridUpdate via rangefinder task
    // No additional update needed here
}

// Optical flow interface: read
void esp32OpticalflowRead(opticalflowDev_t *dev, opticalflowData_t *data)
{
    UNUSED(dev);

    data->timeStampUs = sensorState.lastUpdateTime * 1000;  // Convert ms to us
    data->quality = sensorState.flowValid ? 100 : 0;  // Simple binary quality

    // Flow rates are already in rad/s
    data->flowRate.x = sensorState.flowRateX;
    data->flowRate.y = sensorState.flowRateY;
}

#endif // USE_OPTICALFLOW

#endif // USE_RANGEFINDER && USE_HYBRID_ESP32
