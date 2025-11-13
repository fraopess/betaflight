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
 * Driver for ESP32Cam-TFMini hybrid sensor
 * Combines optical flow from ESP32-CAM and distance from TFMini LiDAR
 * 
 * Protocol: Binary packet format
 * [0xAA][0x55][timestamp(4)][velocity_x(2)][velocity_y(2)][distance(2)][checksum(1)]
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_ESP32CAM_TFMINI)

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_esp32cam_tfmini.h"

#include "io/serial.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/esp32cam_tfmini_config.h"

// Parser state machine
typedef enum {
    STATE_WAIT_HEADER1,
    STATE_WAIT_HEADER2,
    STATE_READ_DATA
} parserState_e;

// Serial port handle
static serialPort_t *esp32camTfminiSerialPort = NULL;

// Sensor state
typedef struct {
    parserState_e state;
    esp32camTfminiPacket_t packet;
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
} esp32camTfminiState_t;

static esp32camTfminiState_t sensorState = {
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
    const esp32camTfminiConfig_t *config = esp32camTfminiConfig();
    
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
        case ESP32CAM_TFMINI_ALIGN_DEFAULT:
        case ESP32CAM_TFMINI_ALIGN_CW0_FLIP:
            // No rotation needed (flip already handled by invert flags)
            break;
            
        case ESP32CAM_TFMINI_ALIGN_CW90:
        case ESP32CAM_TFMINI_ALIGN_CW90_FLIP:
            // 90° clockwise rotation: X' = -Y, Y' = X
            *flowX = -rotatedY;
            *flowY = rotatedX;
            break;
            
        case ESP32CAM_TFMINI_ALIGN_CW180:
        case ESP32CAM_TFMINI_ALIGN_CW180_FLIP:
            // 180° rotation: X' = -X, Y' = -Y
            *flowX = -rotatedX;
            *flowY = -rotatedY;
            break;
            
        case ESP32CAM_TFMINI_ALIGN_CW270:
        case ESP32CAM_TFMINI_ALIGN_CW270_FLIP:
            // 270° clockwise rotation: X' = Y, Y' = -X
            *flowX = rotatedY;
            *flowY = -rotatedX;
            break;
    }
}

// Process received data packet
static void processPacket(esp32camTfminiPacket_t *packet)
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
    const esp32camTfminiConfig_t *config = esp32camTfminiConfig();
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

    // Update DEBUG values - show RAW and TRANSFORMED values at sensor level
    // NOTE: For full processing chain including gyro compensation and body velocities, use DEBUG_OPTICAL_FLOW
    DEBUG_SET(DEBUG_RANGEFINDER_ESP32CAM, 0, sensorState.distance);                     // Altitude in cm
    DEBUG_SET(DEBUG_RANGEFINDER_ESP32CAM, 1, (int32_t)(rawFlowX * 1000));               // RAW flowX before transforms (mrad/s)
    DEBUG_SET(DEBUG_RANGEFINDER_ESP32CAM, 2, (int32_t)(rawFlowY * 1000));               // RAW flowY before transforms (mrad/s)
    DEBUG_SET(DEBUG_RANGEFINDER_ESP32CAM, 3, (int32_t)(sensorState.flowRateX * 1000));  // Transformed flowX after rotation/invert/scale (mrad/s)
    DEBUG_SET(DEBUG_RANGEFINDER_ESP32CAM, 4, (int32_t)(sensorState.flowRateY * 1000));  // Transformed flowY after rotation/invert/scale (mrad/s)
    DEBUG_SET(DEBUG_RANGEFINDER_ESP32CAM, 5, (int32_t)(packet->velocity_x / 1000));     // Sensor velocity_x from hardware (mm/s)
    DEBUG_SET(DEBUG_RANGEFINDER_ESP32CAM, 6, (int32_t)(packet->velocity_y / 1000));     // Sensor velocity_y from hardware (mm/s)
    DEBUG_SET(DEBUG_RANGEFINDER_ESP32CAM, 7, packet->distance);                         // Raw distance from sensor (cm)

}

// Parse incoming serial data
static void esp32camTfminiParse(uint8_t c)
{
    switch (sensorState.state) {
        case STATE_WAIT_HEADER1:
            if (c == ESP32CAM_TFMINI_HEADER1) {
                sensorState.state = STATE_WAIT_HEADER2;
            }
            break;

        case STATE_WAIT_HEADER2:
            if (c == ESP32CAM_TFMINI_HEADER2) {
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
bool esp32camTfminiDetect(rangefinderDev_t *rangefinder)
{
    // Find and open the serial port for ESP32CAM-TFMini
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESP32CAM_TFMINI);
    if (!portConfig) {
        return false;
    }

    esp32camTfminiSerialPort = openSerialPort(portConfig->identifier, FUNCTION_ESP32CAM_TFMINI, NULL, NULL, 115200, MODE_RXTX, 0);
    if (esp32camTfminiSerialPort == NULL) {
        return false;
    }

    // Set detection cone
    rangefinder->delayMs = 20;  // 50Hz update rate = 20ms, use 10ms for scheduler
    rangefinder->maxRangeCm = 1200;  // TFMini max range
    rangefinder->detectionConeDeciDegrees = ESP32CAM_TFMINI_DETECTION_CONE_DECIDEGREES;
    rangefinder->detectionConeExtendedDeciDegrees = ESP32CAM_TFMINI_DETECTION_CONE_EXTENDED_DECIDEGREES;

    rangefinder->init = &esp32camTfminiInit;
    rangefinder->update = &esp32camTfminiUpdate;
    rangefinder->read = &esp32camTfminiRead;

    return true;
}

// Initialization function
void esp32camTfminiInit(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);

    // Reset state
    memset(&sensorState, 0, sizeof(sensorState));
    sensorState.state = STATE_WAIT_HEADER1;
    sensorState.distance = RANGEFINDER_NO_NEW_DATA;
}

// Update function called by scheduler
void esp32camTfminiUpdate(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);
    
    // Process all available serial data
    if (esp32camTfminiSerialPort) {
        while (serialRxBytesWaiting(esp32camTfminiSerialPort) > 0) {
            uint8_t c = serialRead(esp32camTfminiSerialPort);
            esp32camTfminiParse(c);
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
int32_t esp32camTfminiRead(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);
    return sensorState.distance;  // Returns distance in cm
}

// Optical flow data access functions
float esp32camTfminiGetFlowRateX(void)
{
    return sensorState.flowRateX;
}

float esp32camTfminiGetFlowRateY(void)
{
    return sensorState.flowRateY;
}

uint32_t esp32camTfminiGetLastUpdateTime(void)
{
    return sensorState.lastUpdateTime;
}

bool esp32camTfminiIsFlowValid(void)
{
    return sensorState.flowValid && sensorState.distanceValid;
}

#endif // USE_RANGEFINDER && USE_RANGEFINDER_ESP32CAM_TFMINI