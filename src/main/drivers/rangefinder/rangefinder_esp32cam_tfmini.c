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

    // Rangefinder data
    int32_t distance;
    bool distanceValid;

    // Optical flow data
    float flowRateX;        // rad/s
    float flowRateY;        // rad/s
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
    
    // Update rangefinder distance
    sensorState.distance = packet->distance;
    sensorState.distanceValid = (packet->distance > 0 && packet->distance < 1200);
    
    // Update optical flow data
    // Convert from mm/s * 1000 to m/s, then to rad/s
    // velocity_x and velocity_y are in mm/s * 1000
    float velX_m_s = (float)packet->velocity_x / 1000000.0f;  // Convert to m/s
    float velY_m_s = (float)packet->velocity_y / 1000000.0f;  // Convert to m/s
    
    // Convert linear velocity to angular rate using distance
    // flowRate (rad/s) = velocity (m/s) / distance (m)
    if (sensorState.distanceValid && packet->distance > 10) {
        float distance_m = packet->distance / 100.0f;  // Convert cm to m
        sensorState.flowRateX = velX_m_s / distance_m;
        sensorState.flowRateY = velY_m_s / distance_m;
        sensorState.flowValid = true;
    } else {
        sensorState.flowRateX = 0.0f;
        sensorState.flowRateY = 0.0f;
        sensorState.flowValid = false;
    }
    
    // Update DEBUG values for monitoring
    DEBUG_SET(DEBUG_RANGEFINDER, 0, packet->distance);
    DEBUG_SET(DEBUG_RANGEFINDER, 1, packet->velocity_x / 10);  // Scale for visibility
    DEBUG_SET(DEBUG_RANGEFINDER, 2, packet->velocity_y / 10);
    DEBUG_SET(DEBUG_RANGEFINDER, 3, sensorState.goodPackets & 0xFFFF);
}

// Parse incoming serial data
static void esp32camTfminiParse(uint8_t c)
{
    switch (sensorState.state) {
        case STATE_WAIT_HEADER1:
            if (c == ESP32CAM_TFMINI_HEADER1) {
                sensorState.packet.header1 = c;
                sensorState.state = STATE_WAIT_HEADER2;
            }
            break;
            
        case STATE_WAIT_HEADER2:
            if (c == ESP32CAM_TFMINI_HEADER2) {
                sensorState.packet.header2 = c;
                sensorState.dataIndex = 0;
                sensorState.state = STATE_READ_DATA;
            } else {
                sensorState.state = STATE_WAIT_HEADER1;
            }
            break;
            
        case STATE_READ_DATA:
            {
                uint8_t *dataPtr = (uint8_t *)&sensorState.packet.timestamp_ms;
                dataPtr[sensorState.dataIndex++] = c;
                
                // Check if we have received all data + checksum
                const size_t expectedBytes = sizeof(sensorState.packet.timestamp_ms) + 
                                            sizeof(sensorState.packet.velocity_x) + 
                                            sizeof(sensorState.packet.velocity_y) + 
                                            sizeof(sensorState.packet.distance) + 
                                            sizeof(sensorState.packet.checksum);
                
                if (sensorState.dataIndex >= expectedBytes) {
                    processPacket(&sensorState.packet);
                    sensorState.state = STATE_WAIT_HEADER1;
                }
            }
            break;
    }
}

// Detection function
bool esp32camTfminiDetect(rangefinderDev_t *rangefinder)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_ESP32CAM_TFMINI);
    if (!portConfig) {
        return false;
    }

    esp32camTfminiSerialPort = openSerialPort(portConfig->identifier, FUNCTION_ESP32CAM_TFMINI, NULL, NULL, 115200, MODE_RXTX, 0);
    if (esp32camTfminiSerialPort == NULL) {
        return false;
    }

    rangefinder->delayMs = 10;  // 50Hz update rate = 20ms, use 10ms for scheduler
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

// Read function - returns the most recent distance measurement
int32_t esp32camTfminiRead(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);
    return sensorState.distance;
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
