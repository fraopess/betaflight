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
 * Implements serial communication with MTF-02 hybrid sensor
 * providing both rangefinder and optical flow capabilities.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_RANGEFINDER) && defined(USE_HYBRID_MTF02)

#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "drivers/rangefinder/rangefinder.h"
#include "drivers/hybrid/mtf02/mtf02.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/hybrid_mtf02.h"

#include "sensors/rangefinder.h"

// Frame parsing state machine
typedef enum {
    MTF02_FRAME_STATE_WAIT_SYNC1,
    MTF02_FRAME_STATE_WAIT_SYNC2,
    MTF02_FRAME_STATE_READING_PAYLOAD,
    MTF02_FRAME_STATE_WAIT_CKSUM,
} mtf02FrameState_e;

// Static variables
static serialPort_t *mtf02SerialPort = NULL;
static mtf02_data_t mtf02Data = {0};
static uint8_t mtf02Frame[MTF02_FRAME_LENGTH];
static uint8_t mtf02ReceivePosition = 0;
static mtf02FrameState_e mtf02FrameState = MTF02_FRAME_STATE_WAIT_SYNC1;
static timeMs_t mtf02LastFrameReceivedMs = 0;
static int32_t mtf02DistanceValue = RANGEFINDER_NO_NEW_DATA;

// Calculate checksum (simple sum of all payload bytes)
static uint8_t mtf02CalculateChecksum(const uint8_t* data, size_t len)
{
    uint8_t checksum = MTF02_FRAME_SYNC_BYTE1 + MTF02_FRAME_SYNC_BYTE2;
    for (size_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return checksum;
}

// Process received frame
static void mtf02ProcessFrame(const uint8_t* frame)
{
    // Parse frame data (little-endian)
    uint16_t distance = frame[0] | (frame[1] << 8);
    uint16_t strength = frame[2] | (frame[3] << 8);
    int16_t flow_x = (int16_t)(frame[4] | (frame[5] << 8));
    int16_t flow_y = (int16_t)(frame[6] | (frame[7] << 8));
    uint8_t quality = frame[8];
    // frame[9-11] are reserved

    const hybridMtf02Config_t *config = hybridMtf02Config();

    // Validate distance
    bool distanceValid = (distance > 0 &&
                         distance >= config->min_range_cm &&
                         distance <= config->max_range_cm &&
                         strength > 100);  // Minimum signal strength threshold

    // Validate flow
    bool flowValid = (quality >= config->min_quality);

    // Apply flow inversions and scaling
    if (config->flow_invert_x) {
        flow_x = -flow_x;
    }
    if (config->flow_invert_y) {
        flow_y = -flow_y;
    }

    // Apply flow scaling
    flow_x = (flow_x * config->flow_scale) / 100;
    flow_y = (flow_y * config->flow_scale) / 100;

    // Update data structure
    mtf02Data.distance = distance;
    mtf02Data.strength = strength;
    mtf02Data.flow_vel_x = flow_x;
    mtf02Data.flow_vel_y = flow_y;
    mtf02Data.quality = quality;
    mtf02Data.valid = distanceValid;
    mtf02Data.flow_valid = flowValid;
    mtf02Data.timestamp = micros();

    // Update rangefinder value
    if (distanceValid) {
        mtf02DistanceValue = distance;
    } else {
        mtf02DistanceValue = RANGEFINDER_OUT_OF_RANGE;
    }

    mtf02LastFrameReceivedMs = millis();
}

// Send configuration command to MTF-02 (if needed)
static void mtf02SendConfig(void)
{
    if (!mtf02SerialPort) {
        return;
    }

    // MTF-02 configuration command (placeholder - adjust based on actual sensor spec)
    // Format: [0x5A][0x05][0x05][0x01][0x65] - example command to set 100Hz mode
    static const uint8_t configCmd[] = { 0x5A, 0x05, 0x05, 0x01, 0x65 };

    for (size_t i = 0; i < sizeof(configCmd); i++) {
        serialWrite(mtf02SerialPort, configCmd[i]);
    }
}

// ============================================================================
// Rangefinder Interface Implementation
// ============================================================================

bool mtf02Detect(rangefinderDev_t *rangefinder)
{
    // Find serial port configured for MTF-02
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_HYBRID_MTF02);
    if (!portConfig) {
        return false;
    }

    // Open serial port
    mtf02SerialPort = openSerialPort(portConfig->identifier,
                                     FUNCTION_HYBRID_MTF02,
                                     NULL, NULL,
                                     MTF02_BAUDRATE,
                                     MODE_RXTX,
                                     0);

    if (!mtf02SerialPort) {
        return false;
    }

    // Configure rangefinder device
    rangefinder->delayMs = MTF02_TASK_PERIOD_MS;
    rangefinder->maxRangeCm = MTF02_MAX_RANGE_CM;
    rangefinder->detectionConeDeciDegrees = MTF02_DETECTION_CONE;
    rangefinder->detectionConeExtendedDeciDegrees = MTF02_DETECTION_CONE;

    // Set function pointers
    rangefinder->init = &mtf02Init;
    rangefinder->update = &mtf02Update;
    rangefinder->read = &mtf02Read;

    return true;
}

void mtf02Init(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);

    // Reset state machine
    mtf02FrameState = MTF02_FRAME_STATE_WAIT_SYNC1;
    mtf02ReceivePosition = 0;
    mtf02LastFrameReceivedMs = millis();

    // Send configuration command
    mtf02SendConfig();
}

void mtf02Update(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);

    if (!mtf02SerialPort) {
        return;
    }

    // Process incoming serial data with state machine
    while (serialRxBytesWaiting(mtf02SerialPort) > 0) {
        uint8_t c = serialRead(mtf02SerialPort);

        switch (mtf02FrameState) {
            case MTF02_FRAME_STATE_WAIT_SYNC1:
                if (c == MTF02_FRAME_SYNC_BYTE1) {
                    mtf02FrameState = MTF02_FRAME_STATE_WAIT_SYNC2;
                }
                break;

            case MTF02_FRAME_STATE_WAIT_SYNC2:
                if (c == MTF02_FRAME_SYNC_BYTE2) {
                    mtf02FrameState = MTF02_FRAME_STATE_READING_PAYLOAD;
                    mtf02ReceivePosition = 0;
                } else {
                    mtf02FrameState = MTF02_FRAME_STATE_WAIT_SYNC1;
                }
                break;

            case MTF02_FRAME_STATE_READING_PAYLOAD:
                mtf02Frame[mtf02ReceivePosition++] = c;
                if (mtf02ReceivePosition >= MTF02_FRAME_LENGTH) {
                    mtf02FrameState = MTF02_FRAME_STATE_WAIT_CKSUM;
                }
                break;

            case MTF02_FRAME_STATE_WAIT_CKSUM:
                {
                    uint8_t computedChecksum = mtf02CalculateChecksum(mtf02Frame, MTF02_FRAME_LENGTH);
                    if (c == computedChecksum) {
                        // Valid frame - process it
                        mtf02ProcessFrame(mtf02Frame);
                    }
                    // Reset state machine
                    mtf02FrameState = MTF02_FRAME_STATE_WAIT_SYNC1;
                    mtf02ReceivePosition = 0;
                }
                break;
        }
    }

    // Timeout handling - reconfigure if no frames received
    timeMs_t timeNowMs = millis();
    if (cmp32(timeNowMs, mtf02LastFrameReceivedMs) > MTF02_TIMEOUT_MS) {
        mtf02SendConfig();
        mtf02LastFrameReceivedMs = timeNowMs;
    }
}

int32_t mtf02Read(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);
    return mtf02DistanceValue;
}

// ============================================================================
// Optical Flow Data Access Functions
// ============================================================================

float mtf02GetFlowRateX(void)
{
    if (!mtf02Data.flow_valid || !mtf02Data.valid || mtf02Data.distance == 0) {
        return 0.0f;
    }

    // Convert cm/s at 1m to rad/s at actual distance
    // flow_rate (rad/s) = velocity (m/s) / distance (m)
    // velocity (m/s) = flow_vel_x (cm/s @ 1m) * (distance / 100cm) / 100
    float distance_m = mtf02Data.distance / 100.0f;
    float velocity_m_s = (mtf02Data.flow_vel_x / 100.0f) * distance_m;
    float flow_rate_rad_s = velocity_m_s / distance_m;

    return flow_rate_rad_s;
}

float mtf02GetFlowRateY(void)
{
    if (!mtf02Data.flow_valid || !mtf02Data.valid || mtf02Data.distance == 0) {
        return 0.0f;
    }

    // Convert cm/s at 1m to rad/s at actual distance
    float distance_m = mtf02Data.distance / 100.0f;
    float velocity_m_s = (mtf02Data.flow_vel_y / 100.0f) * distance_m;
    float flow_rate_rad_s = velocity_m_s / distance_m;

    return flow_rate_rad_s;
}

uint32_t mtf02GetLastUpdateTime(void)
{
    return mtf02Data.timestamp;
}

bool mtf02IsFlowValid(void)
{
    return mtf02Data.flow_valid && mtf02Data.valid;
}

uint8_t mtf02GetFlowQuality(void)
{
    return mtf02Data.quality;
}

// ============================================================================
// Optical Flow Interface Implementation
// ============================================================================

#ifdef USE_OPTICALFLOW

#include "drivers/opticalflow/opticalflow.h"
#include "sensors/opticalflow.h"

bool mtf02OpticalflowDetect(opticalflowDev_t *dev)
{
    // Optical flow is available if rangefinder is detected
    // (they share the same serial port)
    if (!mtf02SerialPort) {
        return false;
    }

    dev->delayMs = MTF02_TASK_PERIOD_MS;
    dev->minRangeCm = hybridMtf02Config()->min_range_cm;
    dev->minQualityThreshold = hybridMtf02Config()->min_quality;

    dev->init = &mtf02OpticalflowInit;
    dev->update = &mtf02OpticalflowUpdate;
    dev->read = &mtf02OpticalflowRead;

    return true;
}

void mtf02OpticalflowInit(opticalflowDev_t *dev)
{
    UNUSED(dev);
    // Initialization already done in mtf02Init
}

void mtf02OpticalflowUpdate(opticalflowDev_t *dev)
{
    UNUSED(dev);
    // Update already done in mtf02Update (shared serial port)
}

void mtf02OpticalflowRead(opticalflowDev_t *dev, opticalflowData_t *data)
{
    UNUSED(dev);

    if (!mtf02IsFlowValid()) {
        data->quality = 0;
        return;
    }

    data->timeStampUs = mtf02Data.timestamp;
    data->quality = mtf02Data.quality;
    data->flowRate.x = mtf02GetFlowRateX();
    data->flowRate.y = mtf02GetFlowRateY();
}

#endif // USE_OPTICALFLOW

#endif // USE_RANGEFINDER && USE_HYBRID_MTF02
