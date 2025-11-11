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
 * Configuration defaults for ESP32Cam-TFMini sensor
 */

#include "platform.h"

#ifdef USE_RANGEFINDER_ESP32CAM_TFMINI

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/esp32cam_tfmini_config.h"

PG_REGISTER_WITH_RESET_FN(esp32camTfminiConfig_t, esp32camTfminiConfig, PG_ESP32CAM_TFMINI_CONFIG, 0);

// Default configuration values
// NOTE: All altitude/distance values are in CENTIMETERS (consistent with baro/GPS)
void pgResetFn_esp32camTfminiConfig(esp32camTfminiConfig_t *config)
{
    config->alignment = ESP32CAM_TFMINI_ALIGN_DEFAULT;  // No rotation
    config->opticalFlowScale = 100;      // 100% = no scaling
    config->rangefinderScale = 100;      // 100% = no scaling
    config->minAltitudeCm = 10;          // 10cm minimum altitude for valid flow
    config->maxAltitudeCm = 0;           // 0 = use sensor max (1200cm = 12m for TFMini)
    config->flowInvertX = false;         // Don't invert X axis
    config->flowInvertY = false;         // Don't invert Y axis
}

#endif // USE_RANGEFINDER_ESP32CAM_TFMINI