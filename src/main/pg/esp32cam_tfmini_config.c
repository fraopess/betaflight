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

#include "platform.h"

#ifdef USE_RANGEFINDER_ESP32CAM_TFMINI

#include "esp32cam_tfmini_config.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

// Default configuration values
PG_REGISTER_WITH_RESET_TEMPLATE(esp32camTfminiConfig_t, esp32camTfminiConfig, PG_ESP32CAM_TFMINI_CONFIG, 0);

PG_RESET_TEMPLATE(esp32camTfminiConfig_t, esp32camTfminiConfig,
    .alignment = ESP32CAM_TFMINI_ALIGN_DEFAULT,
    .opticalFlowScale = 100,        // 100% default scale
    .rangefinderScale = 100,        // 100% default scale
    .minAltitudeCm = 10,            // 10cm minimum altitude
    .maxAltitudeCm = 0,             // 0 = use sensor maximum (1200cm)
    .flowInvertX = false,
    .flowInvertY = false,
);

#endif // USE_RANGEFINDER_ESP32CAM_TFMINI
