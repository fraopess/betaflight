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
 * Configuration defaults for ESP32 hybrid sensor
 */

#include "platform.h"

#ifdef USE_HYBRID_ESP32

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "drivers/hybrid/esp32/esp32.h"

PG_REGISTER_WITH_RESET_FN(esp32HybridConfig_t, esp32HybridConfig, PG_ESP32_HYBRID_CONFIG, 0);

// Default configuration values
// NOTE: All altitude/distance values are in CENTIMETERS (consistent with baro/GPS)
void pgResetFn_esp32HybridConfig(esp32HybridConfig_t *config)
{
    config->alignment = ESP32_HYBRID_ALIGN_DEFAULT;  // No rotation
    config->opticalFlowScale = 100;      // 100% = no scaling
    config->rangefinderScale = 100;      // 100% = no scaling
    config->minAltitudeCm = 10;          // 10cm minimum altitude for valid flow
    config->maxAltitudeCm = 0;           // 0 = use sensor max (1200cm = 12m for TFMini)
    config->flowInvertX = false;         // Don't invert X axis
    config->flowInvertY = false;         // Don't invert Y axis
}

#endif // USE_HYBRID_ESP32
