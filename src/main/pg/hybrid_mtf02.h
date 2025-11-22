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
 * MTF-02 Hybrid Sensor Configuration
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"

// Configuration structure for MTF-02 hybrid sensor
typedef struct hybridMtf02Config_s {
    uint16_t min_range_cm;          // Minimum valid range in centimeters (default: 10)
    uint16_t max_range_cm;          // Maximum valid range in centimeters (default: 1200)
    uint8_t min_quality;            // Minimum flow quality threshold (0-255, default: 20)
    uint8_t flow_scale;             // Flow scale factor (percentage, default: 100)
    bool flow_invert_x;             // Invert X flow axis
    bool flow_invert_y;             // Invert Y flow axis
} hybridMtf02Config_t;

PG_DECLARE(hybridMtf02Config_t, hybridMtf02Config);
