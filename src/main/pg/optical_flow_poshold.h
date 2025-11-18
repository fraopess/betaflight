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
 * Configuration for optical flow position hold
 */

#pragma once

#include <stdint.h>
#include "pg/pg.h"

typedef struct opticalFlowPosHoldConfig_s {
    uint8_t max_angle;              // Maximum autopilot angle in degrees (default 10)
    uint8_t pid_p;                  // P gain * 100 (default 50 = 0.5)
    uint8_t pid_i;                  // I gain * 100 (default 10 = 0.1)
    uint8_t pid_d;                  // D gain * 100 (default 5 = 0.05)
    uint8_t pid_i_max;              // I max * 10 (default 100 = 10.0)
    uint8_t stick_deadband;         // Stick deadband for position hold deactivation (default 12)
} opticalFlowPosHoldConfig_t;

PG_DECLARE(opticalFlowPosHoldConfig_t, opticalFlowPosHoldConfig);
