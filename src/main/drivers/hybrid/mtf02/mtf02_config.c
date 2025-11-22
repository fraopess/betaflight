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
 * Configuration defaults for MTF-02 hybrid sensor
 */

#include "platform.h"

#ifdef USE_HYBRID_MTF02

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/hybrid_mtf02.h"

PG_REGISTER_WITH_RESET_FN(hybridMtf02Config_t, hybridMtf02Config, PG_MTF02_HYBRID_CONFIG, 0);

// Default configuration values
void pgResetFn_hybridMtf02Config(hybridMtf02Config_t *config)
{
    config->min_range_cm = 10;       // 10cm minimum range
    config->max_range_cm = 1200;     // 1200cm (12m) maximum range
    config->min_quality = 20;        // Minimum flow quality of 20
    config->flow_scale = 100;        // 100% = no scaling
    config->flow_invert_x = false;   // Don't invert X axis
    config->flow_invert_y = false;   // Don't invert Y axis
}

#endif // USE_HYBRID_MTF02
