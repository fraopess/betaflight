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
 * MTF-02 Hybrid Sensor Driver (PLACEHOLDER)
 *
 * The MTF-02 is another hybrid sensor providing:
 * - Rangefinder capability
 * - Optical flow capability
 *
 * This is a placeholder for future implementation.
 * The MTF-02 uses a different communication protocol than the ESP32.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "drivers/rangefinder/rangefinder.h"

// TODO: Define MTF-02 protocol specifications
// TODO: Define MTF-02 packet structure
// TODO: Define MTF-02 configuration structure

// Rangefinder interface functions (placeholder)
bool mtf02Detect(rangefinderDev_t *rangefinder);
void mtf02Init(rangefinderDev_t *rangefinder);
void mtf02Update(rangefinderDev_t *rangefinder);
int32_t mtf02Read(rangefinderDev_t *rangefinder);

// Optical flow data access functions (placeholder)
float mtf02GetFlowRateX(void);
float mtf02GetFlowRateY(void);
uint32_t mtf02GetLastUpdateTime(void);
bool mtf02IsFlowValid(void);

// Optical flow interface functions (placeholder)
struct opticalflowDev_s;
struct opticalflowData_s;
bool mtf02OpticalflowDetect(struct opticalflowDev_s *dev);
void mtf02OpticalflowInit(struct opticalflowDev_s *dev);
void mtf02OpticalflowUpdate(struct opticalflowDev_s *dev);
void mtf02OpticalflowRead(struct opticalflowDev_s *dev, struct opticalflowData_s *data);
