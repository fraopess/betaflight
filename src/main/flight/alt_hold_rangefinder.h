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
 * Rangefinder-based Altitude Hold
 *
 * This module provides rangefinder-specific altitude hold functionality.
 * It is independent from the main altitude hold logic and handles:
 * - Rangefinder altitude validation
 * - Smooth transitions between rangefinder and baro/GPS
 * - Rangefinder-specific altitude calculations
 *
 * This separation allows:
 * - Minimal changes to core altitude hold code
 * - Easy addition of new rangefinder types
 * - Maintainability for upstream Betaflight merges
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Initialize rangefinder altitude hold
void rangefinderAltHoldInit(void);

// Check if rangefinder is valid for altitude hold
bool rangefinderAltHoldIsValid(void);

// Get rangefinder altitude in centimeters
// Returns the altitude from rangefinder, or falls back to baro/GPS if unavailable
float rangefinderAltHoldGetAltitudeCm(void);

// Get rangefinder altitude derivative (velocity) in cm/s
float rangefinderAltHoldGetDerivativeCmS(float dt);
