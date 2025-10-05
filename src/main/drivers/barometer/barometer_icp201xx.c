/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_BARO) && defined(USE_BARO_ICP201XX)

#include "build/build_config.h"
#include "build/debug.h"

#include "barometer.h"
#include "barometer_icp201xx.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

// Static test values for dummy driver
static int32_t dummy_pressure = 101325;    // 1013.25 hPa in Pa
static int32_t dummy_temperature = 2500;   // 25.00°C in centidegrees

// Debug counter for tracking function calls
static uint16_t debug_counter = 0;
static uint16_t calculate_calls = 0;

// Function declarations
static bool icp201xxStartUT(baroDev_t *baro);
static bool icp201xxReadUT(baroDev_t *baro);
static bool icp201xxGetUT(baroDev_t *baro);
static bool icp201xxStartUP(baroDev_t *baro);
static bool icp201xxReadUP(baroDev_t *baro);
static bool icp201xxGetUP(baroDev_t *baro);
static void icp201xxCalculate(int32_t *pressure, int32_t *temperature);

bool icp201xxDetect(baroDev_t *baro)
{
    UNUSED(baro);
    
    // For dummy driver, always succeed detection
    DEBUG_SET(DEBUG_BARO, 4, 0x01); // Signal detection attempt
    
    // Set up barometer interface
    baro->combined_read = false;
    baro->ut_delay = 5000;  // 5ms
    baro->up_delay = 8000;  // 8ms
    
    baro->start_ut = icp201xxStartUT;
    baro->get_ut = icp201xxGetUT;
    baro->read_ut = icp201xxReadUT;
    
    baro->start_up = icp201xxStartUP;
    baro->get_up = icp201xxGetUP;
    baro->read_up = icp201xxReadUP;
    
    baro->calculate = icp201xxCalculate;

    DEBUG_SET(DEBUG_BARO, 5, 0x63); // Signal successful detection
    
    return true;
}

static bool icp201xxStartUT(baroDev_t *baro)
{
    UNUSED(baro);
    debug_counter++;
    DEBUG_SET(DEBUG_BARO, 4, 0xAA); // Signal UT start
    return true;
}

static bool icp201xxReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool icp201xxGetUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool icp201xxStartUP(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool icp201xxReadUP(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool icp201xxGetUP(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static void icp201xxCalculate(int32_t *pressure, int32_t *temperature)
{
    calculate_calls++;
    
    if (pressure) {
        // Keep pressure stable with tiny random-like variation
        int32_t variation = (calculate_calls % 7) - 3; // +/-3 Pa variation
        *pressure = dummy_pressure + variation;
        DEBUG_SET(DEBUG_BARO, 7, (*pressure >> 8) & 0xFFFF);
    }
    
    if (temperature) {
        // Keep temperature stable with tiny variation
        int32_t temp_variation = (calculate_calls % 5) - 2; // +/-0.02°C variation  
        *temperature = dummy_temperature + temp_variation;
        DEBUG_SET(DEBUG_BARO, 6, *temperature & 0xFFFF);
    }
    
    DEBUG_SET(DEBUG_BARO, 5, calculate_calls & 0xFFFF);
}

#endif /* USE_BARO && USE_BARO_ICP201XX */