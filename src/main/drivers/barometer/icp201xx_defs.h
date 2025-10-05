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

#pragma once

#include <stdint.h>

// ICP201XX Constants
#define EXPECTED_DEVICE_ID                      0x63

// ICP201XX I2C Addresses
#define ICP201XX_I2C_ADDR_LOW                   0x63
#define ICP201XX_I2C_ADDR_HIGH                  0x64

// SPI Commands
#define ICP201XX_SPI_READ_CMD                   0x3C
#define ICP201XX_SPI_WRITE_CMD                  0x33

// Error codes
#define INV_ERROR_SUCCESS                       0
#define INV_ERROR                              -1
#define INV_ERROR_TRANSPORT                    -3
#define INV_ERROR_TIMEOUT                      -4
#define INV_ERROR_BAD_ARG                     -11

// Operation modes
typedef enum {
    ICP201XX_OP_MODE0 = 0,  // BW: 6.25Hz, ODR: 25Hz
    ICP201XX_OP_MODE1 = 1,  // BW: 30Hz,   ODR: 120Hz
    ICP201XX_OP_MODE2 = 2,  // BW: 10Hz,   ODR: 40Hz
    ICP201XX_OP_MODE3 = 3,  // BW: 0.5Hz,  ODR: 2Hz
    ICP201XX_OP_MODE4 = 4,  // User configurable
} icp201xx_op_mode_t;

// Measurement modes
typedef enum {
    ICP201XX_MEAS_MODE_FORCED_TRIGGER = 0,
    ICP201XX_MEAS_MODE_CONTINUOUS = 1
} icp201xx_meas_mode_t;

// FIFO readout modes
typedef enum {
    ICP201XX_FIFO_READOUT_MODE_PRES_TEMP = 0,
    ICP201XX_FIFO_READOUT_MODE_TEMP_ONLY = 1,
    ICP201XX_FIFO_READOUT_MODE_TEMP_PRES = 2,
    ICP201XX_FIFO_READOUT_MODE_PRES_ONLY = 3
} icp201xx_FIFO_readout_mode_t;

// Interface modes
typedef enum {
    ICP201XX_IF_I2C = 0,
    ICP201XX_IF_I3C = 1,
    ICP201XX_IF_3_WIRE_SPI = 2,
    ICP201XX_IF_4_WIRE_SPI = 3
} icp201xx_if_t;

// Register addresses
#define MPUREG_MODE_SELECT                      0xC0
#define MPUREG_MASTER_LOCK                      0xBE

// Data conversion macros
#define ICP201XX_PRESSURE_SCALE_FACTOR          40000   // 40kPa
#define ICP201XX_PRESSURE_OFFSET                70000   // 70kPa (in Pa)
#define ICP201XX_PRESSURE_DIVISOR               131072  // 2^17
#define ICP201XX_TEMP_SCALE_FACTOR              65      // 65C
#define ICP201XX_TEMP_OFFSET                    25      // 25C
#define ICP201XX_TEMP_DIVISOR                   262144  // 2^18

// Convert raw pressure to Pa
#define ICP201XX_CONVERT_PRESSURE(raw) \
    (((int32_t)(raw) * ICP201XX_PRESSURE_SCALE_FACTOR) / ICP201XX_PRESSURE_DIVISOR + ICP201XX_PRESSURE_OFFSET)

// Convert raw temperature to 0.01°C
#define ICP201XX_CONVERT_TEMPERATURE(raw) \
    (((int32_t)(raw) * ICP201XX_TEMP_SCALE_FACTOR * 100) / ICP201XX_TEMP_DIVISOR + (ICP201XX_TEMP_OFFSET * 100))
