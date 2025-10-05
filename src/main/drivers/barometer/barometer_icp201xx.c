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
#include "icp201xx_defs.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

// 10 MHz max SPI frequency for ICP201XX
#define ICP201XX_MAX_SPI_CLK_HZ 10000000

// Register definitions from TDK library
#define ICP201XX_DEVICE_ID_REG          0x0C  // MPUREG_DEVICE_ID
#define ICP201XX_FIFO_CONFIG_REG        0xC3  // MPUREG_FIFO_CONFIG
#define ICP201XX_MODE_SELECT_REG        0xC0  // MPUREG_MODE_SELECT
#define ICP201XX_INT_STATUS_REG         0xC1  // MPUREG_INTERRUPT_STATUS
#define ICP201XX_FIFO_FILL_REG          0xC4  // MPUREG_FIFO_FILL
#define ICP201XX_FIFO_DATA_REG          0xFA  // MPUREG_FIFO_BASE
#define ICP201XX_PRESS_ABS_LSB_REG      0xC7  // MPUREG_PRESS_ABS_LSB
#define ICP201XX_TEMP_OUT_LSB_REG       0x09
#define ICP201XX_MASTER_LOCK_REG        0xBE  // MPUREG_MASTER_LOCK
#define ICP201XX_DEVICE_STATUS_REG      0xCD  // MPUREG_DEVICE_STATUS

// Mode selection values
#define ICP201XX_MODE_ACTIVE            0x04
#define ICP201XX_MODE_NORMAL            0x00

// FIFO configuration bits
#define ICP201XX_FIFO_PRES_EN           (1 << 0)
#define ICP201XX_FIFO_TEMP_EN           (1 << 1)
#define ICP201XX_FIFO_FLUSH             (1 << 7)

// Data frame size for combined pressure+temperature reading
#define ICP201XX_DATA_FRAME_SIZE        6

// Calibration data storage
static uint8_t icp201xx_chip_id = 0;
static DMA_DATA_ZERO_INIT uint8_t sensor_data[ICP201XX_DATA_FRAME_SIZE];

// Raw data storage
static int32_t icp201xx_up = 0;  // uncompensated pressure
static int32_t icp201xx_ut = 0;  // uncompensated temperature

// State tracking
static bool warmup_complete = false;
static uint8_t warmup_count = 0;
#define WARMUP_SAMPLES 14  // First 14 samples discarded for FIR filter settling

// Function declarations
static bool icp201xxStartUT(baroDev_t *baro);
static bool icp201xxReadUT(baroDev_t *baro);
static bool icp201xxGetUT(baroDev_t *baro);
static bool icp201xxStartUP(baroDev_t *baro);
static bool icp201xxReadUP(baroDev_t *baro);
static bool icp201xxGetUP(baroDev_t *baro);
static void icp201xxCalculate(int32_t *pressure, int32_t *temperature);

// SPI/I2C helper functions
static bool icp201xxWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t value)
{
    return busWriteRegister(dev, reg, value);
}



static bool icp201xxSoftReset(const extDevice_t *dev)
{
    // Perform soft reset sequence similar to Zephyr driver
    const uint8_t active_mode = ICP201XX_MODE_ACTIVE;
    const uint8_t normal_mode = ICP201XX_MODE_NORMAL;
    const uint8_t unlock = 0x1F;
    const uint8_t lock = 0x00;
    
    delay(4);  // Wait 4ms
    
    // Set to active mode
    if (!icp201xxWriteRegister(dev, ICP201XX_MODE_SELECT_REG, active_mode)) {
        return false;
    }
    
    delay(4);  // Wait 4ms
    
    // Unlock main register write access
    if (!icp201xxWriteRegister(dev, ICP201XX_MASTER_LOCK_REG, unlock)) {
        return false;
    }
    
    delay(4);  // Wait 4ms
    
    // Lock main register write access
    if (!icp201xxWriteRegister(dev, ICP201XX_MASTER_LOCK_REG, lock)) {
        return false;
    }
    
    // Return to normal mode
    if (!icp201xxWriteRegister(dev, ICP201XX_MODE_SELECT_REG, normal_mode)) {
        return false;
    }
    
    delay(4);  // Wait 4ms
    
    return true;
}

static bool icp201xxConfig(const extDevice_t *dev, icp201xx_op_mode_t mode)
{
    // Configure FIFO for pressure and temperature
    uint8_t fifo_config = ICP201XX_FIFO_PRES_EN | ICP201XX_FIFO_TEMP_EN;
    if (!icp201xxWriteRegister(dev, ICP201XX_FIFO_CONFIG_REG, fifo_config)) {
        return false;
    }
    
    // Set operation mode (this sets the ODR and bandwidth)
    uint8_t mode_config = (uint8_t)mode;
    if (!icp201xxWriteRegister(dev, ICP201XX_MODE_SELECT_REG, mode_config)) {
        return false;
    }
    
    return true;
}

static void icp201xxBusInit(const extDevice_t *dev)
{
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin); // Disable
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(ICP201XX_MAX_SPI_CLK_HZ));
    }
}

static void icp201xxBusDeinit(const extDevice_t *dev)
{
    if (dev->bus->busType == BUS_TYPE_SPI) {
        ioPreinitByIO(dev->busType_u.spi.csnPin, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
    }
}

bool icp201xxDetect(baroDev_t *baro)
{
    delay(20);

    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    icp201xxBusInit(dev);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for ICP201XX
        dev->busType_u.i2c.address = ICP201XX_I2C_ADDR_LOW;
        defaultAddressApplied = true;
    }

    // Read chip ID
    if (!busReadRegisterBuffer(dev, ICP201XX_DEVICE_ID_REG, &icp201xx_chip_id, 1)) {
        icp201xxBusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    if (icp201xx_chip_id != EXPECTED_DEVICE_ID) {
        icp201xxBusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    // Perform soft reset
    if (!icp201xxSoftReset(dev)) {
        return false;
    }

    // Configure the sensor for continuous mode
    if (!icp201xxConfig(dev, ICP201XX_OP_MODE2)) { // Use mode 2: 40Hz ODR, 10Hz BW
        return false;
    }

    // Reset warmup state
    warmup_complete = false;
    warmup_count = 0;

    // Set up barometer interface
    baro->combined_read = true;  // We read both pressure and temperature together
    baro->ut_delay = 0;          // Temperature read is dummy (part of combined read)
    baro->up_delay = 25;         // ~25ms for 40Hz ODR mode
    
    baro->start_ut = icp201xxStartUT;
    baro->get_ut = icp201xxGetUT;
    baro->read_ut = icp201xxReadUT;
    
    baro->start_up = icp201xxStartUP;
    baro->get_up = icp201xxGetUP;
    baro->read_up = icp201xxReadUP;
    
    baro->calculate = icp201xxCalculate;

    return true;
}

static bool icp201xxStartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy function - temperature is read as part of combined read
    return true;
}

static bool icp201xxReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy function - temperature is read as part of combined read
    return true;
}

static bool icp201xxGetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy function - temperature is read as part of combined read
    return true;
}

static bool icp201xxStartUP(baroDev_t *baro)
{
    // For ICP201XX, we don't need to start a measurement in continuous mode
    // The sensor automatically provides data in FIFO
    UNUSED(baro);
    return true;
}

static bool icp201xxReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // Check if FIFO has data
    uint8_t fifo_status;
    if (!busReadRegisterBuffer(&baro->dev, ICP201XX_FIFO_FILL_REG, &fifo_status, 1)) {
        return false;
    }

    // Extract FIFO count from status register (bits 0-4)
    uint8_t fifo_count = fifo_status & 0x1F;
    if (fifo_count == 0) {
        return false;
    }

    // Read FIFO data (6 bytes: 3 for pressure, 3 for temperature)
    return busReadRegisterBufferStart(&baro->dev, ICP201XX_FIFO_DATA_REG, sensor_data, ICP201XX_DATA_FRAME_SIZE);
}

static bool icp201xxGetUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // Extract pressure and temperature from the 6-byte FIFO data
    // ICP201XX FIFO format: [P2][P1][P0][T2][T1][T0]
    // Each value is 20-bit, stored in 3 bytes (MSB first)
    
    icp201xx_up = ((uint32_t)sensor_data[0] << 16) | 
                  ((uint32_t)sensor_data[1] << 8) | 
                  sensor_data[2];
    
    icp201xx_ut = ((uint32_t)sensor_data[3] << 16) | 
                  ((uint32_t)sensor_data[4] << 8) | 
                  sensor_data[5];

    // Sign extend 20-bit values to 32-bit
    if (icp201xx_up & 0x80000) {
        icp201xx_up |= 0xFFF00000;
    }
    
    if (icp201xx_ut & 0x80000) {
        icp201xx_ut |= 0xFFF00000;
    }

    // Handle warmup period - discard first 14 samples for FIR filter settling
    if (!warmup_complete) {
        warmup_count++;
        if (warmup_count >= WARMUP_SAMPLES) {
            warmup_complete = true;
            // Flush any remaining FIFO data
            uint8_t fifo_config_flush = ICP201XX_FIFO_PRES_EN | ICP201XX_FIFO_TEMP_EN | ICP201XX_FIFO_FLUSH;
            busWriteRegister(&baro->dev, ICP201XX_FIFO_CONFIG_REG, fifo_config_flush);
        }
        // During warmup, return false so data is not used
        return false;
    }

    return true;
}

static void icp201xxCalculate(int32_t *pressure, int32_t *temperature)
{
    if (pressure) {
        // P = (POUT/2^17)*40kPa + 70kPa
        // Convert to Pa: multiply by 1000
        int64_t press_pa = ((int64_t)icp201xx_up * ICP201XX_PRESSURE_SCALE_FACTOR) / ICP201XX_PRESSURE_DIVISOR;
        press_pa += ICP201XX_PRESSURE_OFFSET;
        *pressure = (int32_t)press_pa;
    }

    if (temperature) {
        // T = (TOUT/2^18)*65C + 25C  
        // Convert to 0.01°C units
        int64_t temp_centidegrees = ((int64_t)icp201xx_ut * ICP201XX_TEMP_SCALE_FACTOR * 100) / ICP201XX_TEMP_DIVISOR;
        temp_centidegrees += (ICP201XX_TEMP_OFFSET * 100);
        *temperature = (int32_t)temp_centidegrees;
    }
}

#endif /* USE_BARO && USE_BARO_ICP201XX */