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

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "barometer.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "barometer_icp20100.h"

// 12 MHz max SPI frequency
#define ICP20100_MAX_SPI_CLK_HZ 12000000

#if defined(USE_BARO) && (defined(USE_BARO_ICP20100) || defined(USE_BARO_SPI_ICP20100))

#define ICP20100_I2C_ADDR                    (0x63)
#define ICP20100_EXPECTED_DEVICE_ID          (0x63)

// Register definitions
#define ICP20100_REG_DEVICE_ID               (0x0C)
#define ICP20100_REG_MODE_SELECT             (0xC0)
#define ICP20100_REG_FIFO_FILL               (0xC4)
#define ICP20100_REG_FIFO_BASE               (0xFA)
#define ICP20100_REG_SPI_MODE                (0xC5)
#define ICP20100_REG_OTP_STATUS2             (0xBF)

// Mode select register bits
#define ICP20100_MODE_FORCED_TRIGGER         (0x10)
#define ICP20100_MODE_CONTINUOUS             (0x08)
#define ICP20100_MODE_ACTIVE                 (0x04)
#define ICP20100_FIFO_PRES_TEMP              (0x00)

// SPI mode configuration
#define ICP20100_SPI_MODE_4_WIRE             (0x00)
#define ICP20100_SPI_MODE_3_WIRE             (0x01)



// FIFO status bits
#define ICP20100_FIFO_LEVEL_MASK             (0x1F)

// OTP status
#define ICP20100_OTP_BOOTUP_STATUS           (0x01)

// Measurement delays (in microseconds)
// Mode 0: ODR = 25Hz, so measurement period = 1/25Hz = 40ms
#define ICP20100_MEASUREMENT_DELAY           (40000)  // 40ms

// FIFO data frame size (6 bytes: 3 for pressure, 3 for temperature)
#define ICP20100_DATA_FRAME_SIZE             (6)

// uncompensated pressure and temperature
static int32_t icp20100_up = 0;
static int32_t icp20100_ut = 0;
static uint8_t sensor_data[ICP20100_DATA_FRAME_SIZE];

static bool icp20100StartUT(baroDev_t *baro);
static bool icp20100ReadUT(baroDev_t *baro);
static bool icp20100GetUT(baroDev_t *baro);
static bool icp20100StartUP(baroDev_t *baro);
static bool icp20100ReadUP(baroDev_t *baro);
static bool icp20100GetUP(baroDev_t *baro);
static void icp20100Calculate(int32_t *pressure, int32_t *temperature);

static void icp20100BusInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_ICP20100
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin); // Disable
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(ICP20100_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
#endif
}

static void icp20100BusDeinit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_ICP20100
    if (dev->bus->busType == BUS_TYPE_SPI) {
        ioPreinitByIO(dev->busType_u.spi.csnPin, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
    }
#else
    UNUSED(dev);
#endif
}

bool icp20100Detect(baroDev_t *baro)
{
    delay(20);

    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    icp20100BusInit(dev);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for ICP20100
        dev->busType_u.i2c.address = ICP20100_I2C_ADDR;
        defaultAddressApplied = true;
    }

    uint8_t device_id = 0;
    busReadRegisterBuffer(dev, ICP20100_REG_DEVICE_ID, &device_id, 1);

    if (device_id != ICP20100_EXPECTED_DEVICE_ID) {
        icp20100BusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    // Wait for OTP bootup to complete
    uint8_t otp_status;
    int timeout = 100;
    do {
        delay(1);
        busReadRegisterBuffer(dev, ICP20100_REG_OTP_STATUS2, &otp_status, 1);
        timeout--;
    } while (!(otp_status & ICP20100_OTP_BOOTUP_STATUS) && timeout > 0);

    if (timeout <= 0) {
        icp20100BusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    // Configure SPI mode for 4-wire SPI if using SPI bus
    if (dev->bus->busType == BUS_TYPE_SPI) {
        busWriteRegister(dev, ICP20100_REG_SPI_MODE, ICP20100_SPI_MODE_4_WIRE);
    }

    // Configure sensor for continuous mode with pressure and temperature
    // Mode register: [MEAS_CONFIG:3][0][MEAS_MODE:1][POW_MODE:1][FIFO_MODE:2]
    // MEAS_CONFIG = 0 (mode 0), MEAS_MODE = 1 (continuous), POW_MODE = 1 (active), FIFO_MODE = 0 (pres+temp)
    uint8_t mode_config = ICP20100_MODE_CONTINUOUS | ICP20100_MODE_ACTIVE | ICP20100_FIFO_PRES_TEMP;
    busWriteRegister(dev, ICP20100_REG_MODE_SELECT, mode_config);

    // Set up the barometer device structure
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->start_ut = icp20100StartUT;
    baro->get_ut = icp20100GetUT;
    baro->read_ut = icp20100ReadUT;
    baro->start_up = icp20100StartUP;
    baro->get_up = icp20100GetUP;
    baro->read_up = icp20100ReadUP;
    baro->up_delay = ICP20100_MEASUREMENT_DELAY;
    baro->calculate = icp20100Calculate;

    return true;
}

static bool icp20100StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Temperature is read combined with pressure
    return true;
}

static bool icp20100ReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Temperature is read combined with pressure
    return true;
}

static bool icp20100GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Temperature is read combined with pressure
    return true;
}

static bool icp20100StartUP(baroDev_t *baro)
{
    // In continuous mode, we don't need to trigger each measurement
    // Just return true as the sensor is already running
    UNUSED(baro);
    return true;
}

static bool icp20100ReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // Read FIFO status
    return busReadRegisterBufferStart(&baro->dev, ICP20100_REG_FIFO_FILL, sensor_data, 1);
}

static bool icp20100GetUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // Check FIFO status first
    uint8_t fifo_status = sensor_data[0];

    // Check if FIFO has data
    if ((fifo_status & ICP20100_FIFO_LEVEL_MASK) == 0) {
        return false;  // No data available
    }

    // Read FIFO data now
    if (!busReadRegisterBuffer(&baro->dev, ICP20100_REG_FIFO_BASE, sensor_data, ICP20100_DATA_FRAME_SIZE)) {
        return false;
    }

    // Extract pressure and temperature from FIFO data
    // FIFO format: [P2][P1][P0][T2][T1][T0] (3 bytes pressure, 3 bytes temperature)
    icp20100_up = (int32_t)((sensor_data[0] << 16) | (sensor_data[1] << 8) | sensor_data[2]);
    icp20100_ut = (int32_t)((sensor_data[3] << 16) | (sensor_data[4] << 8) | sensor_data[5]);

    return true;
}

static void icp20100Calculate(int32_t *pressure, int32_t *temperature)
{
    // Convert raw pressure data
    // P = (POUT/2^17)*40kPa + 70kPa
    int32_t press_raw = icp20100_up;

    // Mask to 20 bits and sign extend if negative (bit 19 is set)
    press_raw &= 0x0FFFFF;
    if (press_raw & 0x080000) {
        press_raw |= 0xFFF00000;
    }

    // Convert to Pa: ((press_raw / 131072) * 40000) + 70000
    int32_t press_pa = (int32_t)(((int64_t)press_raw * 40000) / 131072) + 70000;

    // Convert raw temperature data
    // T = (TOUT/2^18)*65C + 25C
    int32_t temp_raw = icp20100_ut;

    // Mask to 20 bits and sign extend if negative (bit 19 is set)
    temp_raw &= 0x0FFFFF;
    if (temp_raw & 0x080000) {
        temp_raw |= 0xFFF00000;
    }

    // Convert to centi-degrees Celsius: ((temp_raw / 262144) * 6500) + 2500
    int32_t temp_cdeg = (int32_t)(((int64_t)temp_raw * 6500) / 262144) + 2500;

    if (pressure) {
        *pressure = press_pa;
    }

    if (temperature) {
        *temperature = temp_cdeg;
    }
}

#endif
