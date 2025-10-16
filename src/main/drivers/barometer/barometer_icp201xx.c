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
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "barometer.h"
#include "barometer_icp201xx.h"

#include "sensors/barometer.h"

#if defined(USE_BARO) && (defined(USE_BARO_ICP201XX) || defined(USE_BARO_SPI_ICP201XX))

// Device constants
#define ICP201XX_I2C_ADDR               0x63
#define ICP201XX_DEVICE_ID              0x63
#define ICP201XX_MAX_SPI_CLK_HZ         10000000
#define ICP201XX_MAX_TRANSFER_SIZE      2048
#define ICP201XX_CONVERSION_INTERVAL    25000

// SPI command bytes
#define ICP201XX_SPI_READ_CMD           0x3C
#define ICP201XX_SPI_WRITE_CMD          0x33

// Debug states
#define ICP201XX_DEBUG_DETECT_START     1
#define ICP201XX_DEBUG_ID_READ          2
#define ICP201XX_DEBUG_ID_VALID         3
#define ICP201XX_DEBUG_VERSION_READ     4
#define ICP201XX_DEBUG_RESET_DONE       5
#define ICP201XX_DEBUG_BOOT_START       6
#define ICP201XX_DEBUG_BOOT_DONE        7
#define ICP201XX_DEBUG_CONFIG_START     8
#define ICP201XX_DEBUG_CONFIG_DONE      9
#define ICP201XX_DEBUG_WAIT_START       10
#define ICP201XX_DEBUG_WAIT_DONE        11
#define ICP201XX_DEBUG_SUCCESS          12
#define ICP201XX_DEBUG_FAIL_ID          20
#define ICP201XX_DEBUG_FAIL_VER         21
#define ICP201XX_DEBUG_FAIL_BOOT        22
#define ICP201XX_DEBUG_FAIL_CONFIG      23

// Register definitions
#define REG_EMPTY                       0x00
#define REG_TRIM1_MSB                   0x05
#define REG_TRIM2_LSB                   0x06
#define REG_TRIM2_MSB                   0x07
#define REG_DEVICE_ID                   0x0C
#define REG_OTP_MTP_OTP_CFG1            0xAC
#define REG_OTP_MTP_MR_LSB              0xAD
#define REG_OTP_MTP_MR_MSB              0xAE
#define REG_OTP_MTP_MRA_LSB             0xAF
#define REG_OTP_MTP_MRA_MSB             0xB0
#define REG_OTP_MTP_MRB_LSB             0xB1
#define REG_OTP_MTP_MRB_MSB             0xB2
#define REG_OTP_MTP_OTP_ADDR            0xB5
#define REG_OTP_MTP_OTP_CMD             0xB6
#define REG_OTP_MTP_RD_DATA             0xB8
#define REG_OTP_MTP_OTP_STATUS          0xB9
#define REG_OTP_DEBUG2                  0xBC
#define REG_MASTER_LOCK                 0xBE
#define REG_OTP_MTP_OTP_STATUS2         0xBF
#define REG_MODE_SELECT                 0xC0
#define REG_INTERRUPT_STATUS            0xC1
#define REG_INTERRUPT_MASK              0xC2
#define REG_FIFO_CONFIG                 0xC3
#define REG_FIFO_FILL                   0xC4
#define REG_SPI_MODE                    0xC5
#define REG_PRESS_ABS_LSB               0xC7
#define REG_PRESS_ABS_MSB               0xC8
#define REG_PRESS_DELTA_LSB             0xC9
#define REG_PRESS_DELTA_MSB             0xCA
#define REG_DEVICE_STATUS               0xCD
#define REG_I3C_INFO                    0xCE
#define REG_VERSION                     0xD3
#define REG_FIFO_BASE                   0xFA

// Operating modes
typedef enum {
    OP_MODE0 = 0,   // Mode 0: Bw:6.25 Hz ODR: 25Hz
    OP_MODE1,       // Mode 1: Bw:30 Hz ODR: 120Hz
    OP_MODE2,       // Mode 2: Bw:10 Hz ODR: 40Hz
    OP_MODE3,       // Mode 3: Bw:0.5 Hz ODR: 2Hz
    OP_MODE4,       // Mode 4: User configurable Mode
} icp201xx_op_mode_t;

// FIFO readout modes
typedef enum {
    FIFO_READOUT_MODE_PRES_TEMP = 0,   // Pressure and temperature as pair (pressure first)
    FIFO_READOUT_MODE_TEMP_ONLY = 1,   // Temperature only reporting
    FIFO_READOUT_MODE_TEMP_PRES = 2,   // Pressure and temperature as pair (temperature first)
    FIFO_READOUT_MODE_PRES_ONLY = 3    // Pressure only reporting
} icp201xx_fifo_readout_mode_t;

// Power modes
typedef enum {
    POWER_MODE_NORMAL = 0,  // Normal Mode: Device is in standby and goes to active mode during measurement
    POWER_MODE_ACTIVE = 1   // Active Mode: Power on DVDD and enable the high frequency clock
} icp201xx_power_mode_t;

// Measurement modes
typedef enum {
    MEAS_MODE_FORCED_TRIGGER = 0, // Force trigger mode
    MEAS_MODE_CONTINUOUS = 1      // Continuous measurements based on selected mode ODR settings
} icp201xx_meas_mode_t;

// Forced measurement trigger
typedef enum {
    FORCE_MEAS_STANDBY = 0,           // Stay in Stand by
    FORCE_MEAS_TRIGGER_FORCE_MEAS = 1 // Trigger for forced measurements
} icp201xx_forced_meas_trigger_t;

// Static configuration
static const icp201xx_op_mode_t op_mode = OP_MODE2;
static const icp201xx_fifo_readout_mode_t fifo_readout_mode = FIFO_READOUT_MODE_PRES_TEMP;
static const icp201xx_power_mode_t power_mode = POWER_MODE_NORMAL;
static const icp201xx_meas_mode_t meas_mode = MEAS_MODE_CONTINUOUS;
static const icp201xx_forced_meas_trigger_t forced_meas_trigger = FORCE_MEAS_STANDBY;

// State variables
static float pressure_pa = 0.0f;
static float temperature_c = 0.0f;
static uint32_t last_measure_us = 0;

// Core transport functions that handle bus type internally - device, reg to read, buffer to log reply to, length of buffer
static bool icp201xx_transfer_read(const extDevice_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (dev->bus->busType == BUS_TYPE_SPI) {
        // ICP201XX SPI protocol: command (0x3C) + register + dummy bytes for clocking data out
        // All in one transaction - let spiReadWriteBuf handle CS automatically

        uint8_t txBuf[len + 2];
        uint8_t rxBuf[len + 2];

        // Prepare TX buffer
        txBuf[0] = ICP201XX_SPI_READ_CMD;
        txBuf[1] = reg;
        for (int i = 2; i < len + 2; i++) {
            txBuf[i] = 0x00; // Dummy bytes for clocking out data
        }

        // Single transaction with automatic CS handling
        spiReadWriteBuf(dev, txBuf, rxBuf, len + 2);

        // Copy received data (skip command and address echo)
        for (int i = 0; i < len; i++) {
            buf[i] = rxBuf[i + 2];
        }

        return true;
    } else {
        // I2C: standard register read followed by dummy read
        bool ret = busReadRegisterBuffer(dev, reg, buf, len);
        if (ret) {
            // Perform dummy register read as per ICP201XX I2C protocol
            uint8_t dummy;
            busReadRegisterBuffer(dev, REG_EMPTY, &dummy, 1);
        }
        return ret;
    }
}

static bool icp201xx_transfer_write(const extDevice_t *dev, uint8_t reg, uint8_t val)
{
    if (dev->bus->busType == BUS_TYPE_SPI) {
        // ICP201XX SPI protocol: command (0x33) + register + data byte
        // All in one transaction - let spiReadWriteBuf handle CS automatically

        uint8_t txBuf[3];
        uint8_t rxBuf[3]; // Not used but needed for spiReadWriteBuf

        txBuf[0] = ICP201XX_SPI_WRITE_CMD;
        txBuf[1] = reg;
        txBuf[2] = val;

        // Single transaction with automatic CS handling
        spiReadWriteBuf(dev, txBuf, rxBuf, 3);

        return true;
    } else {
        // I2C: standard register write followed by dummy read
        bool ret = busWriteRegister(dev, reg, val);
        if (ret) {
            // Perform dummy register read as per ICP201XX I2C protocol
            uint8_t dummy;
            busReadRegisterBuffer(dev, REG_EMPTY, &dummy, 1);
        }
        return ret;
    }
}

// High-level sensor functions
static bool icp201xx_mode_select(const extDevice_t *dev, uint8_t mode)
{
    uint8_t mode_sync_status = 0;
    uint32_t timeout = 1000; // 1000ms timeout

    do {
        if (!icp201xx_transfer_read(dev, REG_DEVICE_STATUS, &mode_sync_status, 1)) {
            DEBUG_SET(DEBUG_BARO, 0, 50); // Failed to read device status
            return false;
        }

        if (mode_sync_status & 0x01) {
            break;
        }

        delay(1);
        timeout--;
        if (timeout == 0) {
            DEBUG_SET(DEBUG_BARO, 0, 51); // Mode select timeout
            DEBUG_SET(DEBUG_BARO, 1, mode_sync_status); // Show last status
            return false;
        }
    } while (1);

    bool result = icp201xx_transfer_write(dev, REG_MODE_SELECT, mode);
    DEBUG_SET(DEBUG_BARO, 0, result ? 52 : 53); // Mode select result
    DEBUG_SET(DEBUG_BARO, 1, mode); // Show mode written
    return result;
}

static bool icp201xx_read_otp_data(const extDevice_t *dev, uint8_t addr, uint8_t cmd, uint8_t *val)
{
    uint8_t otp_status = 0xFF;
    uint32_t timeout = 10000; // 10ms timeout

    // Write the address content and read command
    if (!icp201xx_transfer_write(dev, REG_OTP_MTP_OTP_ADDR, addr)) {
        return false;
    }

    if (!icp201xx_transfer_write(dev, REG_OTP_MTP_OTP_CMD, cmd)) {
        return false;
    }

    // Wait for the OTP read to finish - Monitor otp_status
    do {
        if (!icp201xx_transfer_read(dev, REG_OTP_MTP_OTP_STATUS, &otp_status, 1)) {
            return false;
        }

        if (otp_status == 0) {
            break;
        }

        delayMicroseconds(1);
        timeout--;
        if (timeout == 0) {
            return false;
        }
    } while (1);

    // Read the data from register
    return icp201xx_transfer_read(dev, REG_OTP_MTP_RD_DATA, val, 1);
}

// Forward declaration
static bool icp201xx_flush_fifo(const extDevice_t *dev);

static bool icp201xx_get_sensor_data(const extDevice_t *dev, float *pressure, float *temperature)
{
    uint8_t fifo_data[96] = {0};
    uint8_t fifo_packets = 0;
    int32_t data_temp = 0;
    int32_t data_press = 0;
    *pressure = 0;
    *temperature = 0;

    if (!icp201xx_transfer_read(dev, REG_FIFO_FILL, &fifo_packets, 1)) {
        DEBUG_SET(DEBUG_BARO, 0, 40); // Failed to read FIFO fill
        return false;
    }

    fifo_packets = (uint8_t)(fifo_packets & 0x1F);
    DEBUG_SET(DEBUG_BARO, 1, fifo_packets); // Show FIFO packet count

    if (fifo_packets > 16) {
        DEBUG_SET(DEBUG_BARO, 0, 41); // FIFO overflow
        icp201xx_flush_fifo(dev);
        return false;
    }

    if (fifo_packets > 0 && fifo_packets <= 16) {
        uint8_t fifo_read_len = fifo_packets * 2 * 3;
        // fifo_read_len max is 16 * 2 * 3 = 96 bytes, well within transfer limits
        if (!icp201xx_transfer_read(dev, REG_FIFO_BASE, fifo_data, fifo_read_len)) {
            DEBUG_SET(DEBUG_BARO, 0, 42); // Failed to read FIFO data
            return false;
        }

        uint8_t offset = 0;

        for (uint8_t i = 0; i < fifo_packets; i++) {
            // Extract pressure data (20-bit two's complement)
            data_press = (int32_t)(((fifo_data[offset + 2] & 0x0f) << 16) |
                                  (fifo_data[offset + 1] << 8) |
                                  fifo_data[offset]);
            if (data_press & 0x080000) {
                data_press |= 0xFFF00000;
            }
            // P = (POUT/2^17)*40kPa + 70kPa
            *pressure += ((float)(data_press) * 40.0f / 131072.0f) + 70.0f;
            offset += 3;

            // Extract temperature data (20-bit two's complement)
            data_temp = (int32_t)(((fifo_data[offset + 2] & 0x0f) << 16) |
                                 (fifo_data[offset + 1] << 8) |
                                 fifo_data[offset]);
            if (data_temp & 0x080000) {
                data_temp |= 0xFFF00000;
            }
            // T = (TOUT/2^18)*65C + 25C
            *temperature += ((float)(data_temp) * 65.0f / 262144.0f) + 25.0f;
            offset += 3;
        }

        *pressure = *pressure * 1000.0f / fifo_packets; // Convert to Pa
        *temperature = *temperature / fifo_packets;
        DEBUG_SET(DEBUG_BARO, 0, 43); // Successful sensor data read
        return true;
    }

    DEBUG_SET(DEBUG_BARO, 0, 44); // No packets available
    return false;
}

static bool icp201xx_flush_fifo(const extDevice_t *dev)
{
    uint8_t reg_value;

    if (!icp201xx_transfer_read(dev, REG_FIFO_FILL, &reg_value, 1)) {
        return false;
    }

    reg_value |= 0x80;

    if (!icp201xx_transfer_write(dev, REG_FIFO_FILL, reg_value)) {
        return false;
    }

    return true;
}

static void icp201xx_soft_reset(const extDevice_t *dev)
{
    // Stop the measurement
    icp201xx_mode_select(dev, 0x00);
    delay(2);

    // Flush FIFO
    icp201xx_flush_fifo(dev);

    // Mask all interrupts
    icp201xx_transfer_write(dev, REG_FIFO_CONFIG, 0x00);
    icp201xx_transfer_write(dev, REG_INTERRUPT_MASK, 0xFF);
}

static bool icp201xx_boot_sequence(const extDevice_t *dev)
{
    uint8_t reg_value = 0;
    uint8_t offset = 0, gain = 0, Hfosc = 0;
    uint8_t version = 0;
    uint8_t bootup_status = 0;
    bool ret = true;

    // Read version register
    if (!icp201xx_transfer_read(dev, REG_VERSION, &version, 1)) {
        DEBUG_SET(DEBUG_BARO, 2, 100); // Boot fail - version read
        return false;
    }

    DEBUG_SET(DEBUG_BARO, 2, version + 50); // Show version in boot sequence

    if (version == 0xB2) {
        // B2 version Asic is detected. Boot up sequence is not required for B2 Asic
        DEBUG_SET(DEBUG_BARO, 2, 200); // B2 version detected
        return true;
    }

    // Read boot up status and avoid re running boot up sequence if it is already done
    if (!icp201xx_transfer_read(dev, REG_OTP_MTP_OTP_STATUS2, &bootup_status, 1)) {
        DEBUG_SET(DEBUG_BARO, 2, 101); // Boot fail - status read
        return false;
    }

    DEBUG_SET(DEBUG_BARO, 2, bootup_status + 60); // Show boot status

    if (bootup_status & 0x01) {
        // Boot up sequence is already done, not required to repeat boot up sequence
        DEBUG_SET(DEBUG_BARO, 2, 201); // Boot already done
        return true;
    }

    // Bring the ASIC in power mode to activate the OTP power domain and get access to the main registers
    DEBUG_SET(DEBUG_BARO, 2, 202); // Starting A0 boot sequence
    icp201xx_mode_select(dev, 0x04);
    delay(4);

    // Unlock the main registers
    icp201xx_transfer_write(dev, REG_MASTER_LOCK, 0x1F);

    // Enable the OTP and the write switch
    if (!icp201xx_transfer_read(dev, REG_OTP_MTP_OTP_CFG1, &reg_value, 1)) {
        return false;
    }
    reg_value |= 0x03;
    icp201xx_transfer_write(dev, REG_OTP_MTP_OTP_CFG1, reg_value);
    delayMicroseconds(10);

    // Toggle the OTP reset pin
    if (!icp201xx_transfer_read(dev, REG_OTP_DEBUG2, &reg_value, 1)) {
        return false;
    }
    reg_value |= 1 << 7;
    icp201xx_transfer_write(dev, REG_OTP_DEBUG2, reg_value);
    delayMicroseconds(10);

    if (!icp201xx_transfer_read(dev, REG_OTP_DEBUG2, &reg_value, 1)) {
        return false;
    }
    reg_value &= ~(1 << 7);
    icp201xx_transfer_write(dev, REG_OTP_DEBUG2, reg_value);
    delayMicroseconds(10);

    // Program redundant read
    icp201xx_transfer_write(dev, REG_OTP_MTP_MRA_LSB, 0x04);
    icp201xx_transfer_write(dev, REG_OTP_MTP_MRA_MSB, 0x04);
    icp201xx_transfer_write(dev, REG_OTP_MTP_MRB_LSB, 0x21);
    icp201xx_transfer_write(dev, REG_OTP_MTP_MRB_MSB, 0x20);
    icp201xx_transfer_write(dev, REG_OTP_MTP_MR_LSB, 0x10);
    icp201xx_transfer_write(dev, REG_OTP_MTP_MR_MSB, 0x80);

    // Read the data from register
    DEBUG_SET(DEBUG_BARO, 2, 203); // Reading OTP data
    ret &= icp201xx_read_otp_data(dev, 0xF8, 0x10, &offset);
    ret &= icp201xx_read_otp_data(dev, 0xF9, 0x10, &gain);
    ret &= icp201xx_read_otp_data(dev, 0xFA, 0x10, &Hfosc);
    delayMicroseconds(10);

    DEBUG_SET(DEBUG_BARO, 3, offset); // Show OTP values
    if (!ret) {
        DEBUG_SET(DEBUG_BARO, 2, 102); // OTP read failed
        return false;
    }

    // Write OTP values to main registers
    if (ret && icp201xx_transfer_read(dev, REG_TRIM1_MSB, &reg_value, 1)) {
        reg_value = (reg_value & (~0x3F)) | (offset & 0x3F);
        ret &= icp201xx_transfer_write(dev, REG_TRIM1_MSB, reg_value);
    }

    if (ret && icp201xx_transfer_read(dev, REG_TRIM2_MSB, &reg_value, 1)) {
        reg_value = (reg_value & (~0x70)) | ((gain & 0x07) << 4);
        ret &= icp201xx_transfer_write(dev, REG_TRIM2_MSB, reg_value);
    }

    if (ret && icp201xx_transfer_read(dev, REG_TRIM2_LSB, &reg_value, 1)) {
        reg_value = (reg_value & (~0x7F)) | (Hfosc & 0x7F);
        ret &= icp201xx_transfer_write(dev, REG_TRIM2_LSB, reg_value);
    }

    delayMicroseconds(10);

    // Update boot up status to 1
    if (ret && icp201xx_transfer_read(dev, REG_OTP_MTP_OTP_STATUS2, &reg_value, 1)) {
        reg_value |= 0x01;
        ret &= icp201xx_transfer_write(dev, REG_OTP_MTP_OTP_STATUS2, reg_value);
    }

    // Disable OTP and write switch
    if (icp201xx_transfer_read(dev, REG_OTP_MTP_OTP_CFG1, &reg_value, 1)) {
        reg_value &= ~0x03;
        icp201xx_transfer_write(dev, REG_OTP_MTP_OTP_CFG1, reg_value);
    }

    // Lock the main register
    icp201xx_transfer_write(dev, REG_MASTER_LOCK, 0x00);

    // Move to standby
    icp201xx_mode_select(dev, 0x00);

    DEBUG_SET(DEBUG_BARO, 2, ret ? 204 : 103); // Boot sequence result
    return ret;
}

static bool icp201xx_configure(const extDevice_t *dev)
{
    uint8_t reg_value = 0;

    // Initiate Triggered Operation: Stay in Standby mode
    reg_value |= (reg_value & (~0x10)) | ((uint8_t)forced_meas_trigger << 4);

    // Power Mode Selection: Normal Mode
    reg_value |= (reg_value & (~0x04)) | ((uint8_t)power_mode << 2);

    // FIFO Readout Mode Selection: Pressure first
    reg_value |= (reg_value & (~0x03)) | ((uint8_t)(fifo_readout_mode));

    // Measurement Configuration: Mode2
    reg_value |= (reg_value & (~0xE0)) | (((uint8_t)op_mode) << 5);

    // Measurement Mode Selection: Continuous Measurements (duty cycled)
    reg_value |= (reg_value & (~0x08)) | ((uint8_t)meas_mode << 3);

    return icp201xx_mode_select(dev, reg_value);
}

static void icp201xx_wait_read(const extDevice_t *dev)
{
    /*
    * If FIR filter is enabled, it will cause a settling effect on the first 14 pressure values.
    * Therefore the first 14 pressure output values are discarded.
    */
    uint8_t fifo_packets = 0;
    uint8_t fifo_packets_to_skip = 14;

    do {
        delay(10);
        icp201xx_transfer_read(dev, REG_FIFO_FILL, &fifo_packets, 1);
        fifo_packets = (uint8_t)(fifo_packets & 0x1F);
    } while (fifo_packets < fifo_packets_to_skip);

    icp201xx_flush_fifo(dev);
    fifo_packets = 0;

    do {
        delay(10);
        icp201xx_transfer_read(dev, REG_FIFO_FILL, &fifo_packets, 1);
        fifo_packets = (uint8_t)(fifo_packets & 0x1F);
    } while (fifo_packets == 0);
}

// Betaflight barometer interface functions
static bool icp201xx_start_ut(baroDev_t *baro)
{
    UNUSED(baro);
    // Temperature measurement is combined with pressure
    return true;
}

static bool icp201xx_read_ut(baroDev_t *baro)
{
    UNUSED(baro);
    // Temperature measurement is combined with pressure
    return true;
}

static bool icp201xx_get_ut(baroDev_t *baro)
{
    UNUSED(baro);
    // Temperature measurement is combined with pressure
    return true;
}

static bool icp201xx_start_up(baroDev_t *baro)
{
    UNUSED(baro);
    // Continuous mode - no need to start
    return true;
}

static bool icp201xx_read_up(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        DEBUG_SET(DEBUG_BARO, 0, 30); // Bus busy
        return false;
    }

    // Read sensor data
    float pressure, temperature;
    bool result = icp201xx_get_sensor_data(&baro->dev, &pressure, &temperature);

    if (result) {
        pressure_pa = pressure;
        temperature_c = temperature;
        last_measure_us = micros();
        DEBUG_SET(DEBUG_BARO, 0, 31); // Successful read
        DEBUG_SET(DEBUG_BARO, 1, (int32_t)(pressure / 100)); // Pressure in hPa
    } else {
        // Check for timeout and flush if needed
        if (micros() - last_measure_us > ICP201XX_CONVERSION_INTERVAL * 3) {
            icp201xx_flush_fifo(&baro->dev);
            last_measure_us = micros();
            DEBUG_SET(DEBUG_BARO, 0, 32); // Timeout, flushed FIFO
        } else {
            DEBUG_SET(DEBUG_BARO, 0, 33); // Read failed, no timeout
        }
    }

    return result;
}

static bool icp201xx_get_up(baroDev_t *baro)
{
    UNUSED(baro);
    // Data already read in read_up
    return true;
}

static void icp201xx_calculate(int32_t *pressure, int32_t *temperature)
{
    if (pressure) {
        *pressure = (int32_t)pressure_pa;
    }
    if (temperature) {
        *temperature = (int32_t)(temperature_c * 100); // Convert to centidegrees
    }
}

// Device detection and initialization
bool icp201xxDetect(baroDev_t *baro)
{
    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;
    bool ret = true;
    delay(1000);

    DEBUG_SET(DEBUG_BARO, 0, ICP201XX_DEBUG_DETECT_START);

    if (dev->bus->busType == BUS_TYPE_I2C) {
        if (dev->busType_u.i2c.address == 0) {
            // Default address for ICP201XX
            dev->busType_u.i2c.address = ICP201XX_I2C_ADDR;
            defaultAddressApplied = true;
        }
    } else if (dev->bus->busType == BUS_TYPE_SPI) {
        // SPI initialization
#ifdef USE_BARO_SPI_ICP201XX
        IOHi(dev->busType_u.spi.csnPin); // Disable CS
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(ICP201XX_MAX_SPI_CLK_HZ));
        spiSetClkPhasePolarity(dev, false); // SPI_MODE3: CPOL=1, CPHA=1
        delay(50); // Allow SPI configuration to settle
#endif
    } else {
        return false; // Unsupported bus type
    }

    delay(100); // Extended initial delay for sensor stability

    uint8_t id = 0xFF;
    uint8_t ver = 0xFF;

    DEBUG_SET(DEBUG_BARO, 1, ICP201XX_DEBUG_ID_READ);

    // Read device ID twice (as per ArduPilot implementation)
    delay(10); // Small delay before first critical read
    icp201xx_transfer_read(dev, REG_DEVICE_ID, &id, 1);
    delay(5);  // Small delay between reads
    icp201xx_transfer_read(dev, REG_DEVICE_ID, &id, 1);

    DEBUG_SET(DEBUG_BARO, 2, id); // Show actual ID read

    delay(5);  // Small delay before version read
    icp201xx_transfer_read(dev, REG_VERSION, &ver, 1);

    DEBUG_SET(DEBUG_BARO, 3, ver); // Show actual version read

    if (id != ICP201XX_DEVICE_ID) {
        DEBUG_SET(DEBUG_BARO, 0, ICP201XX_DEBUG_FAIL_ID);
        ret = false;
        goto cleanup;
    }

    DEBUG_SET(DEBUG_BARO, 1, ICP201XX_DEBUG_ID_VALID);

    if (ver != 0x00 && ver != 0xB2) {
        DEBUG_SET(DEBUG_BARO, 0, ICP201XX_DEBUG_FAIL_VER);
        ret = false;
        goto cleanup;
    }

    DEBUG_SET(DEBUG_BARO, 1, ICP201XX_DEBUG_VERSION_READ);

    delay(10);

    // Perform soft reset
    icp201xx_soft_reset(dev);
    DEBUG_SET(DEBUG_BARO, 1, ICP201XX_DEBUG_RESET_DONE);

    // Run boot sequence if needed
    DEBUG_SET(DEBUG_BARO, 0, ICP201XX_DEBUG_BOOT_START);
    if (!icp201xx_boot_sequence(dev)) {
        DEBUG_SET(DEBUG_BARO, 0, ICP201XX_DEBUG_FAIL_BOOT);
        ret = false;
        goto cleanup;
    }
    DEBUG_SET(DEBUG_BARO, 1, ICP201XX_DEBUG_BOOT_DONE);

    // Configure the device
    DEBUG_SET(DEBUG_BARO, 0, ICP201XX_DEBUG_CONFIG_START);
    if (!icp201xx_configure(dev)) {
        DEBUG_SET(DEBUG_BARO, 0, ICP201XX_DEBUG_FAIL_CONFIG);
        ret = false;
        goto cleanup;
    }
    DEBUG_SET(DEBUG_BARO, 1, ICP201XX_DEBUG_CONFIG_DONE);

    // Wait for initial readings
    DEBUG_SET(DEBUG_BARO, 0, ICP201XX_DEBUG_WAIT_START);
    icp201xx_wait_read(dev);
    DEBUG_SET(DEBUG_BARO, 1, ICP201XX_DEBUG_WAIT_DONE);

    // Register the device
    busDeviceRegister(dev);

    DEBUG_SET(DEBUG_BARO, 0, ICP201XX_DEBUG_SUCCESS);

cleanup:
    if (!ret) {
        if (dev->bus->busType == BUS_TYPE_SPI) {
#ifdef USE_BARO_SPI_ICP201XX
            ioPreinitByTag(barometerConfig()->baro_spi_csn, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
#endif
        } else if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    // Set up barometer interface
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->start_ut = icp201xx_start_ut;
    baro->read_ut = icp201xx_read_ut;
    baro->get_ut = icp201xx_get_ut;
    baro->up_delay = ICP201XX_CONVERSION_INTERVAL / 2; // 12.5ms
    baro->start_up = icp201xx_start_up;
    baro->read_up = icp201xx_read_up;
    baro->get_up = icp201xx_get_up;
    baro->calculate = icp201xx_calculate;

    return true;
}

#endif // USE_BARO && (USE_BARO_ICP201XX || USE_BARO_SPI_ICP201XX)
