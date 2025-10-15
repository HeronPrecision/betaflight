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

#if defined(USE_MAG_BMM350)

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"

#include "compass_bmm350.h"

// BMM350 Register definitions (from DFRobot driver)
#define BMM350_REG_CHIP_ID              0x00
#define BMM350_REG_PMU_CMD_AGGR_SET     0x04
#define BMM350_REG_PMU_CMD_AXIS_EN      0x05
#define BMM350_REG_PMU_CMD              0x06
#define BMM350_REG_PMU_CMD_STATUS_0     0x07
#define BMM350_REG_INT_CTRL             0x2E
#define BMM350_REG_MAG_X_XLSB           0x31
#define BMM350_REG_OTP_CMD_REG          0x50
#define BMM350_REG_OTP_DATA_MSB_REG     0x52
#define BMM350_REG_OTP_DATA_LSB_REG     0x53
#define BMM350_REG_OTP_STATUS_REG       0x55
#define BMM350_REG_CMD                  0x7E

// I2C addresses
#define BMM350_I2C_ADDR_MIN             0x14
#define BMM350_I2C_ADDR_MAX             0x17
#define BMM350_I2C_ADDRESS              0x14

// Commands and constants from DFRobot implementation
#define BMM350_CMD_SOFTRESET            0xB6
#define BMM350_CHIP_ID                  0x33

// Power modes
#define BMM350_PMU_CMD_SUS              0x00
#define BMM350_PMU_CMD_NM               0x01
#define BMM350_PMU_CMD_UPD_OAE          0x02
#define BMM350_PMU_CMD_BR               0x07
#define BMM350_PMU_CMD_FGR              0x05

// ODR and averaging
#define BMM350_ODR_100HZ                0x04
#define BMM350_AVG_4                    0x02
#define BMM350_AVERAGING_4              (BMM350_AVG_4 << 4)

// OTP commands
#define BMM350_OTP_CMD_DIR_READ         0x20
#define BMM350_OTP_CMD_PWR_OFF_OTP      0x80
#define BMM350_OTP_WORD_ADDR_MSK        0x1F
#define BMM350_OTP_STATUS_CMD_DONE      0x01
#define BMM350_OTP_STATUS_ERROR_MSK     0xE0

// Delays (microseconds) from DFRobot
#define BMM350_START_UP_TIME_FROM_POR   3000
#define BMM350_SOFT_RESET_DELAY         24000
#define BMM350_BR_DELAY                 14000
#define BMM350_FGR_DELAY                18000
#define BMM350_SUSPEND_TO_NORMAL_DELAY  38000
#define BMM350_GOTO_SUSPEND_DELAY       6000
#define BMM350_UPD_OAE_DELAY            1000

// Data processing constants
#define BMM350_OTP_DATA_LENGTH          32
#define BMM350_EN_XYZ_MSK               0x07

// Simple scaling factors for basic operation (approximate values)
#define BMM350_SCALE_FACTOR             0.0001f

// Global state for OTP data and compensation (simplified) - COMMENTED OUT FOR DUMMY TESTING
// static uint16_t otp_data[BMM350_OTP_DATA_LENGTH];
// static bool otp_loaded = false;

/* COMMENTED OUT FOR DUMMY TESTING
static bool bmm350DelayUs(uint32_t period_us)
{
    if (period_us > 1000) {
        delay(period_us / 1000);
    } else {
        delayMicroseconds(period_us);
    }
    return true;
}

static bool bmm350WaitPmuCmdReady(magDev_t *mag, uint8_t expected_cmd, uint32_t timeout_ms)
{
    uint32_t start = millis();

    do {
        delay(1);
        uint8_t status;
        if (!busReadRegisterBuffer(&mag->dev, BMM350_REG_PMU_CMD_STATUS_0, &status, 1)) {
            return false;
        }
        // Check if command is done (bit 0 == 0) and matches expected command (bits 7:5)
        if (((status & 0x01) == 0x00) && (((status & 0xE0) >> 5) == expected_cmd)) {
            return true;
        }
    } while ((millis() - start) < timeout_ms);

    return false;
}

static bool bmm350SetPowerMode(magDev_t *mag, uint8_t mode)
{
    extDevice_t *dev = &mag->dev;

    // Set power mode
    if (!busWriteRegister(dev, BMM350_REG_PMU_CMD, mode)) {
        return false;
    }

    // Wait for mode change with appropriate delay
    uint32_t delay_time = 40;
    if (mode == BMM350_PMU_CMD_NM) {
        delay_time = BMM350_SUSPEND_TO_NORMAL_DELAY / 1000;
    } else if (mode == BMM350_PMU_CMD_SUS) {
        delay_time = BMM350_GOTO_SUSPEND_DELAY / 1000;
    }

    return bmm350WaitPmuCmdReady(mag, mode, delay_time);
}

static bool bmm350ReadOtpWord(magDev_t *mag, uint8_t addr, uint16_t *data)
{
    extDevice_t *dev = &mag->dev;
    uint8_t otp_cmd, otp_status = 0;
    uint8_t lsb, msb;
    int retries = 10;

    // Set OTP command for reading at specified address
    otp_cmd = BMM350_OTP_CMD_DIR_READ | (addr & BMM350_OTP_WORD_ADDR_MSK);
    if (!busWriteRegister(dev, BMM350_REG_OTP_CMD_REG, otp_cmd)) {
        return false;
    }

    // Wait for OTP operation to complete
    do {
        bmm350DelayUs(300);
        if (!busReadRegisterBuffer(dev, BMM350_REG_OTP_STATUS_REG, &otp_status, 1)) {
            return false;
        }

        // Check for errors
        if (otp_status & BMM350_OTP_STATUS_ERROR_MSK) {
            return false;
        }

        retries--;
    } while ((!(otp_status & BMM350_OTP_STATUS_CMD_DONE)) && (retries > 0));

    if (retries <= 0) {
        return false;
    }

    // Read OTP data
    if (!busReadRegisterBuffer(dev, BMM350_REG_OTP_DATA_MSB_REG, &msb, 1) ||
        !busReadRegisterBuffer(dev, BMM350_REG_OTP_DATA_LSB_REG, &lsb, 1)) {
        return false;
    }

    *data = ((uint16_t)(msb << 8) | lsb) & 0xFFFF;
    return true;
}

static bool bmm350LoadOtpData(magDev_t *mag)
{
    for (uint8_t i = 0; i < BMM350_OTP_DATA_LENGTH; i++) {
        if (!bmm350ReadOtpWord(mag, i, &otp_data[i])) {
            return false;
        }
    }
    otp_loaded = true;
    return true;
}

static bool bmm350MagneticResetAndWait(magDev_t *mag)
{
    extDevice_t *dev = &mag->dev;
    uint8_t pmu_cmd;
    bool restore_normal = false;

    // Check current power mode
    uint8_t status;
    if (!busReadRegisterBuffer(dev, BMM350_REG_PMU_CMD_STATUS_0, &status, 1)) {
        return false;
    }

    // If in normal mode, switch to suspend first
    if ((status & 0xE0) >> 5 == BMM350_PMU_CMD_NM) {
        restore_normal = true;
        if (!bmm350SetPowerMode(mag, BMM350_PMU_CMD_SUS)) {
            return false;
        }
    }

    // Perform BR (Bit Reset)
    pmu_cmd = BMM350_PMU_CMD_BR;
    if (!busWriteRegister(dev, BMM350_REG_PMU_CMD, pmu_cmd)) {
        return false;
    }
    bmm350DelayUs(BMM350_BR_DELAY);

    // Verify BR command
    if (!bmm350WaitPmuCmdReady(mag, BMM350_PMU_CMD_BR, 20)) {
        return false;
    }

    // Perform FGR (Flux Guide Reset)
    pmu_cmd = BMM350_PMU_CMD_FGR;
    if (!busWriteRegister(dev, BMM350_REG_PMU_CMD, pmu_cmd)) {
        return false;
    }
    bmm350DelayUs(BMM350_FGR_DELAY);

    // Verify FGR command
    if (!bmm350WaitPmuCmdReady(mag, BMM350_PMU_CMD_FGR, 25)) {
        return false;
    }

    // Restore normal mode if it was previously enabled
    if (restore_normal) {
        if (!bmm350SetPowerMode(mag, BMM350_PMU_CMD_NM)) {
            return false;
        }
    }

    return true;
}
END COMMENTED OUT FUNCTIONS */

static bool bmm350Read(magDev_t *mag, int16_t *magData)
{
    UNUSED(mag);
    // DUMMY READ - Return fake compass data
    magData[X] = 100;  // Fake X axis data
    magData[Y] = 200;  // Fake Y axis data
    magData[Z] = 300;  // Fake Z axis data
    return true;
}

static bool bmm350Init(magDev_t *mag)
{
    // DUMMY INIT - Just set ODR and return success
    mag->magOdrHz = 100;
    return true;
}

bool bmm350Detect(magDev_t* mag)
{
    extDevice_t *dev = &mag->dev;

    // Set default I2C address if not specified
    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = BMM350_I2C_ADDRESS;
    }

    // Test I2C communication by trying to read any register
    // uint8_t dummy_data = 0;
    // bool ack = busReadRegisterBuffer(dev, BMM350_REG_CHIP_ID, &dummy_data, 1);

    // if (!ack) {
    //     return false;
    // }

    // Set retry loop for 5
    // int8_t attempts = 0;
    // while (attempts < 10) {
    //     //Request soft reset
    //     attempts++;
    //     busWriteRegister(dev, BMM350_REG_CMD, BMM350_CMD_SOFTRESET);
    //     delay(240);

    //     // Attempt to read the chip ID
    //     uint8_t chip_id;
    //     busReadRegisterBuffer(dev, BMM350_REG_CHIP_ID, &chip_id, 1);
    //     debug[0] = chip_id;
    //     if (chip_id == BMM350_CHIP_ID) {
    //         break;
    //     }
    // }
    // if (attempts >= 10) {
    //     return false;
    // }

    // Set function pointers
    mag->init = bmm350Init;
    mag->read = bmm350Read;

    return true;
}

#endif
