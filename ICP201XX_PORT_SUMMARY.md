# ICP201XX Barometer Driver Port for Betaflight

## Overview

This document summarizes the port of the TDK InvenSense ICP201XX barometer driver from Zephyr OS to Betaflight firmware. The ICP201XX is a high-accuracy MEMS barometer with integrated temperature sensor, supporting both I2C and SPI interfaces.

## Files Created/Modified

### New Driver Files
- `src/main/drivers/barometer/barometer_icp201xx.c` - Main driver implementation
- `src/main/drivers/barometer/barometer_icp201xx.h` - Driver header file
- `src/main/drivers/barometer/icp201xx_defs.h` - Register definitions and constants

### Modified Configuration Files
- `src/config/configs/HERONPRECISION_CHICKADEE/config.h` - Updated to use ICP201XX instead of ICP20100
- `src/main/sensors/barometer.h` - Added BARO_ICP201XX enum (replaced BARO_ICP20100)
- `src/main/sensors/barometer.c` - Added detection and configuration support
- `src/main/cli/settings.c` - Updated barometer hardware lookup table
- `src/main/target/common_pre.h` - Added USE_BARO_ICP201XX define
- `mk/source.mk` - Added driver to build system

## Key Features Implemented

### Hardware Support
- **SPI Mode 0 Interface**: Primary interface using 10MHz max clock
- **I2C Interface**: Fallback support with default addresses 0x63/0x64
- **Device Detection**: Proper chip ID verification (0x63)

### Sensor Configuration
- **Operation Modes**: Multiple ODR/bandwidth combinations (Mode 0-4)
  - Mode 0: 25Hz ODR, 6.25Hz BW
  - Mode 1: 120Hz ODR, 30Hz BW
  - Mode 2: 40Hz ODR, 10Hz BW (default)
  - Mode 3: 2Hz ODR, 0.5Hz BW
  - Mode 4: User configurable
- **FIFO Operation**: Combined pressure+temperature reading
- **Continuous Mode**: Automatic data generation

### Data Processing
- **Pressure Range**: 30-125 kPa with high accuracy
- **Temperature Range**: -40°C to +85°C
- **Resolution**: 20-bit pressure and temperature data
- **Conversion Formulas**:
  - Pressure: `P = (POUT/2^17)*40kPa + 70kPa`
  - Temperature: `T = (TOUT/2^18)*65°C + 25°C`

### Betaflight Integration
- **Combined Read**: Temperature and pressure read together for efficiency
- **Non-blocking Operations**: Uses Betaflight's bus abstraction
- **Warmup Handling**: Discards first 14 samples for FIR filter settling
- **Standard Interface**: Implements baroDev_t function pointers

## Register Mapping

Key registers used from TDK library:
- `0x0C` - Device ID register
- `0xC0` - Mode select register
- `0xC1` - Interrupt status
- `0xC3` - FIFO configuration
- `0xC4` - FIFO fill level
- `0xFA` - FIFO data base
- `0xBE` - Master lock register

## Configuration Notes

### Target Configuration
The driver replaces the previous ICP20100 configuration in HERONPRECISION_CHICKADEE target:
```c
#define USE_BARO_ICP201XX  // Replaces USE_BARO_SPI_ICP20100
```

### SPI Configuration
- Uses SPI4 instance on HERONPRECISION_CHICKADEE
- CS pin controlled via BARO_CS_PIN
- Mode 0 (CPOL=0, CPHA=0) for maximum compatibility

### Default Settings
- Default to I2C operation if no SPI configuration
- 40Hz ODR (Mode 2) for good balance of speed and accuracy
- Combined pressure+temperature FIFO mode
- 25ms update interval to match 40Hz ODR

## Build Verification

The driver has been successfully compiled for the HERONPRECISION_CHICKADEE target:
```bash
make clean && make HERONPRECISION_CHICKADEE
```

## Implementation Notes

### Differences from Zephyr Driver
1. **Bus Abstraction**: Uses Betaflight's unified bus layer instead of Zephyr's device tree
2. **Memory Management**: Static allocation instead of dynamic
3. **Timing**: Uses Betaflight's delay functions instead of kernel sleep
4. **Interface**: Implements baroDev_t structure instead of Zephyr sensor API

### Future Improvements
1. **Interrupt Support**: Could add DRDY interrupt handling for more responsive operation
2. **Advanced Modes**: Implementation of pressure threshold and delta detection
3. **Calibration**: Addition of factory calibration coefficient handling
4. **Power Management**: Sleep/standby mode support for power-sensitive applications

## Testing Recommendations

1. **Hardware Verification**: Ensure proper SPI/I2C connections
2. **Communication Test**: Verify device ID reads correctly (0x63)
3. **Data Validation**: Check pressure/temperature values are reasonable
4. **Performance**: Monitor update rates and timing consistency
5. **Warmup**: Verify first 14 samples are properly discarded

## References

- TDK InvenSense ICP201XX Datasheet
- Zephyr RTOS ICP201XX driver implementation
- Betaflight barometer driver architecture
- Arduino ICP201XX library (pressure.arduino.ICP201XX)