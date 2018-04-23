/*
 * TSL2561.h
 *
 * Created: 1/30/2018 2:54:42 PM
 *  Author: William
 */


/// NOTE: this code is adapted from adafruits TSL2561 arduino library to work on a SAMD21
/// see https://github.com/adafruit/Adafruit_TSL2561 for more information

#ifndef TSL2561_H_
#define TSL2561_H_

#include <asf.h>

#define I2C_TIMEOUT 65535
#define CMD_BIT 0x80 /// bit 8 has to be = 1 for command

// T, FN and CL package values
#define TSL2561_LUX_K1T           (0x0040)  ///< 0.125 * 2^RATIO_SCALE
#define TSL2561_LUX_B1T           (0x01f2)  ///< 0.0304 * 2^LUX_SCALE
#define TSL2561_LUX_M1T           (0x01be)  ///< 0.0272 * 2^LUX_SCALE
#define TSL2561_LUX_K2T           (0x0080)  ///< 0.250 * 2^RATIO_SCALE
#define TSL2561_LUX_B2T           (0x0214)  ///< 0.0325 * 2^LUX_SCALE
#define TSL2561_LUX_M2T           (0x02d1)  ///< 0.0440 * 2^LUX_SCALE
#define TSL2561_LUX_K3T           (0x00c0)  ///< 0.375 * 2^RATIO_SCALE
#define TSL2561_LUX_B3T           (0x023f)  ///< 0.0351 * 2^LUX_SCALE
#define TSL2561_LUX_M3T           (0x037b)  ///< 0.0544 * 2^LUX_SCALE
#define TSL2561_LUX_K4T           (0x0100)  ///< 0.50 * 2^RATIO_SCALE
#define TSL2561_LUX_B4T           (0x0270)  ///< 0.0381 * 2^LUX_SCALE
#define TSL2561_LUX_M4T           (0x03fe)  ///< 0.0624 * 2^LUX_SCALE
#define TSL2561_LUX_K5T           (0x0138)  ///< 0.61 * 2^RATIO_SCALE
#define TSL2561_LUX_B5T           (0x016f)  ///< 0.0224 * 2^LUX_SCALE
#define TSL2561_LUX_M5T           (0x01fc)  ///< 0.0310 * 2^LUX_SCALE
#define TSL2561_LUX_K6T           (0x019a)  ///< 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6T           (0x00d2)  ///< 0.0128 * 2^LUX_SCALE
#define TSL2561_LUX_M6T           (0x00fb)  ///< 0.0153 * 2^LUX_SCALE
#define TSL2561_LUX_K7T           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7T           (0x0018)  ///< 0.00146 * 2^LUX_SCALE
#define TSL2561_LUX_M7T           (0x0012)  ///< 0.00112 * 2^LUX_SCALE
#define TSL2561_LUX_K8T           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8T           (0x0000)  ///< 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8T           (0x0000) ///< 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_LUXSCALE      (14)      ///< Scale by 2^14
#define TSL2561_LUX_RATIOSCALE    (9) ///< Scale ratio by 2^9

// I2C address options

typedef enum {
  ADDR_LOW = 0x29, /// Pin pulled low
  ADDR_FLOAT = 0x39, /// Pin floating
  ADDR_HIGH = 0x49 /// Pin pulled high
} tsl2561_i2c_addr;

/** TSL2561 I2C Registers */
enum tsl2561_registers {
  CTRL_REG = 0x00, /// Control/power register
  TIMING_REG = 0x01, /// Set integration time register
  LOW_THRESHOLD_LOW_BYTE = 0x02, /// Interrupt low threshold low-byte
  LOW_THRESHOLD_HIGH_BYTE = 0x03,/// Interrupt low threshold high-byte
  HIGH_THRESHOLD_LOW_BYTE = 0x04, /// Interrupt high threshold low-byte
  HIGH_THRESHOLD_HIGH_BYTE = 0x05, /// Interrupt high threshold high-byte
  INTERRUPT_CONF_REG = 0x06, /// Interrupt settings
  CRC_REG = 0x08, /// Factory use only
  ID_REG = 0x0A, /// TSL2561 identification setting
  CHAN0_LOW_REG = 0x0C, /// Light data channel 0, low byte
  CHAN0_HIGH_REG = 0x0D, /// Light data channel 0, high byte
  CHAN1_LOW_REG = 0x0E, /// Light data channel 1, low byte
  CHAN1_HIGH_REG = 0x0F /// Light data channel 1, high byte
};

typedef uint8_t tsl2561_registers;

enum tsl2561_integration_readings {
  INTEGRATE_13MS = 0x00,
  INTEGRATE_101MS = 0x01,
  INTEGRATE_402MS = 0x02
};

typedef enum {
  GAIN_1X = 0x00,
  GAIN_16X = 0x10
} tsl2561_gains;

struct i2c_master_module i2c_tsl2561;

/**
 * Initialize i2c for the TSL2561 luminosity sensor.
 */
void configure_i2c_tsl2561(tsl2561_i2c_addr addr);
void power_on_tsl2561();
void power_off_tsl2561();
uint8_t get_tsl2561_device_id();
uint32_t get_lux();

#endif /* TSL2561_H_ */
