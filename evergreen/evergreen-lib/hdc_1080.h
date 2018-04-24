/**
   @file hdc_1080.h
   @brief Header file for Texas Instruments HDC 1080 temperature and relative humidity sensor
   @author William Archer
   @date 22/Apr/2018
**/

/// <a href="http://www.ti.com/lit/ds/symlink/hdc1080.pdf">see datasheet for more information</a>

/// @note This driver is incomplete. It only includes test functions and functionality for humidity and temperature.
/// @details It does not include support for heating up the device to remove moisture in high humidity environments.
#ifndef HDC_1080_H_
#define HDC_1080_H_

#include <asf.h>

#define I2C_TIMEOUT 65535
#define HDC_SLAVE_ADDR 0x40

/**
 * @enum hdc_request
 * List of commands you can give to device to receive data back.
 */
enum hdc_request {
	HDC_TEMP = 0x00,
	HDC_HUMIDITY = 0x01,
	HDC_SET_RES = 0x02,
	HDC_MANUFACTURER_ID = 0xFE,
	HDC_DEVICE_ID = 0xFF,
	HDC_SERIAL_ID_FIRST = 0xFB,
	HDC_SERIAL_ID_MID = 0xFC,
	HDC_SERIAL_ID_LAST = 0xFD
	};

/// @typedef uint8_t hdc_request
/// @brief typedef enum for easy conversion
typedef uint8_t hdc_request;

/**
 * @enum hdc_resolution
 * List of resolution sizes for HDC driver.
 */
typedef enum {
	EIGHT_BIT_RESOLUTION,
	ELEVEN_BIT_RESOLUTION,
	FOURTEEN_BIT_RESOLUTION
	} hdc_resolution;

struct i2c_master_module i2c_hdc;

/**
 * Initialize i2c for the HDC1080 temp and humidity sensor
 *
 * There cannot be another slave with I2C address 0x40.
 */
void configure_i2c_hdc();
/**
 * Set's the number of bits of resolution for the temperature and humidity
 *
 * Temperature can only be either @ref ELEVEN_BIT_RESOLUTION or @ref FOURTEEN_BIT_RESOLUTION.
 * Humidity can have any of the three resolution scopes.
 * While the level of accuracy affects temperature readings, this driver has a set delay for all, so highest resolution is recommended.
 * @param temp_resolution     resolution for temperature
 * @param humidity_resolution resolutin for humidity
 */
void set_resolution(hdc_resolution temp_resolution, hdc_resolution humidity_resolution);

/**
 * Requests data on the I2C line
 * @param  command command to give the device
 * @return         raw data from the register
 */
uint16_t request_data(hdc_request command);
/**
 * gets the manufacturer id of the device (TI). This should always be 0x5449 or your device is failing.
 * @return 0x5449
 */
uint16_t get_hdc_manufacturer_id();
/**
 * gets the device id. This should always be 0x1050 or your device is not properly connected.
 * @return 0x1050
 */
uint16_t get_hdc_device_id();
/**
 * Gets the current relative humidity as a percentage.
 * @return current relative humidity
 */
double get_humidity();

/**
 * Gets the current temperature in Celsius.
 * @return current temperature
 */
double get_temp();

#endif
