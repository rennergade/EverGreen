#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "handler.h"
#include "hdc_1080.h"
#include "TSL2561.h"

#define BL_VERSION "0.0.0" /// bootloader version info

#define APP_VERSION "0.0.0"/// application version info

/// Available GPIO pins for A and B ports
#define GPIO_PIN_A_1 8 //TODO: change a8/a9 as these are i2c
#define GPIO_PIN_A_2 9
#define GPIO_PIN_B_1 2
#define GPIO_PIN_B_2 3

///ADC Pin
#define ADC_PORT 'a'
#define ADC_PIN 2

//TODO: read_ is not implemented correctly

/**
 * helper function to determine which pin to use in GPIO.
 * @param  port port to use. Either A or B
 * @param  pin  pin that's being selected. Depends on port
 * @return      value of the pin that was selected.
 */
int get_gpio_pin(char port, int pin)
{
	int pin_val = -1;

	switch (port) {
	case 'a':
		switch (pin) {
		//TODO: decide which pins can be set in port A 08, 09
		case GPIO_PIN_A_1:
			pin_val = PIN_PA08;
			break;
		case GPIO_PIN_A_2:
			pin_val = PIN_PA09;
			break;
		default:
			//TODO: Error codes
			printf("Only PA%d, PA%d, PB%d, PB%d,can be set\r\n", GPIO_PIN_A_1, GPIO_PIN_A_2, GPIO_PIN_B_1, GPIO_PIN_B_2);
			break;
		}
	case 'b':
		switch (pin) {
		case GPIO_PIN_B_1:
			pin_val = PIN_PB02;
			break;
		case GPIO_PIN_B_2:
			pin_val = PIN_PB03;
			break;
		default:
			printf("Only PA%d, PA%d, PB%d, PB%d,can be set\r\n", GPIO_PIN_A_1, GPIO_PIN_A_2, GPIO_PIN_B_1, GPIO_PIN_B_2);
			//TODO: error codes
		}
	}
	return pin_val;
}

/**
 * Prints out information about the available cli commands.
 */
void help()
{
	printf("help - Prints all the available commands and a short synopsis \r\n"
	       "ver_bl - Prints the bootloader firmware version \r\n"
	       "ver_app	- Prints the application code firmware version \r\n"
	       "gpio_set [port] [pin] - Set a GPIO pin to high / 1 \r\n"
	       "gpio_clear [port] [pin]	- Set a GPIO pin to low / 0 \r\n"
	       "gpio_get [port] [pin] - Get state of specified GPIO pin \r\n"
	       "mac - returns the mac address of the device \r\n"
	       "ip - returns the IPv4 address \r\n"
	       "read_<sensor> [readings] [interval] - Prints a number of readings at the given interval \r\n"
	       "adc_get [port] [pin] - Get the ADC value of the given pin. \r\n"
	       "mcu_temp - Reports the temperature of the mcu in Celsius. \r\n"
	       "i2c_scan - Prints out a list connected I2C slave addresses \r\n"
		   "relay_set - turns on relay \r\n"
		   "relay_clear - turns off relay \r\n");
}

/**
 * Prints out the bootloader version.
 *
 * Version follows correct Semantic Versioning guidelines. <br>
 * <b>See</b> <a href="https://semver.org/">SemVer</a> for more information.
 */
void ver_bl()
{
	printf("Version: %s \r\n", BL_VERSION);
}

/**
 * Prints out the application version.
 *
 * Version follows correct Semantic Versioning guidelines. <br>
 * <b>See</b> <a href="https://semver.org/">SemVer</a> for more information.
 */
void ver_app()
{
	printf("Version: %s \r\n", APP_VERSION);
}

/**
 * Sets the GPIO pin specificied by @c port and @c pin.
 *
 * Setting a pin pulls the voltage to a logical 1 or HIGH.
 * @param port port specificed. Either A or B
 * @param pin  pin to use.
 */
void gpio_set(char port, int pin)
{
	int pin_val = get_gpio_pin(port, pin);

	if (pin_val != -1) {
		configure_port_pins_set(pin_val); //TODO: decide if this is necessary
		port_pin_set_output_level(pin_val, true);

		printf("Pin %d set high\r\n", pin_val);
	}
}

/**
 * Clears the GPIO pin specificied by @c port and @c pin.
 *
 * Setting a pin pulls the voltage to a logical 0 or LOW.
 * @param port port specificed. Either A or B
 * @param pin  pin to use.
 */
void gpio_clear(char port, int pin)
{
	int pin_val = get_gpio_pin(port, pin);

	if (pin_val != -1) {
		configure_port_pins_set(pin_val); //TODO: decide if this is necessary
		port_pin_set_output_level(pin_val, 0);

		printf("Pin %d cleared\r\n", pin_val);
	}
}

/**
 * Gets the value of the GPIO pin specificied by @c port and @c pin.
 *
 * Will either be logical 0 or 1.
 * @param port port specificed. Either A or B
 * @param pin  pin to use.
 */
void gpio_get(char port, int pin)
{
	int pin_val = get_gpio_pin(port, pin);

	configure_port_pins_get(pin_val); //TODO: decide if this is necessary
	int state = port_pin_get_input_level(pin_val);
	printf("pin %d value: %d\r\n", pin_val, state);
}

/**
 * Prints the MAC address of the Wi-Fi chip.
 *
 * For more information on MAC addresses, see <a href="https://en.wikipedia.org/wiki/MAC_address">Wikipedia</a>
 */
void mac()
{
	printf("00.00.00.00\r\n");
}


/**
 * Prints the IP address of the Wi-Fi chip.
 *
 * @note Currently this only returns a dummy IP address and will need to be updated.
 */
void ip()
{
	printf("255.255.255.255 \r\n");
}

/**
 * Reads values from an attached sensor a set number of times specified by @c readings at intervals (in ms) given by @c interval_ms.
 *
 * @note Currently this only returns dummy info and will need to be implemented.
 * @note Currently the CLI is not configured correctly and instead should run specific to read_<sensor> instead of read <sensor> ...
 *
 * @param sensor_name name of the sensor to read from
 * @param readings    number of readings to do
 * @param interval_ms how many milliseconds to wait between readings
 */
void read_sensor(char *sensor_name, int readings, int interval_ms)
{
	//TODO: check for values greater than 0 for readings and interval_ms
	static uint8_t read_buffer[10];
	
	if(!strcmp("lux", sensor_name)) {
		power_on_tsl2561();
		printf("Current lux: %d\r\n", get_lux());
		power_off_tsl2561();
	}
	
	if(!strcmp("temp", sensor_name)) {
		double temperature = get_temp();
		double humidity = get_humidity();
		printf("Current temperature: %3.2f\r\n Current humidity: %3.2f\r\n", temperature, humidity);
	}
	
	if(!strcmp("moisture", sensor_name)) {
		float m_value = get_moisture();
		printf("Current moisture: %.02f %% \r\n", m_value);
	}
}

/**
 * Reads ADC values from the pin specificied by @c port and @c pin.
 *
 * ADC is only valid at pin PA02 currently. Functionality may be extended for more pins later.
 *
 * @param port port to read from. Only accepts a currently.
 * @param pin  pin to read from. Only accepts 02 currently.
 */
void adc_get(char port, int pin)
{
	int pin_val = -1;

	switch (port) {
		case ADC_PORT:
			switch (pin) {
				case ADC_PIN:
					pin_val = ADC_POSITIVE_INPUT_PIN0;
					break;
				default:
					printf("fail on pin ADC can currently only be configured on P%c%d. Please try again.\r\n", toupper(ADC_PORT), ADC_PIN);
					break;
			}
			break;
		default:
			printf("fail on port ADC can currently only be configured on %c%d. Please try again.\r\n", toupper(ADC_PORT), ADC_PIN);
			break;
	}
	
	if (pin_val != -1) {
		configure_adc(pin_val);
		uint16_t adc_result;
		adc_start_conversion(&adc_instance);
		/* Wait for conversion to be done and read out result */
		do {
		} while (adc_read(&adc_instance, &adc_result) == STATUS_BUSY);
		float voltage = (adc_result/4095.0)*1.65; //TODO: set values, NO MAGIC NUMBERS
		printf("Voltage at P%c%d: %f\r\n", toupper(port), pin_val, voltage);
	}
}

/* Gets the moisture value from SEN13322
*
*/
//TODO Set reference value
float get_moisture(void)
{
	// turn on sensor
	configure_port_pins_set(PIN_PA17);
	port_pin_set_output_level(PIN_PA17, true);
	
	delay_ms(500);
	
	// read ADC
	uint16_t adc_result;
	adc_start_conversion(&adc_instance);
	/* Wait for conversion to be done and read out result */
	do {
	} while (adc_read(&adc_instance, &adc_result) == STATUS_BUSY);
	float moisture = (adc_result/4095.0f)*(100.0f); //TODO: set values, NO MAGIC NUMBERS
	
	//turn off sensor
	port_pin_set_output_level(PIN_PA17, false);
	

	return moisture;

	
}

// 1 = update 0 check for upgrade
void otafu(uint8_t update_or_firmware) {
	if(update_or_firmware) {
		if(download_firmware()) {
			printf("firmware successfully downloaded!\r\n");
		} else {
			printf("download failed...\r\n");
		}
	} else {
		if(check_for_update()) {
			printf("update available!\r\n");
		} else {
			printf("on latest version\r\n");
		}
	}
	
}


/**
 * Gets the temperature in Celsius of the MCU.
 *
 * See the <a href="http://asf.atmel.com/docs/3.21.0/samd21/html/group__asfdoc__sam0__at30tse75x__group.html">board documentation</a> for more information.
 */
//TODO: Use the ASF temp sensor library instead.
void mcu_temp()
{
	int pin = ADC_POSITIVE_INPUT_TEMP;
	uint16_t adc_result;
	configure_adc(pin);

	adc_start_conversion(&adc_instance);
	/* Wait for conversion to be done and read out result */
	do {
	} while (adc_read(&adc_instance, &adc_result) == STATUS_BUSY);
	
	//Equation found here: https://github.com/jrowberg/i2cdevlib/pull/59/files
	double temperature;
	if((adc_result & 0x8000) == 0) {
      temperature = (adc_result >> 8) + ((adc_result & 0x00F0)>>4)*0.5;
    }
    else {
      uint16_t twosComplement = (~adc_result) + 1;
      temperature = - (twosComplement >> 8) - ((twosComplement & 0x00F0)>>4)*0.5;
    }
	printf("MCU temperature: %dC \r\n", (int) temperature);
}

/**
 * Reports all connected I2C slave devices over a 7 bit (128) address space.
 *
 */
void i2c_scan()
{
	struct i2c_master_packet wr_packet = {
		.address		= 0,
		.data_length		= 1,
		.data			=  0,
		.ten_bit_address	= false,
		.high_speed		= false,
		.hs_master_code		= 0x0,
	};
	struct i2c_master_packet rd_packet = {
		.address		= 0,
		.data_length		= 1,
		.data			=  0,
		.ten_bit_address	= false,
		.high_speed		= false,
		.hs_master_code		= 0x0,
	};
	for (int slave_address = 0; slave_address < 128; slave_address++) {
		enum status_code i2c_status;
		wr_packet.address = slave_address;
		rd_packet.address = slave_address;
		wr_packet.data_length = 0;
		uint8_t wr_buffer = 0x05;
		wr_packet.data = &wr_buffer;
		i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_tsl2561, &wr_packet);
		if (i2c_status == STATUS_OK) {
			i2c_status = i2c_master_read_packet_wait(&i2c_tsl2561, &rd_packet);
			printf("Address found at %#X\r\n", slave_address);
		}
		i2c_master_send_stop(&i2c_tsl2561);
	}
	
	printf("Scanning temp bus\r\n");
	

	for (int slave_address = 0; slave_address < 128; slave_address++) {
		//int slave_address = 64;
		enum status_code i2c_status;
		wr_packet.address = slave_address;
		rd_packet.address = slave_address;
		i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_hdc, &wr_packet);
		if (i2c_status == STATUS_OK) {
			i2c_status = i2c_master_read_packet_wait(&i2c_hdc, &rd_packet);
			printf("Address found at %#X\r\n", slave_address);
		}
		i2c_master_send_stop(&i2c_hdc);
	}
	
	printf("Scans complete.\r\n");
}

/**
 * Prints a general error message.
 *
 * @param func_name function where error occured.
 */
void print_general_error(char *func_name)
{
	printf("there was an error parsing your args for %s. See help for correct usage.\r\n", func_name);
}
/**
 * Prints an error message if the incorrect number of args were given.
 *
 * @param func_name     name of the function where error occured
 * @param required_args number of arguments required for the function
 * @param num_args      number of arguments given to the function
 */
void print_args_error(char *func_name, int required_args, int num_args)
{
	printf("Invalid number of args: %s requires %d arguments and you provided %d \r\n."
	       "See help for more information. \r\n", func_name, required_args, num_args);
}

/**
 * handles input given to the CLI and calls the appropriate function.
 *
 * @param argc argument count, number of arguments given.
 * @param argv buffer holding all arguments
 */
void input_handle(int argc, char **argv)
{
	if (!(strcmp("help", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("help", required_args, argc);
			return;
		}
		help();
	} else if (!(strcmp("ver_bl", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("ver_bl", required_args, argc);
			return;
		}
		ver_bl();
	} else if (!(strcmp("ver_app", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("ver_app", required_args, argc);
			return;
		}
		ver_app();
	} else if (!(strcmp("gpio_set", argv[0]))) {
		int required_args = 3;
		if (argc != required_args) {
			print_args_error("gpio_set", required_args, argc);
			return;
		}
		char port = argv[1][0];
		int pin = atoi(argv[2]);
		if (isdigit(argv[2][0]))
			gpio_set(port, pin);
		else
			print_general_error("gpio_set");
	} else if (!(strcmp("gpio_clear", argv[0]))) {
		int required_args = 3;
		if (argc != required_args) {
			print_args_error("gpio_clear", required_args, argc);
			return;
		}
		char port = argv[1][0];
		int pin = atoi(argv[2]);
		if (isdigit(argv[2][0]))
			gpio_clear(port, pin);
		else
			print_general_error("gpio_clear");
	} else if (!(strcmp("gpio_get", argv[0]))) {
		int required_args = 3;
		if (argc != required_args) {
			print_args_error("gpio_get", required_args, argc);
			return;
		}
		char port = argv[1][0];
		int pin = atoi(argv[2]);
		if (isdigit(argv[2][0]))
			gpio_get(port, pin);
		else
			print_general_error("gpio_get");
	} else if (!(strcmp("mac", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("mac", required_args, argc);
			return;
		}
		mac();
	} else if (!(strcmp("ip", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("ip", required_args, argc);
			return;
		}
		ip();
	} else if (!(strcmp("read", argv[0]))) {
		int required_args = 4;
		if (argc != required_args) {
			print_args_error("read", required_args, argc);
			return;
		}
		int reading = atoi(argv[2]);
		int interval_ms = atoi(argv[3]);
		if (isdigit(argv[2][0]) && isdigit(argv[3][0]))
			read_sensor(argv[1], reading, interval_ms);
		else
			print_general_error("read");
	} else if (!(strcmp("adc_get", argv[0]))) {
		int required_args = 3;
		if (argc != required_args) {
			print_args_error("adc_get", required_args, argc);
			return;
		}
		char port = argv[1][0];
		int pin = atoi(argv[2]);
		//TODO: isdigit should check [2][0] for all isdigit calls
		if (isdigit(argv[2][0])) {
			adc_get(port, pin);
		} else {
			print_general_error("adc_get");
		}
	} else if (!(strcmp("mcu_temp", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("mcu_temp", required_args, argc);
			return;
		}
		mcu_temp();
	} else if (!(strcmp("i2c_scan", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("i2c_scan", required_args, argc);
			return;
		}
		printf("running i2c_scan\r\n");
		i2c_scan();
	} else if (!(strcmp("relay1_on", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("relay1_on", required_args, argc);
			return;
		}
	printf("Relay 1 turning on.\r\n");
	relay1_enable();
	}  else if (!(strcmp("relay1_off", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("relay1_off", required_args, argc);
			return;
		}
	printf("Relay 1 turning off.\r\n");
	relay1_disable();
	}  else if (!(strcmp("relay2_on", argv[0]))) {
	int required_args = 1;
	if (argc != required_args) {
		print_args_error("relay2_on", required_args, argc);
		return;
	}
	printf("Relay 2 turning on.\r\n");
	relay2_enable();
	}  else if (!(strcmp("relay2_off", argv[0]))) {
	int required_args = 1;
	if (argc != required_args) {
		print_args_error("relay2_off", required_args, argc);
		return;
	}
	printf("Relay 2 turning off.\r\n");
	relay2_disable();
	} else if (!(strcmp("boost_on", argv[0]))) {
	int required_args = 1;
	if (argc != required_args) {
		print_args_error("boost_on", required_args, argc);
		return;
	}
	printf("Boost converter enabled.\r\n");
	boost_enable();
	}  else if (!(strcmp("boost_off", argv[0]))) {
	int required_args = 1;
	if (argc != required_args) {
		print_args_error("boost_off", required_args, argc);
		return;
	}
	printf("Boost converter disabled.\r\n");
	boost_disable();
	} else if (!(strcmp("led1_on", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("led1_on", required_args, argc);
			return;
		}
	printf("LED1 turning on.\r\n");
	led1_on();
	} else if (!(strcmp("led1_off", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
		print_args_error("led1_off", required_args, argc);
		return;
		}
	printf("LED1 turning off.\r\n");
	led1_off();
	} else if (!(strcmp("led2_on", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("led2_on", required_args, argc);
			return;
		}
	printf("LED2 turning on.\r\n");
	led2_on();
	} else if (!(strcmp("led2_off", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("led2_off", required_args, argc);
			return;
		}
	printf("LED2 turning off.\r\n");
	led2_off();
	} else if (!(strcmp("check_for_update", argv[0]))) {
	int required_args = 1;
	if (argc != required_args) {
		print_args_error("check_for_update", required_args, argc);
		return;
	}
	otafu(0);
	} else if (!(strcmp("download_update", argv[0]))) {
	int required_args = 1;
	if (argc != required_args) {
		print_args_error("download_update", required_args, argc);
		return;
	}
	otafu(1);
	} else if (!(strcmp("run_pump", argv[0]))) {
	int required_args = 2;
	if (argc != required_args) {
		print_args_error("run_pump", required_args, argc);
		return;
	}
	int duration = atoi(argv[1]);
	if (isdigit(argv[1][0])) {
		run_pump(duration);
	}
	else
	print_general_error("run_pump");
	
	} else {
		printf("Invalid input. See help for correct usage.\r\n");
	}
}

/**
 * Configures ADC with default values
 *
 * Configuration as follows:
 *      @li ADC Pin: @c pin
 *      @li Reference voltage: 1/2* VCC (1.65V)
 *      @li clock prescale: 16x
 * @param pin pin to read value from
 */
void configure_adc(int pin)
{
	struct adc_config config_adc;

	adc_get_config_defaults(&config_adc);
	config_adc.positive_input = pin;
	config_adc.reference = ADC_REFERENCE_INTVCC0;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
}

/**
 * Sets a given port to output.
 * @param pin pin of the port to set
 */
void configure_port_pins_set(int pin)
{
	struct port_config config_port_pin;

	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(pin, &config_port_pin);
}

/**
 * Sets a given port to input
 * @param pin pin to set as input
 */
void configure_port_pins_get(int pin)
{
	struct port_config config_port_pin;

	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(pin, &config_port_pin);
}

/**
 * Functions to toggle LEDs
 */
void led1_on(void)
{
	configure_port_pins_set(PIN_PA21);
	port_pin_set_output_level(PIN_PA21, true);	
}

void led1_off(void)
{
	configure_port_pins_set(PIN_PA21);
	port_pin_set_output_level(PIN_PA21, false);
}

void led2_on(void)
{
	configure_port_pins_set(PIN_PA03);
	port_pin_set_output_level(PIN_PA03, true);
}

void led2_off(void)
{
	configure_port_pins_set(PIN_PA03);
	port_pin_set_output_level(PIN_PA03, false);
}

void boost_enable(void)
{
	configure_port_pins_set(PIN_PA20);
	port_pin_set_output_level(PIN_PA20, true);
}

void boost_disable(void)
{
	configure_port_pins_set(PIN_PA20);
	port_pin_set_output_level(PIN_PA20, false);
}

void relay1_enable(void)
{
	configure_port_pins_set(PIN_PB02);
	port_pin_set_output_level(PIN_PB02, true);
}

void relay1_disable(void)
{
	configure_port_pins_set(PIN_PB02);
	port_pin_set_output_level(PIN_PB02, false);
}

void relay2_enable(void)
{
	configure_port_pins_set(PIN_PB03);
	port_pin_set_output_level(PIN_PB03, true);
}

void relay2_disable(void)
{
	configure_port_pins_set(PIN_PB03);
	port_pin_set_output_level(PIN_PB03, false);
}

void gpio5_enable(void)
{
	configure_port_pins_set(PIN_PB23);
	port_pin_set_output_level(PIN_PB23, true);
}

void gpio5_disable(void)
{
	configure_port_pins_set(PIN_PB23);
	port_pin_set_output_level(PIN_PB23, false);
}

/**
 * ASF PWM example
 */
static void configure_tcc_pwm(void)
{
	struct tcc_config config_tcc;
	tcc_get_config_defaults(&config_tcc, CONF_PWM_MODULE);
	config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV256;
	config_tcc.counter.period = 0xFFFF;
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_MATCH_FREQ;
	config_tcc.compare.match[CONF_PWM_CHANNEL] = (0xFFFF / (20*0xFFFF));
	config_tcc.pins.enable_wave_out_pin[CONF_PWM_OUTPUT] = true;
	config_tcc.pins.wave_out_pin[CONF_PWM_OUTPUT]        = CONF_PWM_OUT_PIN;
	config_tcc.pins.wave_out_pin_mux[CONF_PWM_OUTPUT]    = CONF_PWM_OUT_MUX;
	tcc_init(&tcc_instance_pwm, CONF_PWM_MODULE, &config_tcc);
	tcc_enable(&tcc_instance_pwm);
}

void ramp_tcc_pwm(int duty)
{
	tcc_disable(&tcc_instance_pwm);
	
	struct tcc_config config_tcc;
	
	tcc_get_config_defaults(&config_tcc, CONF_PWM_MODULE);
	config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV256;
	config_tcc.counter.period = 0xFFFF/2;
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_NORMAL_FREQ;
	config_tcc.compare.match[CONF_PWM_CHANNEL] = (0xFFFF / 0xFFFF);
	config_tcc.pins.enable_wave_out_pin[CONF_PWM_OUTPUT] = true;
	config_tcc.pins.wave_out_pin[CONF_PWM_OUTPUT]        = CONF_PWM_OUT_PIN;
	config_tcc.pins.wave_out_pin_mux[CONF_PWM_OUTPUT]    = CONF_PWM_OUT_MUX;

	tcc_init(&tcc_instance_pwm, CONF_PWM_MODULE, &config_tcc);
	tcc_enable(&tcc_instance_pwm);
}

void run_pump(int duration) {
	
	//boost_enable();
	gpio5_enable();
	//configure_tcc_pwm();
	
// 	for (int i = 10; i >= 1; i--) //set pwm range
// 		{
// 			ramp_tcc_pwm(0xFFFF*i);
// 			delay_ms(2000);
// 		}
		
	delay_ms(duration); //duration of pump
	
	//tcc_disable(&tcc_instance_pwm);
	gpio5_disable();
	//boost_disable();
	
}