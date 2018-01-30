#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "handler.h"
#include "TSL2561/TSL2561.h"

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
		tsl2561_init();
		uint32_t lux_value = getLuminosity();
		printf("Current lux: %d\r\n", lux_value);
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
	configure_i2c();
	configure_i2c_callbacks();

	for (int slave_address = 0; slave_address < 128; slave_address++) {
		enum status_code i2c_status;
		wr_packet.address = slave_address;
		rd_packet.address = slave_address;
		wr_packet.data_length = 1;
		wr_buffer[0] = 0x05;
		wr_packet.data = wr_buffer;
		i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &wr_packet);
		if (i2c_status == STATUS_OK) {
			i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &rd_packet);
			printf("Address found at %#X\r\n", slave_address);
		}
		i2c_master_send_stop(&i2c_master_instance);
	}
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
	} else if (!(strcmp("relay_set", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("relay_set", required_args, argc);
			return;
		}
		gpio_set('b', 2);
	}  else if (!(strcmp("relay_clear", argv[0]))) {
		int required_args = 1;
		if (argc != required_args) {
			print_args_error("relay_clear", required_args, argc);
			return;
		}
		gpio_clear('b', 2);
	} else {
		printf("Invalid input. See help for correct usage.\r\n");
	}
}

/**
 * sets up i2c with basic configuration.
 *
 * config is as follows:
 *      @li Timeout: 65535
 *      @li Pin SDA: PA08 //TODO: CONFIRM
 *      @li Pin SCL: PA09 //TODO: CONFIRM
 *      @li Clock Generator: @ref GCLK_GENERATOR_0
 */
void configure_i2c()
{
	/* Initialize config structure and software module */
	struct i2c_master_config config_i2c_master;

	i2c_master_get_config_defaults(&config_i2c_master);
	/* Change buffer timeout to something longer */
	config_i2c_master.buffer_timeout = 65535;
	config_i2c_master.pinmux_pad0 = PINMUX_PA08D_SERCOM2_PAD0;
	config_i2c_master.pinmux_pad1 = PINMUX_PA09D_SERCOM2_PAD1;
	config_i2c_master.generator_source = GCLK_GENERATOR_0;
	/* Initialize and enable device with config */
	while (i2c_master_init(&i2c_master_instance, CONF_I2C_MASTER_MODULE, &config_i2c_master) != STATUS_OK);
	i2c_master_enable(&i2c_master_instance);
}

/**
 * Callback after a successful I2C write.
 * @param module i2c module to bind to
 */
void i2c_write_complete_callback(struct i2c_master_module *const module)
{
	/* Initiate new packet read */
	i2c_master_read_packet_job(&i2c_master_instance, &rd_packet);
}

/**
 * Configures callbacks for I2C
 */
void configure_i2c_callbacks(void)
{
	/* Register callback function. */
	i2c_master_register_callback(&i2c_master_instance, i2c_write_complete_callback,
				     I2C_MASTER_CALLBACK_WRITE_COMPLETE);
	i2c_master_enable_callback(&i2c_master_instance,
				   I2C_MASTER_CALLBACK_WRITE_COMPLETE);
}

/**
 * Configures USART for PA20 and PA21.
 *
 * Config as follows:
 *      @li Baudrate: 115200
 *      @li RX Pin: PA20
 *      @li TX Pin: PA21
 *      @li SERCOM: SERCOM3
 */
void configure_usart(void)
{
	struct usart_config config_usart;

	usart_get_config_defaults(&config_usart);
	config_usart.baudrate = 9600;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;

	
	//config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	//config_usart.pinmux_pad0 = PINMUX_UNUSED;
	//config_usart.pinmux_pad1 = PINMUX_UNUSED;
	//config_usart.pinmux_pad2 = PINMUX_PA20D_SERCOM3_PAD2;
	//config_usart.pinmux_pad3 = PINMUX_PA21D_SERCOM3_PAD3;

	stdio_serial_init(&usart_instance, EDBG_CDC_MODULE, &config_usart);

	usart_enable(&usart_instance);
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
	config_adc.reference = ADC_REFERENCE_INTVCC1;
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
