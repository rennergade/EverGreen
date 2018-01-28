#include "handler.h"


void input_handle(char* cmd, char* a1, char* a2, char* a3) {
	
	char* port;
	int pinnum;
	int pin;
	int state;
	
	uint16_t adc_result;
	
	switch (cmd) {
		
		case "help":
			printf("help - Prints all the available commands and a short synopsis\r\n"
			"ver_bl - Prints the bootloader firmware version\r\n"
			"ver_app - Prints the application code firmware version\r\n"
			"gpio_set [port] [pin number] - Set a GPIO pin to high / 1\r\n"
			"gpio_clear [port] [pin number] - Set a GPIO pin to low / 0\r\n"
			"gpio_get [port] [pin number] - Get state of specified GPIO pin\r\n"
			"mac - returns the mac address of the device\r\n"
			"ip - returns the ip address of the device in the standard format: ex. 255.255.255.255\r\n"
			"read_<sensor name> [readings] [interval_ms] - Prints a number of readings at the given interval\r\n"
			"adc_get [port] [pin number] - Get the ADC value of the given pin.\r\n"
			"mcu_temp - Prints the temperature of the mcu from the on-board temp sensor in celsius.\r\n"
			"i2c_scan - Prints out a list of addresses for devices on the I2C bus after scanning through all possible 7-bit combinations.\r\n");
			break;
			
			case "ver_bl":
				printf("Version: %s \r\n", VERSION);
				break;
				
			case "ver_app":
				printf("Version: %s \r\n", VERSION);
				break;
				
			case "gpio_set":
				port = a1;
				pinnum = atoi(a2);
				
				if (port == "B") {
					if (pinnum == 2) {
						pin = PIN_PB02;	
					}
					if (pinnum == 3) {
						pin = PIN_PB03;						
					}					
				}
				
				configure_port_pins_set(pin);
				port_pin_set_output_level(pin, 1);
				
				printf("Pin %d set high\r\n");									

				break;
				
			case "gpio_clear":
				port = a1;
				pinnum = atoi(a2);
				
				if (port == "B") {
					if (pinnum == 2) {
						pin = PIN_PB02;
					}
					if (pinnum == 3) {
						pin = PIN_PB03;
					}
				}
				
				configure_port_pins_set(pin);
				port_pin_set_output_level(pin, 0);
				
				printf("Pin %d cleared\r\n");

				break;
				
			case "gpio_get":
				port = a1;
				pinnum = atoi(a2);
				
				
				if (port == "B") {
					if (pinnum == 2) {
						pin = PIN_PB02;
					}
					if (pinnum == 3) {
						pin = PIN_PB03;
					}
				}
				
				configure_port_pins_get(pin);
				state = port_pin_get_input_level(pin);				
				printf("Current pin state: %d\r\n");
				break;
				
			case "mac":
				printf("00.00.00.00\r\n");
				break;
				
			case "ip":
				printf("255.255.255.255");
				break;
				
			case "read":
				printf("Dummy info");
				
				break;
				
			case "adc_get": 
			
				port = a1;
				pinnum = atoi(a2);
				pin;
								
				if (port == "A") {
					if (pinnum == 2) {
						pin = ADC_POSITIVE_INPUT_PIN0;
					}
				}
				
				configure_adc(pin);
				
				adc_start_conversion(&adc_instance);
				/* Wait for conversion to be done and read out result */
				do {
				} while (adc_read(&adc_instance, &adc_result) == STATUS_BUSY);
				
				printf("Pin %d ADC value: %d\r\n",pin, adc_result);
					
				break; 
				
			case "mcu_temp":
				pin = ADC_POSITIVE_INPUT_TEMP;	
							
				configure_adc(pin);
				
				adc_start_conversion(&adc_instance);
				/* Wait for conversion to be done and read out result */
				do {
				} while (adc_read(&adc_instance, &adc_result) == STATUS_BUSY);
				
				//need to convert ADC value to temperature, replace factor with equation
				int factor = 1;
				int temperature = adc_result * factor;
				
				printf("MCU temperature: %d\r\n",temperature);
			
			
				break;
				
			case "i2c_scan":				
				
				configure_i2c();
				configure_i2c_callbacks();			
	
				for (int slave_address = 0; slave_address < 128; slave_address++){
					enum status_code i2c_status;
					wr_packet.address     = slave_address;
					rd_packet.address	  = slave_address;
					wr_packet.data_length = 1;
					wr_buffer[0]          = 0x05;
					wr_packet.data        = wr_buffer;
					i2c_status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &wr_packet);
					if( i2c_status == STATUS_OK ){ i2c_status = i2c_master_read_packet_wait(&i2c_master_instance, &rd_packet);
						printf("Address found at %#X\r\n", slave_address);
					}
					i2c_master_send_stop(&i2c_master_instance);
				}
			
				break;
				
		default:
			printf("Invalid input. Try help.");
			break;
		
	}
}


void configure_i2c(void)
{
	/* Initialize config structure and software module */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	/* Change buffer timeout to something longer */
	config_i2c_master.buffer_timeout = 65535;
	config_i2c_master.pinmux_pad0       = PINMUX_PA08D_SERCOM2_PAD0;
	config_i2c_master.pinmux_pad1       = PINMUX_PA09D_SERCOM2_PAD1;
	config_i2c_master.generator_source  = GCLK_GENERATOR_0;
	/* Initialize and enable device with config */
	while(i2c_master_init(&i2c_master_instance, CONF_I2C_MASTER_MODULE, &config_i2c_master) != STATUS_OK);
	i2c_master_enable(&i2c_master_instance);
}

void i2c_write_complete_callback(struct i2c_master_module *const module)
{
	/* Initiate new packet read */
	i2c_master_read_packet_job(&i2c_master_instance,&rd_packet);
}

void configure_i2c_callbacks(void)
{
	/* Register callback function. */
	i2c_master_register_callback(&i2c_master_instance, i2c_write_complete_callback,
	I2C_MASTER_CALLBACK_WRITE_COMPLETE);
	i2c_master_enable_callback(&i2c_master_instance,
	I2C_MASTER_CALLBACK_WRITE_COMPLETE);
}

void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 115200;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PA20D_SERCOM3_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PA21D_SERCOM3_PAD3;
	
	//config_usart.baudrate    = 9600;
	//config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	//config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	//config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	//config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	//config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	
	stdio_serial_init(&usart_instance, SERCOM3, &config_usart);
	
	usart_enable(&usart_instance);
}

void usart_read_callback(const struct usart_module *const usart_module)
{
	usart_write_buffer_job(&usart_instance,
	(uint8_t *)rx_buffer, 5);
}

void usart_write_callback(const struct usart_module *const usart_module)
{
	port_pin_toggle_output_level(LED_0_PIN);
}

void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance,
	usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,
	usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}


//! [module_inst]

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

void configure_port_pins_set(int pin)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(pin, &config_port_pin);
}

void configure_port_pins_get(int pin)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(pin, &config_port_pin);
}