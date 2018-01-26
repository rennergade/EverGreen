#include <asf.h>
#include <usart.h>

struct usart_module usart_instance;
volatile uint8_t rx_buffer[5];

#define CONF_I2C_MASTER_MODULE SERCOM2

#define DATA_LENGTH 8

static uint8_t wr_buffer[DATA_LENGTH] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};
static uint8_t wr_buffer_reversed[DATA_LENGTH] = {
	0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
};
static uint8_t rd_buffer[DATA_LENGTH];

struct i2c_master_module i2c_master_instance;

static struct i2c_master_packet wr_packet = {
	.address          = 0,
	.data_length      = DATA_LENGTH,
	.data             = wr_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};

static struct i2c_master_packet rd_packet = {
	.address          = 0,
	.data_length      = DATA_LENGTH,
	.data             = rd_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};


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

void configure_adc(void);

//! [module_inst]
struct adc_module adc_instance;

void configure_adc(void)
{
	struct adc_config config_adc;

	adc_get_config_defaults(&config_adc);
	config_adc.reference = ADC_REFERENCE_INTVCC0;
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
}

void configure_port_pins_set(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PB02, &config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PB03, &config_port_pin);
}

void configure_port_pins_get(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PB02, &config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PB03, &config_port_pin);
}



int main (void)
{
	
	system_init();
	system_interrupt_enable_global();
	
	configure_usart();
	configure_usart_callbacks();
	
	
	configure_adc();
	
	configure_i2c();
	configure_i2c_callbacks();
	
	uint8_t string[] = "Hello World!\r\n";
	printf(string);
	
	printf("ADC: \r\n");
	adc_start_conversion(&adc_instance);
	
	uint16_t result;

	do {
		/* Wait for conversion to be done and read out result */
	} while (adc_read(&adc_instance, &result) == STATUS_BUSY);
	
	printf("ADC value: %d\r\n",result);
	
	printf("I2C: \r\n");
	
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
	
	printf("GPIO: \r\n");
	
	bool pin_state;
	
	printf("Get: \r\n");
	configure_port_pins_set();
	pin_state = port_pin_get_input_level(PIN_PB02);
	pin_state = port_pin_get_input_level(PIN_PB03);
	
	printf("Set: \r\n");
	port_pin_set_output_level(PIN_PB02, 0);
	port_pin_set_output_level(PIN_PB03, 0);



	printf("Clear: \r\n");
	port_pin_set_output_level(PIN_PB02, 0);
	port_pin_set_output_level(PIN_PB03, 1);


	
	    
	while (1) {
// 		uint8_t buffer[50];
// 		scanf("%s", buffer);
// 		
// 		if (buffer) {
// 			printf("got the following: %s\r\n", buffer);
// 		}


	}
	
	
	//usart_write_buffer_wait(&usart_instance, string, sizeof(string));
	return 0;
}
