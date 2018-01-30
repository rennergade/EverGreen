#include <asf.h>
#include <usart.h>
#include "delay.h"

//defines

#define CONF_I2C_MASTER_MODULE SERCOM2

#define DATA_LENGTH 8



void input_handle(int argc, char **argv);

void configure_usart(void);

void configure_i2c(void);
void i2c_write_complete_callback(struct i2c_master_module *const module);
void configure_i2c_callbacks(void);

void configure_port_pins_set(int pin);
void configure_port_pins_get(int pin);

void configure_adc(int pin);


struct usart_module usart_instance;
struct adc_module adc_instance;
static uint8_t rd_buffer[DATA_LENGTH];
struct i2c_master_module i2c_master_instance;


static uint8_t wr_buffer[DATA_LENGTH] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};
static uint8_t wr_buffer_reversed[DATA_LENGTH] = {
	0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
};

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
