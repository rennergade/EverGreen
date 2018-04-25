/*
 * TSL2561.c
 *
 * Created: 1/30/2018 2:54:54 PM
 *  Author: William
 */

#include "TSL2561.h"

tsl2561_i2c_addr tsl2561_addr = 0;

void configure_i2c_tsl2561(tsl2561_i2c_addr addr)
{
	struct i2c_master_config config_i2c_master;

	i2c_master_get_config_defaults(&config_i2c_master);
	config_i2c_master.buffer_timeout = I2C_TIMEOUT;
	config_i2c_master.pinmux_pad0 = PINMUX_PA08C_SERCOM0_PAD0;
	config_i2c_master.pinmux_pad1 = PINMUX_PA09C_SERCOM0_PAD1;
	config_i2c_master.generator_source = GCLK_GENERATOR_0;
	enum status_code init_status = i2c_master_init(&i2c_tsl2561, SERCOM0, &config_i2c_master);
	if (STATUS_OK != init_status) {
		printf("failed to initialize TSL2561 driver.\r\n");
		return;
	}
	i2c_master_enable(&i2c_tsl2561);
	tsl2561_addr = addr;
}


uint8_t read_byte(tsl2561_registers reg)
{
	uint8_t received_data;
	uint8_t write_buffer = reg;
	struct i2c_master_packet packet = {
		.address		= tsl2561_addr,
		.data_length		= 1,
		.data			= &write_buffer,
		.ten_bit_address	= false,
		.high_speed		= false,
		.hs_master_code		= 0x0,
	};

	enum status_code i2c_code = i2c_master_write_packet_wait(&i2c_tsl2561, &packet);

	if (STATUS_OK != i2c_code) {
		printf("failed to send register request for command 0x%02x!\r\n", write_buffer);
		return -1;
	}
	packet.data = &received_data;
	packet.data_length = 1;
	i2c_code = i2c_master_read_packet_wait(&i2c_tsl2561, &packet);
	if (STATUS_OK != i2c_code) {
		printf("failed to receive bytes for command 0x%02x!\r\n", write_buffer);
		return -1;
	}
	return received_data;
}

void write_byte(tsl2561_registers reg, uint8_t data_byte)
{
	uint8_t write_buffer[2] = { reg, data_byte };
	struct i2c_master_packet packet = {
		.address		= tsl2561_addr,
		.data_length		= 2,
		.data			= &write_buffer,
		.ten_bit_address	= false,
		.high_speed		= false,
		.hs_master_code		= 0x0,
	};

	enum status_code i2c_code = i2c_master_write_packet_wait_no_stop(&i2c_tsl2561, &packet);

	if (STATUS_OK != i2c_code) {
		printf("failed to send register request for command 0x%02x!\r\n", write_buffer[1]);
		return;
	}
}

/**
 * sets the gain of the light sensor
 *
 * For now the integration time will always be 402ms. Will update once decide
 * if we need faster readings
 * @param new_gain new gain multiplier
 */
void set_gain(tsl2561_gains new_gain)
{
	write_byte(CMD_BIT | TIMING_REG, (INTEGRATE_402MS | new_gain));
}

void power_on_tsl2561()
{
	write_byte(CMD_BIT | CTRL_REG, 0x03);
	set_gain(GAIN_16X);
}

void power_off_tsl2561()
{
	write_byte(CMD_BIT | CTRL_REG, 0x0);
}

//SHOULD RETURN 0x50
uint8_t get_tsl2561_device_id()
{
	return read_byte(CMD_BIT | ID_REG);
}

uint16_t get_chan0()
{
	//TODO: add way to change this
	delay_ms(443);
	//HAVE TO READ LOW REGISTER BEFORE HIGH REG PER DATASHEET
	return read_byte(CMD_BIT | CHAN0_LOW_REG) | (read_byte(CMD_BIT | CHAN0_HIGH_REG) << 8);
}

uint16_t get_chan1()
{
	//TODO: add way to change this
	delay_ms(443);
	return read_byte(CMD_BIT | CHAN1_LOW_REG) | (read_byte(CMD_BIT | CHAN1_HIGH_REG) << 8);
}

uint32_t get_lux()
{
	uint16_t chan0 = get_chan0();
	uint16_t chan1 = get_chan1();

	uint16_t saturation_test = 0xffff; // can't guarantee accuracy if reg is filled
	if (chan0 > saturation_test || (chan1 > saturation_test)) {
		printf("get_lux: TSL2561 light is saturated");
		return -1;
	}

	unsigned long ratio10 = 0;
	if(chan0 != 0) {
		 ratio10 = (chan1 << (TSL2561_LUX_RATIOSCALE + 1)) / chan0;
	}
	ratio10 = (ratio10 + 1) >> 1;
	unsigned int b, m;

	if ((ratio10 >= 0) && (ratio10 <= TSL2561_LUX_K1T)) {
		b = TSL2561_LUX_B1T; m = TSL2561_LUX_M1T;
	} else if (ratio10 <= TSL2561_LUX_K2T) {
		b = TSL2561_LUX_B2T; m = TSL2561_LUX_M2T;
	} else if (ratio10 <= TSL2561_LUX_K3T) {
		b = TSL2561_LUX_B3T; m = TSL2561_LUX_M3T;
	} else if (ratio10 <= TSL2561_LUX_K4T) {
		b = TSL2561_LUX_B4T; m = TSL2561_LUX_M4T;
	} else if (ratio10 <= TSL2561_LUX_K5T) {
		b = TSL2561_LUX_B5T; m = TSL2561_LUX_M5T;
	} else if (ratio10 <= TSL2561_LUX_K6T) {
		b = TSL2561_LUX_B6T; m = TSL2561_LUX_M6T;
	} else if (ratio10 <= TSL2561_LUX_K7T) {
		b = TSL2561_LUX_B7T; m = TSL2561_LUX_M7T;
	} else if (ratio10 > TSL2561_LUX_K8T) {
		b = TSL2561_LUX_B8T; m = TSL2561_LUX_M8T;
	}
	
	
	unsigned long temp = ((chan0 * b) - (chan1 * m));
	temp = (temp) ? temp : 0; // no negative lux
	temp += (1 << (TSL2561_LUX_LUXSCALE - 1));
	uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;
	return lux;
}
