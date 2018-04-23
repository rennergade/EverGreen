/**
 * @file hdc_1080.c
 * @brief Implementation file for HDC1080 sensor
 * @author William Archer
 * @date 22/Apr/2018
 **/

#include "hdc_1080.h"

void configure_i2c_hdc()
{
	struct i2c_master_config config_i2c_master;

	i2c_master_get_config_defaults(&config_i2c_master);
	config_i2c_master.buffer_timeout = I2C_TIMEOUT;
	config_i2c_master.pinmux_pad0 = PINMUX_PA22C_SERCOM3_PAD0;
	config_i2c_master.pinmux_pad1 = PINMUX_PA23C_SERCOM3_PAD1;
	config_i2c_master.generator_source = GCLK_GENERATOR_0;
	enum status_code init_status = i2c_master_init(&i2c_hdc, SERCOM3, &config_i2c_master);
	if (STATUS_OK != init_status) {
		printf("failed to initialize HDC1080 driver.\r\n");
		return;
	}
	i2c_master_enable(&i2c_hdc);
}

//NOTE: Can only set temp resolution to 14 or 11 bit
//NOTE: bit[10] = 0 for 14 bit temp
//NOTE: bit[10] = 1 for 11 bit temp
//NOTE: bit[9:8] = 00 for 14 bit humidity
//NOTE: bit[9:8] = 01 for 11 bit humidity
//NOTE: bit[9:8] = 10 for 8 bit humidity
void set_resolution(hdc_resolution temp_resolution, hdc_resolution humidity_resolution)
{
	int8_t data_packet[3];

	data_packet[0] = HDC_SET_RES;
	uint8_t write_byte = 0;
	switch (temp_resolution) {
	case FOURTEEN_BIT_RESOLUTION: {
		write_byte = 0;
		break;
	}
	case ELEVEN_BIT_RESOLUTION: {
		write_byte = (1 << 2);
		break;
	}
	default: {
		//TODO: error
		return;
	}
	}

	switch (humidity_resolution) {
	case FOURTEEN_BIT_RESOLUTION: {
		write_byte |= 0;
		break;
	}
	case ELEVEN_BIT_RESOLUTION: {
		write_byte |= 1;
		break;
	}
	case EIGHT_BIT_RESOLUTION: {
		write_byte |= 2;
		break;
	}
	default: {
		//TODO: error
		return;
	}
	}
	write_byte |= (1 << 4);
	data_packet[1] = write_byte;
	data_packet[2] = 0x00;
	printf("data packet: 0x%02x 0x%02x 0x%02x\r\n", data_packet[0], data_packet[1], data_packet[2]);
	struct i2c_master_packet packet = {
		.address		= HDC_SLAVE_ADDR,
		.data_length		= 3,
		.data			= data_packet,
		.ten_bit_address	= false,
		.high_speed		= false,
		.hs_master_code		= 0x0,
	};
	enum status_code i2c_status = i2c_master_write_packet_wait(&i2c_hdc, &packet);
	if (STATUS_OK != i2c_status)
		printf("error trying to set resolution!\r\n");

	return;
}

uint16_t request_data(hdc_request command)
{
	uint8_t received_data[2];
	uint8_t write_buffer = command;
	struct i2c_master_packet packet = {
		.address		= HDC_SLAVE_ADDR,
		.data_length		= 1,
		.data			= &write_buffer,
		.ten_bit_address	= false,
		.high_speed		= false,
		.hs_master_code		= 0x0,
	};

	enum status_code i2c_code = i2c_master_write_packet_wait_no_stop(&i2c_hdc, &packet);

	if (STATUS_OK != i2c_code) {
		printf("failed to s	end register request for command 0x%02x!\r\n", command);
		return -1;
	}
	packet.data = received_data;
	packet.data_length = 2;
	delay_ms(10);
	i2c_code = i2c_master_read_packet_wait(&i2c_hdc, &packet);
	if (STATUS_OK != i2c_code) {
		printf("failed to receive bytes for command 0x%02x!\r\n", command);
		return -1;
	}
	return (received_data[0] << 8) | received_data[1];
}

uint16_t get_hdc_manufacturer_id()
{
	return request_data(HDC_MANUFACTURER_ID);
}
uint16_t get_hdc_device_id()
{
	return request_data(HDC_DEVICE_ID);
}
double get_humidity()
{
	double raw_data = request_data(HDC_HUMIDITY);

	raw_data /= 65536.0;
	raw_data *= 100.0;
	return raw_data;
}

double get_temp()
{
	double raw_data = request_data(HDC_TEMP);

	raw_data /= 65536.0;
	raw_data *= 165.0;
	raw_data -= 40.0;
	return raw_data;
}
