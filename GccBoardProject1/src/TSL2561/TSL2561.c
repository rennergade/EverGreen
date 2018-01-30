/*
 * TSL2561.c
 *
 * Created: 1/30/2018 2:54:54 PM
 *  Author: William
 */ 

#include "TSL2561.h"
#include "../handler.h"

int is_init = false;


void write8(uint8_t reg, uint8_t value)
{
	if(is_init) {
		uint8_t write_buffer[2] = {reg, value};
		wr_packet.address = TSL2561_ADDR_FLOAT;
		wr_packet.data_length = 2;
		wr_packet.data = write_buffer;
		int timeout = 0;
		int max_timeout = 1000;
		
		while (i2c_master_write_packet_wait(&i2c_master_instance, &wr_packet) !=
		STATUS_OK) {
			/* Increment timeout counter and check if timed out. */
			if (timeout++ == max_timeout) {
				break;
			}
		}
		
	}
}

void enable(void)
{
	/* Enable the device by setting the control bit to 0x03 */
	write8((TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL), TSL2561_CONTROL_POWERON);
}

void disable(void)
{
	/* Turn the device off to save power */
	write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
}

uint8_t read8_reg(uint8_t reg) {
	if(is_init) {
		enable();
		uint8_t read_buffer[2];
		read_buffer[0] = reg;
		rd_packet.address = TSL2561_ADDR_FLOAT;
		rd_packet.data_length = 2;
		rd_packet.data = read_buffer;
		int timeout = 0;
		int max_timeout = 1000;
		while (i2c_master_read_packet_wait(&i2c_master_instance, &rd_packet) !=
		STATUS_OK) {
			/* Increment timeout counter and check if timed out. */
			if (timeout++ == max_timeout) {
				break;
			}
		}
		
		disable();
		return read_buffer[1];
	}
	return 0;
}

uint16_t read16_reg(uint8_t reg) {
	if(is_init) {
		enable();
		uint8_t read_buffer[3];
		read_buffer[0] = reg;
		rd_packet.address = TSL2561_ADDR_FLOAT;
		rd_packet.data_length = 3;
		rd_packet.data = read_buffer;
		int timeout = 0;
		int max_timeout = 1000;
		while (i2c_master_read_packet_wait(&i2c_master_instance, &rd_packet) !=
		STATUS_OK) {
			/* Increment timeout counter and check if timed out. */
			if (timeout++ == max_timeout) {
				break;
			}
		}
		uint8_t t = read_buffer[1];
		uint8_t x = read_buffer[2];
		x <<= 8;
		x |= t;
		disable();
		return x;
	}
	return 0;
}

int tsl2561_init() {
	if(is_init) {
		return 1;
	}
	configure_i2c();
	is_init = true;
	  uint8_t x = read8_reg(TSL2561_REGISTER_ID);
	  if (x & 0xF0 != 0x10) { // ID code for TSL2561
		  return 0;
	  }
	  return 1;
	
}

int calculateLux(uint16_t broadband, uint16_t ir) {
	unsigned long chScale;
	unsigned long channel1;
	unsigned long channel0;

	/* Make sure the sensor isn't saturated! */
	uint16_t clipThreshold;

	/* Return 65536 lux if the sensor is saturated */
	if ((broadband > clipThreshold) || (ir > clipThreshold))
	{
		return 65536;
	}

	/* Scale the channel values */
	channel0 = (broadband * chScale) >> TSL2561_LUX_CHSCALE;
	channel1 = (ir * chScale) >> TSL2561_LUX_CHSCALE;

	/* Find the ratio of the channel values (Channel1/Channel0) */
	unsigned long ratio1 = 0;
	if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

	/* round the ratio value */
	unsigned long ratio = (ratio1 + 1) >> 1;

	unsigned int b, m;

	#ifdef TSL2561_PACKAGE_CS
	if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C))
	{b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
	else if (ratio <= TSL2561_LUX_K2C)
	{b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
	else if (ratio <= TSL2561_LUX_K3C)
	{b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
	else if (ratio <= TSL2561_LUX_K4C)
	{b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
	else if (ratio <= TSL2561_LUX_K5C)
	{b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
	else if (ratio <= TSL2561_LUX_K6C)
	{b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
	else if (ratio <= TSL2561_LUX_K7C)
	{b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
	else if (ratio > TSL2561_LUX_K8C)
	{b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
	#else
	if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
	{b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
	else if (ratio <= TSL2561_LUX_K2T)
	{b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
	else if (ratio <= TSL2561_LUX_K3T)
	{b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
	else if (ratio <= TSL2561_LUX_K4T)
	{b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
	else if (ratio <= TSL2561_LUX_K5T)
	{b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
	else if (ratio <= TSL2561_LUX_K6T)
	{b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
	else if (ratio <= TSL2561_LUX_K7T)
	{b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
	else if (ratio > TSL2561_LUX_K8T)
	{b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
	#endif

	unsigned long temp;
	temp = ((channel0 * b) - (channel1 * m));

	/* Do not allow negative lux value */
	if (temp < 0) temp = 0;

	/* Round lsb (2^(LUX_SCALE-1)) */
	temp += (1 << (TSL2561_LUX_LUXSCALE-1));

	/* Strip off fractional portion */
	uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

	/* Signal I2C had no errors */
	return lux;
}

int getLuminosity() {
	/* Reads a two byte value from channel 0 (visible + infrared) */
	uint16_t broadband = read16_reg((TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW));

	/* Reads a two byte value from channel 1 (infrared) */
	uint16_t ir = read16_reg((TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW));
	
	return calculateLux(broadband, ir);
}
