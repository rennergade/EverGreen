/*
 * hdc1080.c
 *
 * Created: 4/18/2018 10:19:28 PM
 *  Author: rennergade
 */ 

#include "hdc1080.h"
#include "../handler.h"


/*
 * hdc1080_read_reg() - Read User register
 * @delay: Delay before read
 * @reg: Register address
 * @val: 16-bit register value from the hdc1080
 * Returns status or error for invalid parameters.
 */
int hdc1080_read_reg(uint16_t delay, uint8_t reg, uint16_t *val)
{
	
	int error = STAT_OK;
	uint8_t write_buffer[2];

	// Check argument
	if ((reg != HDC1080_TEMPERATURE) &
		  (reg != HDC1080_HUMIDITY) &
		  (reg != HDC1080_CONFIG)){
		return STAT_ERR;
		printf("HDC ERROR Invalid register\r\n");
		}
	
	write_buffer[0] = reg;
	/* Read register */
	/* Send the read followed by address */
	
	wr_packet.address = HDC1080_ADDR;
	wr_packet.data_length = 2;
	wr_packet.data = write_buffer;
	int timeout = 0;
	int max_timeout = 1000;

	while (i2c_master_write_packet_wait(&i2c_hdc_instance, &wr_packet) !=
	STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == max_timeout) {
			error = STAT_ERR;
			printf("HDC WRITE TIMEOUT\r\n");
			break;
		}
	}
	if (error != STAT_OK)
		return error;

	delay_ms(delay); 
	
	/* Receive a 2-byte result */
	
	uint8_t read_buffer[3];
	read_buffer[0] = reg;
	rd_packet.address = HDC1080_ADDR;
	rd_packet.data_length = 3;
	rd_packet.data = read_buffer;
	timeout = 0;
	max_timeout = 1000;
	while (i2c_master_read_packet_wait(&i2c_hdc_instance, &rd_packet) !=
	STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == max_timeout) {
			printf("HDC READ TIMEOUT\r\n");
			error = STAT_ERR;
			break;
		}
	}
	
	
	if (error != STAT_OK)
		return error;
	
	/* Result */
	*val = read_buffer[0]*256+read_buffer[1]; 

	return STAT_OK;  /* Success */
	
}


/*
 * hdc1080_write_reg() - Write register
 * @reg: Register address
 * @val: 8-bit register value from the Si7013
 * Returns  status or error for invalid arguments.
 */
int hdc1080_write_reg(uint8_t reg, uint16_t val)
{
	uint8_t write_buffer[3];
	int error = STAT_OK;
	
		// Check argument
	if ((reg != HDC1080_TEMPERATURE) &  // dummy write to adr 0 ... trigger measurement
		  (reg != HDC1080_CONFIG) )       // config is "writable"
		return STAT_ERR;

	write_buffer[0] = reg;
	write_buffer[1] = (uint8_t)((val >> 8) & 0xff);  // msb
	write_buffer[2] = (uint8_t)(val & 0xff); 				// lsb
	/* Write the register */
	/* Send the command and data */
	
	wr_packet.address = HDC1080_ADDR;
	wr_packet.data_length = 2;
	wr_packet.data = write_buffer;
	int timeout = 0;
	int max_timeout = 1000;
	
	while (i2c_master_write_packet_wait(&i2c_hdc_instance, &wr_packet) !=
	STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == max_timeout) {
			printf("HDC WRITE TIMEOUT\r\n");
			error = STAT_ERR;
			break;
		}
	}
	if (error != STAT_OK)
		return error;
  else 
	  return STAT_OK;  /* Success */
}




/*
 * hdc1080_measure() - measure humididty and temperature: 

1. Configure the acquisition parameters in config register (address 0x02):
		(a) Set the acquisition mode to measure both temperature
				and humidity by setting Bit[12] to 1.
		(b) Set the desired temperature measurement resolution:
				?  Set Bit[10] to 0 for 14 bit resolution.
				?  Set Bit[10] to 1 for 11 bit resolution.
		(c) Set the desired humidity measurement resolution:
				?  Set Bit[9:8] to 00 for 14 bit resolution.
				?  Set Bit[9:8] to 01 for 11 bit resolution.
				?  Set Bit[9:8] to 10 for 8 bit resolution.

2. Trigger the measurements by writing I2C read reg. address with adr = HDC1080_TEMPERATURE.

3. Wait for the measurements to complete, based on the conversion time

4. Read the output data:
Read the temperature data from register address 0x00, followed by the humidity 
data from register address 0x01 in a single transaction. A read operation will 
return a NACK if the contents of the registers have not been updated.

 * @hi2c:  handle to I2C interface
 * @temp_res    :  temperature measurement resolution:
 *										- HDC1080_T_RES_14 or
 *                    - HDC1080_T_RES_11
 * @humidres    :  humidity readout resolution
 *                    - HDC1080_RH_RES_14 or
 *										- HDC1080_RH_RES_11 or
 *										- HDC1080_RH_RES_8 
 * @heater      :  heater enable (0 = disabled or 1 = enabled)
 * @bat_stat    :  supply voltage 
 *                    - 0 when Ucc > 2,8V
 *										- 1 when Ucc < 2,8V
 * @temperature :  floating point temperature result, unit is ?C
 * @humidity    :  floating point humidity result, unit is RH%
 * Returns status.
 */
int hdc1080_measure(double *temperature, double *humidity)
{

	int error = STAT_OK;
	
	uint16_t r;
	double tmp;
	
	uint8_t temp_res = HDC1080_T_RES_14;
	uint8_t humidres = HDC1080_RH_RES_14;
	uint8_t heater = 0; //heater off
	
	
	error = hdc1080_read_reg(10, HDC1080_CONFIG, &r);
	if (error != STAT_OK) {
		printf("HDC Read Config Error\r\n");
		return error;
	}

	r |= temp_res<<10;
	r |= humidres<<8;
	r |= 1<<12;     // mode = 1;
	r |= heater<<13;
	
	
	// write config
	error = hdc1080_write_reg(HDC1080_CONFIG, r);
	if (error != STAT_OK) {
		printf("HDC Write Config Error\r\n");
		return error;
	}
	

	error = hdc1080_read_reg(150, HDC1080_TEMPERATURE, &r);
	if (error != STAT_OK) {
		printf("HDC Read Temp Error\r\n");
		return error;
	}
	tmp = (double)r;
	tmp = (tmp / 65536.0f) * 165.0f - 40.0f;
	*temperature = tmp;  // C
	
	error = hdc1080_read_reg(150, HDC1080_HUMIDITY, &r);
	tmp = (int32_t)r;
	
	if (error != STAT_OK){
		printf("HDC Read Humidity Error\r\n");
		 return error;
	}
	tmp = (float)r;
	tmp = (tmp / 65536.0f) * 100.0f;
	if (tmp>100.0) tmp = 100.0f;
	if (tmp<0) tmp = 0.0f;
	*humidity = tmp;
	
	return STAT_OK;
}
