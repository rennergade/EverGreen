/**
 * @file main.c
 * @brief Example using HDC 1080 driver for Evergreen board.
 * @author William Archer
 * @date 22/Apr/2018
 *
 */

#include <asf.h>
#include "component-configurations.h"
#include "hdc_1080.h"


struct usart_module usart_instance;

int main (void)
{
	//HDC1080 SDA: PA22
	//HDC1080 SCL: PA23
	//HDC1080 Sercom 3/5
	
	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_usart();
	printf("----- HDC1080 SENSOR EXAMPLE _------\r\n");
	
	configure_i2c_hdc();
	set_resolution(ELEVEN_BIT_RESOLUTION, FOURTEEN_BIT_RESOLUTION);
	printf("Successfully set resolution!\r\n");
	get_device_id();
	get_manufacturer_id();
	double humidity = get_humidity();
	printf("humidity: %3.2f\r\n", humidity);
	double temp = get_temp();
	printf("temp: %3.2f\r\n", temp);
	printf("serial_begin: %d\r\n", request_data(HDC_SERIAL_ID_FIRST));
	printf("serial_mid: %d\r\n", request_data(HDC_SERIAL_ID_MID));
	printf("serial_end: %d\r\n", request_data(HDC_SERIAL_ID_LAST));

	return EXIT_SUCCESS;
}
