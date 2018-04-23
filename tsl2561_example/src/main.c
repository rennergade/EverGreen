/**
 * @file main.c
 * @brief Example using TSL2561 driver
 * @author William Archer
 * @date 23/Apr/2018
 *
 */

#include <asf.h>
#include "component-configurations.h"
#include "TSL2561.h"

struct usart_module usart_instance;

int main (void)
{
	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_usart();
	printf("---- TSL2561 Driver Example ---\r\n");
	
	configure_i2c_tsl2561(ADDR_FLOAT);
	printf("Lux Device ID: 0x%02x\r\n", get_tsl2561_device_id());
	power_on_tsl2561();
	printf("Current Lux: %d\r\n", get_lux());
	return EXIT_SUCCESS;

	/* Insert application code here, after the board has been initialized. */
}
