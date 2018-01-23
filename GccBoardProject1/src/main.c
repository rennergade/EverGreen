/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * This is a bare minimum user application template.
 *
 * For documentation of the board, go \ref group_common_boards "here" for a link
 * to the board-specific documentation.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# Basic usage of on-board LED and button
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <usart.h>

struct usart_module usart_instance;
volatile uint8_t rx_buffer[5];

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


int main (void)
{
	system_init();
	system_interrupt_enable_global();
	configure_usart();
	configure_usart_callbacks();
	
	
	
	uint8_t string[] = "Hello World!\r\n";
	printf(string);
	//usart_write_buffer_wait(&usart_instance, string, sizeof(string));
	return 0;
}
