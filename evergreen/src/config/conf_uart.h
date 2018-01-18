/*
 * conf_uart.c
 *
 * Created: 1/18/2018 1:15:26 PM
 *  Author: William
 */ 

#define USART_SERIAL                     &USARTD0
#define USART_SERIAL_BAUDRATE            115200
#define USART_SERIAL_CHAR_LENGTH         8
#define USART_SERIAL_PARITY              false
#define USART_SERIAL_STOP_BIT            1
#define USART_SERIAL_RX_PIN				 "PA21"
#define USART_SERIAL_TX_PIN				 "PA20" //TODO: find where these pins are defined
