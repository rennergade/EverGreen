#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "handler.h"
#include "MQTTeg.h"
#include "TSL2561/TSL2561.h"
#include "HDC1080/hdc1080.h"


//TODO: bug with backspace doesn't allow whitespace etc

#define COUNTER_MAX 42000 //1 minute in cycles
#define SUCCESS              0
#define FAILURE              1


extern volatile int wifi_connected;
extern volatile int mqtt_connected;

extern char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];



#define CR '\r'                                                 /// Carriage Return
#define LF '\n'                                                 /// Line feed
#define BS 127                                                 /// backspace
#define NULLCHAR '\0'                                           /// null character char
#define SPACE ' '                                               /// ascii space
#define MAX_RX_BUFFER_LENGTH   100                              /// max buffer length for rx reads
#define MAX_ARG_LENGTH (MAX_RX_BUFFER_LENGTH / MAX_ARGS)        /// max arg length
#define MAX_ARGS 4                                              /// max args is 4 for read

volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];               /// rx buffer
uint8_t numberCharsRead;                                        /// number of chars read
uint8_t argc;                                                   /// number of arguments
char *argv[MAX_ARGS];                                           /// arguments stored in buffer

/**
 * stores user input in rx buffer read from serial
 *
 * @return  true if successful, false if error
 */
bool processUserInput(void)
{
	char singleCharInput;
	volatile enum status_code uartReadCode = usart_read_buffer_wait(&usart_instance, &singleCharInput, 1);

	if (STATUS_OK != uartReadCode) {
		return false;
	}
	if((singleCharInput != BS && !numberCharsRead) || numberCharsRead) {
		if (STATUS_OK == uartReadCode) {
			volatile enum status_code uartWriteCode = usart_write_buffer_wait(&usart_instance, &singleCharInput, 1);
		}
	}
	

	switch (singleCharInput) {
	case CR:
	case LF: {
		/// On carriage return (CR) or line feed (LF), the user has hit enter and it's time to process their command.
		/// Remember to null terminate your strings!  Otherwise, you could keep reading throughout memory.
		rx_buffer[numberCharsRead] = NULLCHAR;
		if (numberCharsRead > 0) {
			numberCharsRead = 0;
			return true;
		}
		break;
	}
	case BS: {
		/// User input a backspace -- remove the character
		if(!numberCharsRead) //no characters written
			break;
		numberCharsRead--;
		rx_buffer[numberCharsRead] = NULLCHAR;

		/// Feeling cheeky?  Do it all in one line
		// rx_buffer[--numberCharsRead] = NULLCHAR;
		break;
	}
	default: {
		/// All other cases
		if (numberCharsRead < MAX_RX_BUFFER_LENGTH)
			rx_buffer[numberCharsRead++] = singleCharInput;
		rx_buffer[numberCharsRead] = NULLCHAR;  ///< String read protection
		break;
	}
	}
	return false;
}

/**
 * helper function to set string to all lowercase
 * @param str string to make all lowercase
 */
void make_lowercase(char **str)
{
	int i = 0;

	while (*(*(str) + i) != 0) {
		*(*(str) + i) = tolower(*(*str + i));
		i++;
	}
}

/**
 * fixes arguments and adds them to the argv buffer
 */
void fix_args()
{
	char *p = strtok(rx_buffer, " "); /// NOTE: strtok destroys the input string

	while (p != NULL) {
		make_lowercase(&p);
		strcpy(argv[argc++], p);
		p = strtok(NULL, " ");
	}
}


int main(void)
{
	uint32_t MQTTCounter = 0;
	
	int wifi_result = SUCCESS;
	    
	uint8_t mqtt_send_buffer[MQTT_SEND_BUFFER_SIZE];


	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_usart();
	
	configure_adc(MOISTURE_ANA_PIN); //configure moisture sensor analog
	configure_i2c_temp(); //config i2c
	configure_i2c_lux();
	configure_i2c_callbacks_hdc();
	configure_i2c_callbacks_tsl();
	
	
		
	for (int i = 0; i < MAX_ARGS; i++)
		argv[i] = malloc(sizeof(char) * MAX_ARG_LENGTH);

	//TODO: print version information
	printf("\r\nWelcome to the Evergreen CLI.\r\n");
	printf("> ");
	while (1) {
 			bool commandEntered = processUserInput();
 			if (commandEntered) {
				printf("> %s\r\n", rx_buffer);
 				fix_args();
 				input_handle(argc, argv); //fix
 				argc = 0;
 				printf("> ");
 			}
 		}

	for (int i = 0; i < MAX_ARGS; i++)
		free(argv[i]);


	return 0;
}


