#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "handler.h"

//TODO: bug with backspace doesn't allow whitespace etc


#define CR '\r'                                                 /// Carriage Return
#define LF '\n'                                                 /// Line feed
#define BS '\b'                                                 /// backspace
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
	if (STATUS_OK == uartReadCode) {
		volatile enum status_code uartWriteCode = usart_write_buffer_wait(&usart_instance, &singleCharInput, 1);
	}

	switch (singleCharInput) {
	case CR:
	case LF:
		/// On carriage return (CR) or line feed (LF), the user has hit enter and it's time to process their command.
		/// Remember to null terminate your strings!  Otherwise, you could keep reading throughout memory.
		rx_buffer[numberCharsRead] = NULLCHAR;
		if (numberCharsRead > 0) {
			numberCharsRead = 0;
			return true;
		}
		break;

	case BS:
		/// User input a backspace -- remove the character
		if(!numberCharsRead) //no characters written
			break;
		numberCharsRead--;
		rx_buffer[numberCharsRead] = NULLCHAR;

		/// Feeling cheeky?  Do it all in one line
		// rx_buffer[--numberCharsRead] = NULLCHAR;
		break;

	default:
		/// All other cases
		if (numberCharsRead < MAX_RX_BUFFER_LENGTH)
			rx_buffer[numberCharsRead++] = singleCharInput;
		rx_buffer[numberCharsRead] = NULLCHAR;  ///< String read protection
		break;
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
	system_init();
	system_interrupt_enable_global();
	delay_init();

	configure_usart();
	
	for (int i = 0; i < MAX_ARGS; i++)
		argv[i] = malloc(sizeof(char) * MAX_ARG_LENGTH);

	//TODO: print version information
	printf("Welcome to the Evergreen CLI.\r\n");
	printf("> ");
	while (1) {
		bool commandEntered = processUserInput();
		if (commandEntered) {
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
