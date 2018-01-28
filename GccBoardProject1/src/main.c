
#include "handler.h"


int main (void)
{
	
	system_init();
	system_interrupt_enable_global();
	delay_init();
	
	configure_usart();
	configure_usart_callbacks();
	
	uint8_t string[] = "Welcome to the Evergreen CLI\r\n";
	printf(string);
		
	#define MAX_LEN 50
	
	char* buffer;		
	
	char cmd [20];
	char a1 [10], a2 [10], a3 [10];
	
	while (1) {
		
			printf("EGCLI # ");
			
			while(fgets(buffer, MAX_LEN, stdin) != NULL) {
				if (sscanf(buffer, "%s %s %s %s\n", cmd, a1, a2, a3) == 4) { }
			}

	}
	
	
	return 0;
}
