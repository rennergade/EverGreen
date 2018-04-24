#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "handler.h"
#include "MQTTeg.h"
#include "TSL2561.h"
#include "hdc_1080.h"
#include "wifi.h"


#define COUNTER_MAX 3000000 //1 minute in cycles
#define WIFI_SUCCESS              0
#define WIFI_FAILURE              1


extern volatile int wifi_connected;
extern volatile int mqtt_connected;

extern char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];


int main(void)
{
	uint32_t MQTTCounter = 0;	
	int wifi_result = WIFI_SUCCESS;	    
	
	mqttfirmware_download = 0;

	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_usart();
	wifi_result = wifi_init();
	
	if (WIFI_SUCCESS != wifi_result) printf("\r\n...Wi-Fi failed to configure...\r\n");
	
	printf("Board initialized.\r\n");

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
	MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
	
	while(!(wifi_connected)) {		
		    /* Handle pending events from network controller. */
		    m2m_wifi_handle_events(NULL);
		    /* Checks the timer timeout. */
		    sw_timer_task(&swt_module_inst);
	}
	
	while(!(mqtt_connected)) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		sw_timer_task(&swt_module_inst);
	}
	
	
	
	configure_adc(MOISTURE_ANA_PIN); //configure moisture sensor analog
	configure_i2c_hdc(); //config i2c
	configure_i2c_tsl2561(ADDR_FLOAT);
	uint16_t dev_id = get_hdc_device_id();
	printf("dev id: 0x%02x\r\n", dev_id);
	
	configure_port_pins_get(PIN_PA11); //TODO: decide if this is necessary

	printf("Running as MQTT User: %s\r\n", MQTT_USER);
	
	led1_on(); //show that board is connected	

	while (1) {
	
	    /* Handle pending events from network controller. */
	    m2m_wifi_handle_events(NULL);
	    /* Checks the timer timeout. */
	    sw_timer_task(&swt_module_inst);		
		
		if ((mqttfirmware_download == 1 || port_pin_get_input_level(PIN_PA11) == false))
		{
			deconfigure_mqtt();
			m2m_wifi_deinit(0);
			nm_bsp_deinit();
			
			wifi_config new_wifi_configuration;
			get_default_wifi_config(&new_wifi_configuration);
			new_wifi_configuration.ssid = "SNBP";
			new_wifi_configuration.password = "sn42betarho";
			configure_wifi_module(&new_wifi_configuration);
			configure_flash();
			configure_nvm();
			if(check_for_update()) {
				if(download_firmware()) {
					printf("firmware successfully downloaded!\r\n");
				}
			}			
			mqttfirmware_download = 0;
		}
		
		else if ((MQTTCounter >= COUNTER_MAX))
		{
			MQTTCounter = 0; //reset sensor counter
			printf("Sending sensor values to Cloud.\r\n");			
			publish_sensor_values();			
		}	
				
		++MQTTCounter;
	}

	return 0;
}


