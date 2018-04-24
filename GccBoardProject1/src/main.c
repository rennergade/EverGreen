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
/**
 * MQTT broker has requested LED turn on or off
 * @param on_off 1 if turn on 0 if turn off
 */
void request_led_change(uint8_t on_off)
{
	printf("led change requested w value: %d\r\n", on_off);
}
/**
 * MQTT broker has requested pump turn on or off
 * @param on_off 1 if turn on 0 if turn off
 */
void request_pump_change(uint8_t on_off)
{
	printf("pump change requested w value: %d\r\n", on_off);
}

/**
 * MQTT broker has requested a relay change
 * @param on_off lsb 1 if turn on 0 if turn off, second lsb relay 1 or 2 (0 or 1)
 */
void request_relay_change(uint8_t on_off)
{
	uint8_t which_relay = on_off & 0b10;
	uint8_t on_or_off = on_off & 1;
	printf("relay change requested w value: %d\r\n", on_off);
}

void request_firmware_download()
{
	printf("firmware download requested\r\n");
	deconfigure_mqtt();

	wifi_config new_wifi_configuration;
	get_default_wifi_config(&new_wifi_configuration);
	new_wifi_configuration.ssid = "SNBP";
	new_wifi_configuration.password = "sn42betarho";
	configure_wifi_module(&new_wifi_configuration);
	configure_flash();
	configure_nvm();
	if (check_for_update()) {
		if (download_firmware()) {
			printf("firmware successfully downloaded!\r\n");
		} else {
			printf("firmware unsuccessful. Check log for more details.\r\n");
		}
	} else {
		printf("No update found.\r\n");
	}
}


int main(void)
{
	uint32_t MQTTCounter = 0;

	mqttfirmware_download = 0;

	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_usart();
	mqtt_inst_config new_mqtt;
	get_mqtt_config_defaults(&new_mqtt);
	int mqtt_result = mqtt_initialize(&new_mqtt);
	
	if (WIFI_SUCCESS != mqtt_result) printf("\r\n...Wi-Fi failed to configure...\r\n");

	printf("Board initialized.\r\n");

	/* Connect to router. */
	m2m_wifi_connect(new_mqtt.ssid, strlen(new_mqtt.ssid), new_mqtt.auth, new_mqtt.password, M2M_WIFI_CH_ALL);

	while (!(wifi_connected)) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		//sw_timer_task(&swt_module_inst);
	}

	while (!(mqtt_connected)) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		//sw_timer_task(&swt_module_inst);
	}



	configure_adc(MOISTURE_ANA_PIN);        //configure moisture sensor analog
	configure_i2c_hdc();                    //config i2c
	configure_i2c_tsl2561(ADDR_FLOAT);
	uint16_t dev_id = get_hdc_device_id();
	printf("dev id: 0x%02x\r\n", dev_id);

	configure_port_pins_get(PIN_PA11); //TODO: decide if this is necessary

	printf("Running as MQTT User: %s\r\n", MQTT_USER);
	register_request_topic(LED_TOPIC, '#', &request_led_change);

	led1_on(); //show that board is connected

	while (1) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		//sw_timer_task(&swt_module_inst);

		if ((mqttfirmware_download == 1 || port_pin_get_input_level(PIN_PA11) == false)) {
		} else if ((MQTTCounter >= COUNTER_MAX)) {
			MQTTCounter = 0; //reset sensor counter
			printf("Sending sensor values to Cloud.\r\n");
			//publish_sensor_values();
		}

		++MQTTCounter;
	}

	return 0;
}
