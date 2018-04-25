/**
 * \file
 *
 * \brief Empty user application template
 *
 */
#include <asf.h>
#include "hdc_1080.h"
#include "TSL2561.h"
#include "otafu.h"
#include "mqtt.h"
#include "component-configurations.h"

#define FIRMWARE_VERSION 0

#define COUNTER_MAX 40000 //1 minute in cycles
#define WIFI_SUCCESS              0
#define WIFI_FAILURE              1

extern volatile int wifi_connected;
extern volatile int mqtt_connected;

struct usart_module usart_instance;

struct adc_module adc_instance;



/**
 * Sets a given port to output.
 * @param pin pin of the port to set
 */
void configure_port_pins_set(int pin)
{
	struct port_config config_port_pin;

	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(pin, &config_port_pin);
}

void configure_adc(int pin)
{
	struct adc_config config_adc;

	adc_get_config_defaults(&config_adc);
	config_adc.positive_input = pin;
	config_adc.reference = ADC_REFERENCE_INTVCC0;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
}

float get_moisture(void)
{
	// turn on sensor
	configure_port_pins_set(PIN_PA17);
	port_pin_set_output_level(PIN_PA17, true);
	
	delay_ms(500);
	
	// read ADC
	uint16_t adc_result;
	adc_start_conversion(&adc_instance);
	/* Wait for conversion to be done and read out result */
	do {
	} while (adc_read(&adc_instance, &adc_result) == STATUS_BUSY);
	float moisture = (adc_result/4095.0f)*(100.0f);
	
	//turn off sensor
	port_pin_set_output_level(PIN_PA17, false);
	

	return moisture;	
}


void led_request_callback(uint8_t options) {
		configure_port_pins_set(PIN_PA03);
		port_pin_set_output_level(PIN_PA03, options);
}

void pump_request_callback(uint32_t options)
{
	configure_port_pins_set(PIN_PB23);
	port_pin_set_output_level(PIN_PB23, true);
	delay_ms(options);
	configure_port_pins_set(PIN_PB23);
	port_pin_set_output_level(PIN_PB23, false);
}

void relay_request_callback(uint32_t options)
{
	uint8_t which_relay = options & 0b10;
	uint8_t on_or_off = options & 1;
	
	
	uint32_t relay = (which_relay) ? PIN_PB03 : PIN_PB02;
	configure_port_pins_set(relay);
	port_pin_set_output_level(relay, on_or_off);
}

void firmware_request_download_callback(uint32_t options)
{
	printf("firmware download requested\r\n");
	deconfigure_mqtt();

	wifi_config new_wifi_configuration;
	get_default_wifi_config(&new_wifi_configuration);
	new_wifi_configuration.firmware_header_http_address = "https://www.seas.upenn.edu/~warcher/ese516/metadata-cli.bin";
	new_wifi_configuration.firmware_http_address = "https://www.seas.upenn.edu/~warcher/ese516/cli.bin";
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
	deconfigure_wifi_module();
	
}

void publish_sensors()
{
	
	double temp = get_temp();
	double humidity = get_humidity();
	printf("tsl device id 0x%02x", get_tsl2561_device_id());
	power_on_tsl2561();
	int lux = get_lux();
	power_off_tsl2561();
	float moisture = get_moisture();
	
	char mqtt_array[MQTT_SEND_BUFFER_SIZE] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	sprintf(mqtt_array, "%.2f", moisture);
	publish_to_topic(MOISTURE_TOPIC, mqtt_array, strlen(mqtt_array));
	sprintf(mqtt_array, "%d", lux);
	publish_to_topic(LUX_TOPIC, mqtt_array, strlen(mqtt_array));
	sprintf(mqtt_array, "%2.2f", temp);
	publish_to_topic(TEMPERATURE_TOPIC, mqtt_array, strlen(mqtt_array));
	sprintf(mqtt_array, "%2.2f", humidity);
	publish_to_topic(HUMIDITY_TOPIC, mqtt_array, strlen(mqtt_array));
	publish_to_topic(HEARTBEAT_TOPIC, "beat", sizeof("beat"));
}

int main (void)
{
	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_usart();
	
	configure_port_pins_set(PIN_PB23);
	port_pin_set_output_level(PIN_PB23, false);
	
	printf("--- starting Evergreen V%d ---- \r\n", FIRMWARE_VERSION);
	
	mqtt_inst_config new_mqtt;
	get_mqtt_config_defaults(&new_mqtt);
	
	int mqtt_result = mqtt_initialize(&new_mqtt);
	
	if (WIFI_SUCCESS != mqtt_result) printf("\r\n...Wi-Fi failed to configure...\r\n");

	printf("Board initialized.\r\n");
	uint32_t mqtt_counter = 0;

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
	
	configure_port_pins_set(PIN_PA21);
	port_pin_set_output_level(PIN_PA21, 1);
	
	configure_adc(ADC_POSITIVE_INPUT_PIN0);
	
	configure_i2c_hdc();
	configure_i2c_tsl2561(ADDR_FLOAT);
	printf("tsl manufacturer id: 0x%02x\r\n", get_tsl2561_device_id());
	set_resolution(FOURTEEN_BIT_RESOLUTION, FOURTEEN_BIT_RESOLUTION);
	printf("Running as MQTT User: %s\r\n", MQTT_USER);
	register_request_topic(LED_TOPIC, '#', &led_request_callback);
	register_request_topic(RELAY1_TOPIC, '#', &relay_request_callback);
	register_request_topic(RELAY2_TOPIC, '#', &relay_request_callback);
	register_request_topic(UPGRADE_TOPIC, '#', &firmware_request_download_callback);
	register_request_topic(PUMP_TOPIC, '#', &pump_request_callback);
	while (1) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		//sw_timer_task(&swt_module_inst);

		if ((mqtt_counter >= COUNTER_MAX)) {
			mqtt_counter = 0; //reset sensor counter
			printf("Sending sensor values to Cloud.\r\n");
			publish_sensors();
			//publish_sensor_values();
		}

		mqtt_counter++;
	}
	
	/* Insert application code here, after the board has been initialized. */
	return EXIT_SUCCESS;
}
