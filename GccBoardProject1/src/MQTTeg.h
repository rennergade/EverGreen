

#include "asf.h"
#include "driver/include/m2m_wifi.h"
#include "iot/mqtt/mqtt.h"
#include "iot/sw_timer.h"
#include "socket/include/socket.h"

//BOARD USER NAMES
#define ACTUATOR_USER						"actuator/"
#define SENSOR_USER						"sensor/"
#define FIRMWARE_USER						"firmware/"

#define MQTT_USER		FIRMWARE_USER //CONFIGURE THIS VALUE FOR THE BOARD YOU NEED TO WORK WITH


/* Max size of UART buffer. */
#define MQTT_SEND_BUFFER_SIZE 64

/* Max size of MQTT buffer. */
#define MAIN_MQTT_BUFFER_SIZE 128

/* Limitation of user name. */
#define MAIN_CHAT_USER_NAME_SIZE 64

// MQTT topics
#define TEMPERATURE_TOPIC                "/g0/temp/" MQTT_USER
#define HUMIDITY_TOPIC                   "/g0/hum/" MQTT_USER
#define LUX_TOPIC                        "/g0/lux/" MQTT_USER
#define MOISTURE_TOPIC                   "/g0/moist/" MQTT_USER

#define PUMP_TOPIC                      "/g0/pump/" MQTT_USER
#define RELAY1_TOPIC					"/g0/relay1/" MQTT_USER
#define RELAY2_TOPIC					"/g0/relay2/" MQTT_USER

#define LED_TOPIC						"/g0/led/" MQTT_USER


#define VERSION_TOPIC                   "/g0/version/" MQTT_USER
#define UPGRADE_TOPIC                   "/g0/upgrade/" MQTT_USER

/*
 * A MQTT broker server which was connected.
 * m2m.eclipse.org is public MQTT broker.
 */
static const char main_mqtt_broker[] = "deet.seas.upenn.edu";

//other broker services
//"m10.cloudmqtt.com";
//"broker.hivemq.com";


/** Wi-Fi Settings */
#define MAIN_WLAN_SSID        "AirPennNet-Device" /* < Destination SSID */
#define MAIN_WLAN_AUTH        M2M_WIFI_SEC_WPA_PSK /* < Security manner */
#define MAIN_WLAN_PSK         "penn1740wifi" /* < Password for Destination SSID */

#define CLOUD_PORT          1883


/** Instance of Timer module. */
struct sw_timer_module swt_module_inst;



/* Instance of MQTT service. */
struct mqtt_module mqtt_inst;

/* Receive buffer of the MQTT service. */
char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];



uint8_t mqttfirmware_download;

//static void wifi_callback(uint8 msg_type, void *msg_data);
static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data);
static void socket_resolve_handler(uint8_t *doamin_name, uint32_t server_ip);
static void mqtt_callback(struct mqtt_module *module_inst, int type, union mqtt_data *data);
//static void configure_timer(void);
void configure_mqtt(void);
//int wifi_init(void);
void publish_sensor_values(void);
