

#include "asf.h"
#include "driver/include/m2m_wifi.h"
#include "iot/mqtt/mqtt.h"
#include "iot/sw_timer.h"
#include "socket/include/socket.h"

/* Max size of UART buffer. */
#define MQTT_SEND_BUFFER_SIZE 64

/* Max size of MQTT buffer. */
#define MAIN_MQTT_BUFFER_SIZE 128

/* Limitation of user name. */
#define MAIN_CHAT_USER_NAME_SIZE 64

// MQTT topics
#define TEMPERATURE_TOPIC                "/g0/temp/"
#define HUMIDITY_TOPIC                   "/g0/hum/"
#define LUX_TOPIC                        "/g0/lux/"
#define MOISTURE_TOPIC                   "/g0/moist/"

#define PUMP_TOPIC                      "/g0/pump/"
#define RELAY1_TOPIC					"/g0/relay1/"
#define RELAY2_TOPIC					"/g0/relay2/"

#define LED_TOPIC						"/g0/led/"


#define VERSION_TOPIC                   "/g0/version/"
#define UPGRADE_TOPIC                   "/g0/upgrade/"
#define REPLY_TOPIC                     "/g0/reply/"

/*
 * A MQTT broker server which was connected.
 * m2m.eclipse.org is public MQTT broker.
 */
static const char main_mqtt_broker[] = "deet.seas.upenn.edu";

//"m10.cloudmqtt.com";


//"broker.hivemq.com";

//"m10.cloudmqtt.com";

/** Wi-Fi Settings */
#define MAIN_WLAN_SSID        "AirPennNet-Device" /* < Destination SSID */
#define MAIN_WLAN_AUTH        M2M_WIFI_SEC_WPA_PSK /* < Security manner */
#define MAIN_WLAN_PSK         "penn1740wifi" /* < Password for Destination SSID */

#define BROKERNAME			"wjmiinuz"
#define BROKERPASS			 "IW5PXtBCn9mn"
#define CLOUD_PORT          1883


/** Instance of Timer module. */
struct sw_timer_module swt_module_inst;

/** User name of chat. */
#define MQTT_USER		"evergreen1"

/* Instance of MQTT service. */
struct mqtt_module mqtt_inst;

/* Receive buffer of the MQTT service. */
char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];



static uint8_t RcvDownloadFwCmdByMQTT;
static uint8_t RequestVersionByMQTT;

static void wifi_callback(uint8 msg_type, void *msg_data);
static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data);
static void socket_resolve_handler(uint8_t *doamin_name, uint32_t server_ip);
static void mqtt_callback(struct mqtt_module *module_inst, int type, union mqtt_data *data);
static void configure_timer(void);
static void configure_mqtt(void);
int wifi_init(void);
