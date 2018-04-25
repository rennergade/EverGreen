#ifndef MQTT_H_
#define MQTT_H_

#include "asf.h"
#include "driver/include/m2m_wifi.h"
#include "iot/mqtt/mqtt.h"
#include "iot/sw_timer.h"
#include "socket/include/socket.h"



//BOARD USER NAMES
#define ACTUATOR_USER						"actuator/"
#define SENSOR_USER						"sensor/"
#define FIRMWARE_USER						"firmware/"

#define MQTT_USER		SENSOR_USER //CONFIGURE THIS VALUE FOR THE BOARD YOU NEED TO WORK WITH


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
#define HEARTBEAT_TOPIC                   "/g0/heartbeat/" MQTT_USER
/* Max size of UART buffer. */
#define MQTT_SEND_BUFFER_SIZE 64
#define MAIN_MQTT_BUFFER_SIZE 128
#define MAX_TOPICS 7

//other broker services
//"m10.cloudmqtt.com";
//"broker.hivemq.com";

/**
 * @typedef mqtt_inst_config
 * @brief struct containing configuration values for an MQTT instance
 */
typedef struct {
  char* ssid;
  uint32_t auth;
  char* password;
  uint16_t port;
  char* broker_server;
} mqtt_inst_config;


typedef void (*callback)(uint32_t); /// callback for topic function

typedef struct {
  char topic_name[MQTT_SEND_BUFFER_SIZE];
  callback function;
} topic_struct;

uint8_t mqttfirmware_download;
mqtt_inst_config *curr_mqtt_config;

/**
 * Register a topic that requests information from MQTT Broker
 * @param topic_name name of topic to register
 * @param function function callback for topic
 */
void register_request_topic(char topic_name[MQTT_SEND_BUFFER_SIZE], char wildcard, void (*function));

/**
 * Initialize mqtt conf struct to default values
 * @param mqtt_conf pointer to mqtt_conf struct to use
 */
void get_mqtt_config_defaults(mqtt_inst_config *mqtt_conf);
/**
 * Disable mqtt to enable another TCP session
 */
void deconfigure_mqtt();
/**
 * Initialize TCP session for MQTT with configuration from @p mqtt_conf
 * @param  mqtt_conf configuration information for MQTT
 * @return           1 if successful
 */
int mqtt_initialize(mqtt_inst_config* mqtt_conf);


void publish_to_topic(char topic[MAIN_MQTT_BUFFER_SIZE], uint8_t data[MQTT_SEND_BUFFER_SIZE], uint32_t data_len);

#endif
