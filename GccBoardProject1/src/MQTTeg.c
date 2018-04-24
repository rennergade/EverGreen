/**  * \section files Main Files
 * - main.c : Initialize the WINC1500, connect to MQTT broker and communicate with the other devices.
 * - mqtt.h : Implementation of MQTT 3.1
 *
 * \section usage Usage
 * -# Configure below code in the main.h for AP information to be connected.
 * \code
 *    #define MAIN_WLAN_SSID         "DEMO_AP"
 *    #define MAIN_WLAN_AUTH         M2M_WIFI_SEC_WPA_PSK
 *    #define MAIN_WLAN_PSK          "12345678"
 * \endcode
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application as the follows.
 * \code
 *    Baud Rate : 9600
 *    Data : 8bit
 *    Parity bit : none
 *    Stop bit : 1bit
 *    Flow control : none
 *    Line-Ending style : LF or CR+LF
 * \endcode
 */

#include "asf.h"
#include "MQTTeg.h"
#include "handler.h"
#include "TSL2561.h"
#include "hdc_1080.h"

static uint8_t RcvDownloadFwCmdByMQTT;
static uint8_t RequestVersionByMQTT;

volatile int wifi_connected = 0;
volatile int mqtt_connected = 0;

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] msg_type type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_callback(uint8 msg_type, void *msg_data)
{
	tstrM2mWifiStateChanged *msg_wifi_state;
	uint8 *msg_ip_addr;

	switch (msg_type) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
		msg_wifi_state = (tstrM2mWifiStateChanged *)msg_data;
		if (msg_wifi_state->u8CurrState == M2M_WIFI_CONNECTED) {
			/* If Wi-Fi is connected. */
			printf("Wi-Fi connected\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (msg_wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) {
			/* If Wi-Fi is disconnected. */
			printf("Wi-Fi disconnected\r\n");
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
					MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			/* Disconnect from MQTT broker. */
			/* Force close the MQTT connection, because cannot send a disconnect message to the broker when network is broken. */
			mqtt_disconnect(&mqtt_inst, 1);
		}

		break;

	case M2M_WIFI_REQ_DHCP_CONF:
		msg_ip_addr = (uint8 *)msg_data;
		printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
				msg_ip_addr[0], msg_ip_addr[1], msg_ip_addr[2], msg_ip_addr[3]);
		/* Try to connect to MQTT broker when Wi-Fi was connected. */
		mqtt_connect(&mqtt_inst, main_mqtt_broker);
		wifi_connected = 1; 
		break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the Socket event.
 *
 * \param[in] Socket descriptor.
 * \param[in] msg_type type of Socket notification. Possible types are:
 *  - [SOCKET_MSG_CONNECT](@ref SOCKET_MSG_CONNECT)
 *  - [SOCKET_MSG_BIND](@ref SOCKET_MSG_BIND)
 *  - [SOCKET_MSG_LISTEN](@ref SOCKET_MSG_LISTEN)
 *  - [SOCKET_MSG_ACCEPT](@ref SOCKET_MSG_ACCEPT)
 *  - [SOCKET_MSG_RECV](@ref SOCKET_MSG_RECV)
 *  - [SOCKET_MSG_SEND](@ref SOCKET_MSG_SEND)
 *  - [SOCKET_MSG_SENDTO](@ref SOCKET_MSG_SENDTO)
 *  - [SOCKET_MSG_RECVFROM](@ref SOCKET_MSG_RECVFROM)
 * \param[in] msg_data A structure contains notification informations.
 */
static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data)
{
	mqtt_socket_event_handler(sock, msg_type, msg_data);
}

/**
 * \brief Callback of gethostbyname function.
 *
 * \param[in] doamin_name Domain name.
 * \param[in] server_ip IP of server.
 */
static void socket_resolve_handler(uint8_t *doamin_name, uint32_t server_ip)
{
	mqtt_socket_resolve_handler(doamin_name, server_ip);
}

/**
 * \brief Callback to get the MQTT status update.
 *
 * \param[in] conn_id instance id of connection which is being used.
 * \param[in] type type of MQTT notification. Possible types are:
 *  - [MQTT_CALLBACK_SOCK_CONNECTED](@ref MQTT_CALLBACK_SOCK_CONNECTED)
 *  - [MQTT_CALLBACK_CONNECTED](@ref MQTT_CALLBACK_CONNECTED)
 *  - [MQTT_CALLBACK_PUBLISHED](@ref MQTT_CALLBACK_PUBLISHED)
 *  - [MQTT_CALLBACK_SUBSCRIBED](@ref MQTT_CALLBACK_SUBSCRIBED)
 *  - [MQTT_CALLBACK_UNSUBSCRIBED](@ref MQTT_CALLBACK_UNSUBSCRIBED)
 *  - [MQTT_CALLBACK_DISCONNECTED](@ref MQTT_CALLBACK_DISCONNECTED)
 *  - [MQTT_CALLBACK_RECV_PUBLISH](@ref MQTT_CALLBACK_RECV_PUBLISH)
 * \param[in] data A structure contains notification informations. @ref mqtt_data
 */
static void mqtt_callback(struct mqtt_module *module_inst, int type, union mqtt_data *data)
{
	switch (type) {
	case MQTT_CALLBACK_SOCK_CONNECTED:
	{
		/*
		 * If connecting to broker server is complete successfully, Start sending CONNECT message of MQTT.
		 * Or else retry to connect to broker server.
		 */
		if (data->sock_connected.result >= 0) {
			
			mqtt_connect_broker(module_inst, 1, NULL, NULL, MQTT_USER, NULL, NULL, 0, 0, 0);
			
			mqtt_connected = 1;
			
			

		} else {
			printf("Connect fail to server(%s)! retry it automatically.\r\n", main_mqtt_broker);
			mqtt_connect(module_inst, main_mqtt_broker); /* Retry that. */
		}
	}
	break;

	case MQTT_CALLBACK_CONNECTED:
		if (data->connected.result == MQTT_CONN_RESULT_ACCEPT) {
			/* Subscribe chat topic. */
			  module_inst->busy = 0;
			  mqtt_subscribe(module_inst, PUMP_TOPIC "#", 0);
			  mqtt_subscribe(module_inst, RELAY1_TOPIC "#", 0);
			  mqtt_subscribe(module_inst, RELAY2_TOPIC "#", 0);
			  mqtt_subscribe(module_inst, LED_TOPIC "#", 0);
			
			  mqtt_subscribe(module_inst, UPGRADE_TOPIC "#", 0);
			  mqtt_subscribe(module_inst, VERSION_TOPIC "#", 0);
			printf("Preparation of MQTT has been completed.\r\n");
		} else {
			/* Cannot connect for some reason. */
			printf("MQTT broker declined your access! error code %d\r\n", data->connected.result);
		}

		break;

	case MQTT_CALLBACK_RECV_PUBLISH:
		if (data->recv_publish.topic != NULL && data->recv_publish.msg != NULL)
		{
			if (strncmp(data->recv_publish.topic, PUMP_TOPIC, strlen(PUMP_TOPIC)) == 0)
			{
					printf("%s >> ", PUMP_TOPIC);
					run_pump(10000);
				
			}
			else if (strncmp(data->recv_publish.topic, RELAY1_TOPIC, strlen(RELAY1_TOPIC)) == 0)
			{
					printf("%s >> ", RELAY1_TOPIC);
					if (strncmp("on", data->recv_publish.msg, data->recv_publish.msg_size) == 0) {
						relay1_enable();
					} 
					else if (strncmp("off", data->recv_publish.msg, data->recv_publish.msg_size) == 0) {
						relay1_disable();
					}
				

			}
			else if (strncmp(data->recv_publish.topic, LED_TOPIC, strlen(LED_TOPIC)) == 0)
			{

					printf("%s >> ", LED_TOPIC);
				
					if (strncmp("on", data->recv_publish.msg, data->recv_publish.msg_size) == 0) {
						led2_on();
					}
					else if (strncmp("off", data->recv_publish.msg, data->recv_publish.msg_size) == 0) {
						led2_off();
					}
				

			}
			else if (strncmp(data->recv_publish.topic, RELAY2_TOPIC, strlen(RELAY2_TOPIC)) == 0)
			{
					printf("%s >> ", RELAY2_TOPIC);
					if (strncmp("on", data->recv_publish.msg, data->recv_publish.msg_size) == 0) {
						relay2_enable();
					}
					else if (strncmp("off", data->recv_publish.msg, data->recv_publish.msg_size) == 0) {
						relay2_disable();
					}
				

			}
			else if (strncmp(data->recv_publish.topic, UPGRADE_TOPIC, strlen(UPGRADE_TOPIC)) == 0)
			{
					printf("%s >> ", UPGRADE_TOPIC);
					RcvDownloadFwCmdByMQTT = 1;
				
			}
			else
			{
				printf("Unknown topic: %s", data->recv_publish.topic);
			}
			for (uint8_t i; i < data->recv_publish.msg_size; i++)
			{
				printf("%c", data->recv_publish.msg[i]);
			}
			printf("\r\n");
		}
		break;

	case MQTT_CALLBACK_DISCONNECTED:
		/* Stop timer */
		printf("MQTT disconnected\r\n");
		break;
	}
}


/**
 * \brief Configure Timer module.
 */
static void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
	printf("Timer enabled.\r\n");
}

/**
 * \brief Configure MQTT service.
 */
void configure_mqtt(void)
{
	struct mqtt_config mqtt_conf;
	int result;

	mqtt_get_config_defaults(&mqtt_conf);
	/* To use the MQTT service, it is necessary to always set the buffer and the timer. */
	mqtt_conf.timer_inst = &swt_module_inst;
	mqtt_conf.recv_buffer = mqtt_buffer;
	mqtt_conf.recv_buffer_size = MAIN_MQTT_BUFFER_SIZE;
	mqtt_conf.port = CLOUD_PORT;
	//cloudmqtt port 11353

	result = mqtt_init(&mqtt_inst, &mqtt_conf);
	if (result < 0) {
		printf("MQTT initialization failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}

	result = mqtt_register_callback(&mqtt_inst, mqtt_callback);
	if (result < 0) {
		printf("MQTT register callback failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}
}
/**
 * \brief Initialize the WiFi
 */
int wifi_init(void) 
{
	tstrWifiInitParam param;
	int8_t ret;

	/* Initialize the Timer. */
	configure_timer();

	/* Initialize the MQTT service. */
	configure_mqtt();

	/* Initialize the BSP. */
	nm_bsp_init();
	
	printf("MQTT Configured.\r\n");

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_callback; /* Set Wi-Fi event callback. */
	printf("Initializing...\r\n");
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		return 1;
	} 
		printf("main: m2m_wifi_init call success!(%d)\r\n", ret);
	
	/* Initialize socket interface. */
	socketInit();
	registerSocketCallback(socket_event_handler, socket_resolve_handler);
	printf("Sockets initialized.\r\n");
	
	return 0;
}

void publish_sensor_values(void) {
	
		uint8_t mqtt_send_buffer[MQTT_SEND_BUFFER_SIZE];
		printf("Publishing version to %s\r\n", VERSION_TOPIC);
		//version
		memset(mqtt_send_buffer, 0, sizeof(mqtt_send_buffer));
		sprintf(mqtt_send_buffer, "%s", APP_VERSION); //set to current firmware
		mqtt_publish(&mqtt_inst, VERSION_TOPIC, mqtt_send_buffer, strlen(mqtt_send_buffer), 0, 0);
		
		
		//temp
		set_resolution(FOURTEEN_BIT_RESOLUTION,FOURTEEN_BIT_RESOLUTION);
		double temperature = get_temp();
		double humidity = get_humidity();
		
		printf("Temperature: %.02f\r\n", temperature);
		printf("Humidity: %.02f\r\n", humidity);

		
		memset(mqtt_send_buffer, 0, sizeof(mqtt_send_buffer));
		sprintf(mqtt_send_buffer, "%.02f", temperature);
		mqtt_publish(&mqtt_inst, TEMPERATURE_TOPIC, mqtt_send_buffer, strlen(mqtt_send_buffer), 0, 0);
		
		//humidity
		memset(mqtt_send_buffer, 0, sizeof(mqtt_send_buffer));
		sprintf(mqtt_send_buffer, "%.02f", humidity);
		mqtt_publish(&mqtt_inst, HUMIDITY_TOPIC, mqtt_send_buffer, strlen(mqtt_send_buffer), 0, 0);
		
		
		//lux
		
		power_on_tsl2561();
		uint32_t lux_value = get_lux();
		power_off_tsl2561();
		
		printf("Lux: %d\r\n", lux_value);
		
		
		memset(mqtt_send_buffer, 0, sizeof(mqtt_send_buffer));
		sprintf(mqtt_send_buffer, "%d", lux_value);
		mqtt_publish(&mqtt_inst, LUX_TOPIC, mqtt_send_buffer, strlen(mqtt_send_buffer), 0, 0);
		
		
		//moisture
		float m_value = get_moisture();
		
		printf("Moisture: %.02f\r\n", m_value);
		
		memset(mqtt_send_buffer, 0, sizeof(mqtt_send_buffer));
		sprintf(mqtt_send_buffer, "%.02f", m_value);
		mqtt_publish(&mqtt_inst, MOISTURE_TOPIC, mqtt_send_buffer, strlen(mqtt_send_buffer), 0, 0);
		
	
}
