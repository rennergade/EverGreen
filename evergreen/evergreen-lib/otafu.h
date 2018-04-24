#ifndef WIFI_H_
#define WIFI_H_

#include <asf.h>
#include <errno.h>
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "iot/http/http_client.h"
#include "component-configurations.h"
#include "memory-app-definitions.h"

#define MTU_HTTP 1500                                                                   /// maximum transmission unit (MTU) or maximum packet size for HTTP @details limited by Ethernet
#define IPV4_BYTE(val, index) ((val >> (index * 8)) & 0xFF)                             /// macro function for bit masking IPV4 addressing

/**
 * @typedef download_state
 * @brief current state of downloader
 * @details keeps current state sequential during async functions (ie callbacks)
 */
typedef enum {
	NOT_READY		= 0,    ///  Not ready
	STORAGE_READY		= 0x01, /// Storage is ready
	WIFI_CONNECTED		= 0x02, /// Wi-Fi is connected
	GET_REQUESTED		= 0x04, /// GET request is sent
	DOWNLOADING		= 0x08, /// Running to download
	COMPLETED		= 0x10, /// Download completed
	CANCELED		= 0x20, /// Download canceled
	NOT_CHECKED		= 0x40, /// haven't checked update
	UPDATE_AVAILABLE	= 0x80, /// Update Available
	UPDATE_NOT_AVAILABLE	= 0x100 //Update not available
} download_state;

typedef struct {
  char* ssid;
  uint32_t auth_type;
  char* password;
  char* firmware_header_http_address;
  char* firmware_http_address;
} wifi_config;

wifi_config *current_wifi_config;

void get_default_wifi_config(wifi_config *wifi_configuration);
void configure_wifi_module(wifi_config *wifi_configuration);
void deconfigure_wifi_module();
bool check_for_update();
bool download_firmware();


#endif
