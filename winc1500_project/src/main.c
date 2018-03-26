/*
 * main.c
 *
 * Created: 3/26/2018 1:41:14 PM
 *  Author: William
 */ 
#include <asf.h>
#include "driver/include/m2m_wifi.h"

#define BAUD_RATE               115200
#define FW1_HEADER_ADDR         0x001000        /// Header for current or new firmware
#define FW2_HEADER_ADDR         0x004000        /// Header for safe CLI
#define FW1_ADDR                0x002000        /// New/current firmware
#define FW2_ADDR                0x004100        /// CLI firmware

#define AT25DFX_BUFFER_SIZE  (10)
#define AT25DFX_SPI                 SERCOM1
#define AT25DFX_MEM_TYPE            AT25DFX_081A
#define AT25DFX_SPI_PINMUX_SETTING  SPI_SIGNAL_MUX_SETTING_E
#define AT25DFX_SPI_PINMUX_PAD0     PINMUX_PA16C_SERCOM1_PAD0
#define AT25DFX_SPI_PINMUX_PAD1     PINMUX_UNUSED
#define AT25DFX_SPI_PINMUX_PAD2     PINMUX_PA18C_SERCOM1_PAD2
#define AT25DFX_SPI_PINMUX_PAD3     PINMUX_PA19C_SERCOM1_PAD3
#define AT25DFX_CS                  PIN_PA07
#define AT25DFX_CLOCK_SPEED         1000000

#define MAIN_WLAN_SSID	"AirPennNet-Device"
#define MAIN_WLAN_AUTH	M2M_WIFI_SEC_WPA_PSK
#define MAIN_WLAN_PSK	"penn1740wifi"
#define MAIN_HTTP_FILE_URL "http://www.seas.upenn.edu/~warcher/index.html"

static uint8_t read_buffer[AT25DFX_BUFFER_SIZE];
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;
struct usart_module usart_instance;

static void configure_flash(void)
{
	struct at25dfx_chip_config at_chip_config;
	struct spi_config at25dfx_spi_config;
	at25dfx_spi_get_config_defaults(&at25dfx_spi_config);
	at25dfx_spi_config.mode_specific.master.baudrate = AT25DFX_CLOCK_SPEED;
	at25dfx_spi_config.mux_setting = AT25DFX_SPI_PINMUX_SETTING;
	at25dfx_spi_config.pinmux_pad0 = AT25DFX_SPI_PINMUX_PAD0;
	at25dfx_spi_config.pinmux_pad1 = AT25DFX_SPI_PINMUX_PAD1;
	at25dfx_spi_config.pinmux_pad2 = AT25DFX_SPI_PINMUX_PAD2;
	at25dfx_spi_config.pinmux_pad3 = AT25DFX_SPI_PINMUX_PAD3;
	spi_init(&at25dfx_spi, AT25DFX_SPI, &at25dfx_spi_config);
	spi_enable(&at25dfx_spi);
	
	at_chip_config.type = AT25DFX_MEM_TYPE;
	at_chip_config.cs_pin = AT25DFX_CS;
	at25dfx_chip_init(&at25dfx_chip, &at25dfx_spi, &at_chip_config);
}

static void configure_usart(void)
{
	struct usart_config config_usart;

	usart_get_config_defaults(&config_usart);
	config_usart.baudrate = 115200;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PB10D_SERCOM4_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;


	stdio_serial_init(&usart_instance, SERCOM4, &config_usart);

	usart_enable(&usart_instance);
}

int main() {
//NOTE: HTTP_DOWNLOADER does not actually run just a heads up
	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_usart();
	
	nm_bsp_init(); //initialize wifi chip
	
	tstrWifiInitParam wifi_params;
	memset((uint8_t *) &wifi_params, 0, sizeof(tstrWifiInitParam));
	
	wifi_params.pfAppWifiCb = 0xFFFFFF;
	int8_t ret = m2m_wifi_init(&wifi_params); // NOTE: m2m = machine to machine
	if(M2M_SUCCESS != ret) {
		printf("failed to initialize wifi parameters\r\n");
	}
	
	printf("--- Welcome to ATWINC 1500 Connection Test ---\r\n");
	uint8_t mac_address;
	m2m_wifi_get_mac_address(&mac_address);
	printf("WINC1500 MAC: %X\r\n", mac_address);
	return 0;
	
}