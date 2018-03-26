#include <asf.h>

struct usart_module usart_instance;

void configure_usart(void)
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

//AT25DFX

#define AT25DFX_BUFFER_SIZE  (10)

#define AT25DFX_SPI                 SERCOM1

/** AT25DFx device type */
#define AT25DFX_MEM_TYPE            AT25DFX_081A

#define AT25DFX_SPI_PINMUX_SETTING  SPI_SIGNAL_MUX_SETTING_E
#define AT25DFX_SPI_PINMUX_PAD0     PINMUX_PA16C_SERCOM1_PAD0
#define AT25DFX_SPI_PINMUX_PAD1     PINMUX_UNUSED
#define AT25DFX_SPI_PINMUX_PAD2     PINMUX_PA18C_SERCOM1_PAD2
#define AT25DFX_SPI_PINMUX_PAD3     PINMUX_PA19C_SERCOM1_PAD3
#define AT25DFX_CS                  PIN_PA07
//! SPI master speed in Hz.
#define AT25DFX_CLOCK_SPEED         1000000

static uint8_t read_buffer[AT25DFX_BUFFER_SIZE];
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;

static void at25dfx_init(void)
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

//nvm
void configure_nvm(void)
{
	struct nvm_config config_nvm;
	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);
}

struct nvm_config config_nvm;

//crc

crc32_t crc1;
crc32_t crc2;



int main (void)
{
	system_init();
	system_interrupt_enable_global();
	at25dfx_init();
	configure_nvm();
	configure_usart();
	// 	nvm_get_config_defaults(&config_nvm);
	// 	config_nvm.manual_page_write = false;
	// 	nvm_set_config(&config_nvm);
	
	printf("inside new app\r\n");
	
	//calculate initial checksum of write
	crc32_calculate(write_buffer, sizeof(write_buffer), &crc1);

	at25dfx_chip_wake(&at25dfx_chip);
	
	//check if chip is there
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		printf("No chip.\r\n");
	}
	
	//read beginning of memory
	at25dfx_chip_read_buffer(&at25dfx_chip, 0x0000, read_buffer, AT25DFX_BUFFER_SIZE);
	//disable protection
	at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x10000, false);
	//erase block (sets to FF's)
	at25dfx_chip_erase_block(&at25dfx_chip, 0x10000, AT25DFX_BLOCK_SIZE_4KB);
	//write write buffer at 0x10000
	at25dfx_chip_write_buffer(&at25dfx_chip, 0x10000, write_buffer, AT25DFX_BUFFER_SIZE);
	//re-enable protection
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);
	
	//read at 0x10000 if read doesn't return OK, there is an error
	if (at25dfx_chip_read_buffer(&at25dfx_chip, 0x10000, read_buffer, AT25DFX_BUFFER_SIZE) != STATUS_OK) {
		printf("Read error\r\n");
	}
	//print read buffer
	for (int i = 0; i < AT25DFX_BUFFER_SIZE; i++) {
		printf("%d", read_buffer[i]);
	}
	printf("\r\n");
	
	//calculate crc for read
	crc32_recalculate(read_buffer, sizeof(read_buffer), &crc2);
	//if they don't match, its an error
	if (crc2 != crc1) {
		printf("CRC error!\r\n");

		} else {
		printf("CRC matched!\r\n");
	}
	
	printf("sleep\r\n");
	
	at25dfx_chip_sleep(&at25dfx_chip);
	return 0;
}