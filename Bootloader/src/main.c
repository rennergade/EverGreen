#include <asf.h>


//TODO: Determine where CLI will be located

#define BOOTLOADER_INSTALL		1

#define APP_START_ADDRESS       0x02000
#define BOOT_STATUS_ADDR        0x01F00
#define BAUD_RATE               115200


/** AT25DFx device type */
#define AT25DFX_BUFFER_SIZE         10
#define AT25DFX_MEM_TYPE            AT25DFX_081A
#define AT25DFX_SPI                 SERCOM5
#define AT25DFX_SPI_PINMUX_SETTING  SPI_SIGNAL_MUX_SETTING_E
#define AT25DFX_SPI_PINMUX_PAD0     PIN_PA16C_SERCOM1_PAD0
#define AT25DFX_SPI_PINMUX_PAD1     PINMUX_UNUSED
#define AT25DFX_SPI_PINMUX_PAD2     PIN_PA18C_SERCOM1_PAD2
#define AT25DFX_SPI_PINMUX_PAD3     PIN_PA19C_SERCOM1_PAD3
#define AT25DFX_CS                  PIN_PA07D_SERCOM0_PAD3
#define AT25DFX_CLOCK_SPEED         1000000


#define MAJOR_VERSION(X)    (X & (0xF >> 16 - 8))
#define MINOR_VERSION(X)    (X & (0xF >> 16 - 12))
#define REVISION_VERSION(X) (X & (0xF))


static uint8_t read_buffer[AT25DFX_BUFFER_SIZE];
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;
struct usart_module usart_instance;
uint32_t reset_vector[2];


typedef struct fw_status {
	uint8_t signature[3];           ///set values that confirm fw succesfully flashed
	uint8_t new_image_ready;        /// are we ready to write a new image?
} fw_status_t;


///NOTE: For version, bits are as follows:
///NOTE: uint16: XXXX XXXX XXXX XXXX
///NOTE: 4 MSB are padding, then major, minor, revision.
///NOTE: This means that each versioning can have a maximum number of 15
typedef struct fw_header {
	uint16_t	fw_version;     /// firmware version
	uint16_t	bl_version;     /// bootloader version
	uint16_t	checksum;       /// precomputed checksum value to verify
} fw_header_t;

fw_status_t boot_status;


void configure_port_pins(void)
{
	struct port_config config_port_pin;

	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PA15, &config_port_pin);
}

static void configure_nvm(void)
{
	struct nvm_config config_nvm;

	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	
	nvm_set_config(&config_nvm);
}

void configure_usart(void)
{
	struct usart_config config_usart;

	usart_get_config_defaults(&config_usart);
	config_usart.baudrate = BAUD_RATE;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PB10D_SERCOM4_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;


	stdio_serial_init(&usart_instance, SERCOM4, &config_usart);

	usart_enable(&usart_instance);
}

static void nvm_read(const uint32_t src_address, uint8_t* dest_buffer, uint16_t read_num_bytes) {
	
}

static void nvm_erase(const uint32_t starting_row_address) {
	
}

static void nvm_write(const uint32_t dest_address, uint8_t* src_buffer, uint16_t write_num_bytes) {
	
}

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

static void get_version_digits(char *buffer, uint8_t value)
{
	if (value < 10) {
                //don't want to use itoa to keep footprint small, so doing cast of ascii val
		buffer[0] = (char)(value + 48);
		buffer[1] = '0';
	} else {
                //only want to get first digit
		buffer[0] = (char)(value - 10 + 48);
		buffer[1] = '1';
	}
}

static void create_version_string(char *buffer, uint16_t version)
{
	uint8_t major_version = MAJOR_VERSION(version);
	uint8_t minor_version = MINOR_VERSION(version);
	uint8_t revision_version = REVISION_VERSION(version);

	get_version_digits(buffer, major_version);
	buffer[2] = '.';
	get_version_digits(buffer[3], minor_version);
	buffer[5] = '.';
	get_version_digits(buffer[6], revision_version);
}

void run_application()
{
	void (*application_code_entry)(void);
	__set_MSP(*(uint32_t *)APP_START_ADDRESS);

	SCB->VTOR = ((uint32_t)APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

	application_code_entry = (void (*)(void))(unsigned *) (*(unsigned *)(APP_START_ADDRESS + 4));

	application_code_entry();
}


void run_bootloader()
{
	configure_usart();
	printf("bootloader\n");
	volatile enum status_code uart_read_code;
	char singleCharInput;
	do
		uart_read_code = usart_read_buffer_wait(&usart_instance, &singleCharInput, 1);
	while (STATUS_OK != uart_read_code && singleCharInput != '#');

	printf("got #\n\r");
	//TODO: check if there is application flashed before doing this
	if(reset_vector[1] == 0xFFFFFFFF) {
		volatile enum status_code read_nvm_code;
		fw_header_t fw_information;
		/*
		uint8_t page_buffer[NVMCTRL_PAGE_SIZE];
	
		do
			read_nvm_code = nvm_read_buffer(BOOT_STATUS_ADDR, );
		while (STATUS_OK != read_nvm_code); */

		char fw_version[8];
		char bl_version[8];
		uint16_t test_fw_version = 0x3c42; //0011 1100 0100 0010 14.04.02
		uint16_t test_bl_version = 0x1e80; //0001 1110 1000 0000 14.08.00
		create_version_string(fw_version, test_fw_version);
		create_version_string(bl_version, test_bl_version);
		//create_version_string(fw_version, fw_information.fw_version);
		//create_version_string(bl_version, fw_information.bl_version);


		printf("Evergreen Bootloader\n\r"
		"Current fW version: %s\n\r"
		"Current BL version: %s\n\r", fw_version, bl_version);

		if (boot_status.new_image_ready) {
			//ready to flash new image
			
		} else {
			//no new image start cli
			run_application();
		}	
	} else {
		printf("No application detected.\n\r");
	}
}

int main(void)
{
	//TODO: find out how to disable reset button
	
	
	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_nvm();
	
		enum status_code read_nvm_code;
	
	if(BOOTLOADER_INSTALL) {
		
		//TODO: install bootloader status
		struct nvm_parameters nvm_information;
		nvm_get_parameters(&nvm_information);
		uint16_t num_pages = nvm_information.nvm_number_of_pages;
		uint16_t num_rows = num_pages / NVMCTRL_ROW_PAGES;
		int page_to_write = BOOT_STATUS_ADDR / NVMCTRL_PAGE_SIZE;
		int row_to_erase = page_to_write/ NVMCTRL_ROW_PAGES;
		uint8_t page_offset = page_to_write - (row_to_erase*NVMCTRL_ROW_PAGES);
		int row_address = row_to_erase * NVMCTRL_ROW_SIZE;
		uint8_t row_buffer[NVMCTRL_ROW_SIZE];
			
		for(int i=0; i< NVMCTRL_ROW_PAGES; i++) {
			int offset = i*NVMCTRL_PAGE_SIZE;
			do
				read_nvm_code = nvm_read_buffer(row_address+offset, row_buffer + offset, NVMCTRL_PAGE_SIZE);
			while (STATUS_OK != read_nvm_code);	
		}
		
		do
			read_nvm_code = nvm_erase_row(row_address);
		while (STATUS_OK != read_nvm_code);
		
		fw_status_t write_boot = {.signature = {0xAB,0xAC,0xAB}, .new_image_ready = 0};
		
		memcpy(row_buffer+(page_offset*NVMCTRL_PAGE_SIZE), &write_boot, sizeof(fw_status_t));
		
		for(int i=0; i< NVMCTRL_ROW_PAGES; i++) {
			int offset = i*NVMCTRL_PAGE_SIZE;
			do
				read_nvm_code = nvm_write_buffer(row_address+offset, row_buffer+offset, NVMCTRL_PAGE_SIZE);
			while (STATUS_OK != read_nvm_code);
		}
		

	}
	
	do
		read_nvm_code = nvm_read_buffer(APP_START_ADDRESS, reset_vector, sizeof(reset_vector));
	while (STATUS_OK != read_nvm_code);

	if (reset_vector[1] == 0xFFFFFFFF)
		//go into bootloader
		run_bootloader();

	
	configure_port_pins();
	bool hw_reset = port_pin_get_input_level(PIN_PA15);
	
	if (!hw_reset)
		//go into bootloader
		run_bootloader();



	do
		read_nvm_code = nvm_read_buffer(BOOT_STATUS_ADDR, &boot_status, sizeof(fw_status_t));
	while (STATUS_OK != read_nvm_code);

	if (boot_status.new_image_ready)
		//go into bootloader
		run_bootloader();


	if (boot_status.signature[0] != 0xAB
	    || boot_status.signature[1] != 0xAC
	    || boot_status.signature[2] != 0xAB) {
                //flash error, enter bootloader, throw error
                //TODO: write this code
	}
	
	//run_application();
}
