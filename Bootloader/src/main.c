#include <asf.h>
#include "component-configurations.h"
#include "memory-app-definitions.h"

#define VERSION 0

#define DEBUG_LEVEL 1

#if DEBUG_LEVEL >= 3
#include "../debug.h"
#endif

//TODO: Determine where CLI will be located

//#define BOOTLOADER_INSTALL

//TODO: modify max size of firwmare
#define APP_START_ADDRESS 0x04000


#define MAJOR_VERSION(X)    ((X & (0xF << (16 - 8))) >> (16 - 8))       ///macro function bit masking to get bits 9-12
#define MINOR_VERSION(X)    ((X & (0xF << (16 - 12))) >> (16 - 12))     /// macro function bit masking to get bits 5-8
#define REVISION_VERSION(X) (X & (0xF))                                 /// macro function bit masking to get bits 1-4


struct spi_module at25dfx_spi;                  /// spi struct to store configuration information for external flash
struct at25dfx_chip_module at25dfx_chip;        /// chip struct to store config information for external flash
struct usart_module usart_instance;             /// uart struct to hold config information for uart
uint32_t reset_vector[2];                       /// store reset vector for main application code @details have to store 8 bytes because nvm will only read on page aligns


///NOTE: For version, bits are as follows:
///NOTE: uint16: XXXX XXXX XXXX XXXX
///NOTE: 4 MSB are padding, then major, minor, revision.
///NOTE: This means that each versioning can have a maximum number of 15

fw_status_t *boot_status; /// global boot_status


static void update_boot_status(uint16_t fw_version)
{
	enum status_code read_nvm_code;
	fw_status_t new_boot_status = { .signature = { 0xAB, 0xAC, 0xAB }, .new_image_ready = 0, .bl_version = VERSION, .fw_version = fw_version};
	
	struct nvm_parameters nvm_information;
	nvm_get_parameters(&nvm_information);
	uint16_t num_pages = nvm_information.nvm_number_of_pages;
	uint16_t num_rows = num_pages / NVMCTRL_ROW_PAGES;
	int page_to_write = BOOT_STATUS_ADDR / NVMCTRL_PAGE_SIZE;
	int row_to_erase = page_to_write / NVMCTRL_ROW_PAGES;
	uint8_t page_offset = page_to_write - (row_to_erase * NVMCTRL_ROW_PAGES);
	int row_address = row_to_erase * NVMCTRL_ROW_SIZE;
	uint8_t row_buffer[NVMCTRL_ROW_SIZE];

	for (int i = 0; i < NVMCTRL_ROW_PAGES; i++) {
		int offset = i * NVMCTRL_PAGE_SIZE;
		do
		read_nvm_code = nvm_read_buffer(row_address + offset, row_buffer + offset, NVMCTRL_PAGE_SIZE);
		while (STATUS_OK != read_nvm_code);
	}

	do
	read_nvm_code = nvm_erase_row(row_address);
	while (STATUS_OK != read_nvm_code);

	memcpy(row_buffer + (page_offset * NVMCTRL_PAGE_SIZE), &new_boot_status, sizeof(fw_status_t));

	for (int i = 0; i < NVMCTRL_ROW_PAGES; i++) {
		int offset = i * NVMCTRL_PAGE_SIZE;
		do
		read_nvm_code = nvm_write_buffer(row_address + offset, row_buffer + offset, NVMCTRL_PAGE_SIZE);
		while (STATUS_OK != read_nvm_code);
	}
	
}
/**
 * configure ports for reset button
 */
void configure_port_pins(void)
{
	struct port_config config_port_pin;

	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PB23, &config_port_pin);
}

/**
 * calculates crc for firmware in NVM
 * @param  known_checksum checksum calculated during download
 * @return                true if checksums match
 */
bool verify_flash(crc32_t known_checksum, uint32_t fw_size)
{
#if DEBUG_LEVEL >= 1
	uint32_t num_bytes_used_for_crc = 0;
#endif
	crc32_t flash_checksum = 0;
	uint8_t nvm_page_buffer[NVMCTRL_PAGE_SIZE];
	enum status_code nvm_read_code;
	uint remaining_bytes = fw_size % NVMCTRL_PAGE_SIZE;
	uint num_pages = fw_size / NVMCTRL_PAGE_SIZE;
	for (int i = 0; i < num_pages; i++) {
		int offset = NVMCTRL_PAGE_SIZE * i;
		do
			nvm_read_code = nvm_read_buffer(APP_START_ADDRESS + offset, nvm_page_buffer, NVMCTRL_PAGE_SIZE);
		while (STATUS_OK != nvm_read_code);

		crc32_recalculate(nvm_page_buffer, NVMCTRL_PAGE_SIZE, &flash_checksum);
#if DEBUG_LEVEL >= 1
		num_bytes_used_for_crc += NVMCTRL_PAGE_SIZE;
#endif
	}

	do
		nvm_read_code = nvm_read_buffer(APP_START_ADDRESS + (num_pages * NVMCTRL_PAGE_SIZE), nvm_page_buffer, remaining_bytes);
	while (STATUS_OK != nvm_read_code);

	crc32_recalculate(nvm_page_buffer, remaining_bytes, &flash_checksum);

#if DEBUG_LEVEL >= 1
	num_bytes_used_for_crc += remaining_bytes;
	printf("verify_flash: num bytes cycled: %d\r\n", num_bytes_used_for_crc);
	printf("verify_flash: calculated crc val: %04x\r\n", flash_checksum);
	printf("verify_flash: known checksum: %04x\r\n", known_checksum);
#endif

	return flash_checksum == known_checksum;
}

/**
 * helper function to convert verison info to char
 * @param buffer char array to write to
 * @param value  version to convert
 */
static void get_version_digits(char *buffer, uint8_t value)
{
	if (value < 10) {
		//don't want to use itoa to keep footprint small, so doing cast of ascii val
		buffer[1] = (char)(value + 48);
		buffer[0] = '0';
	} else {
		//only want to get first digit
		buffer[1] = (char)(value - 10 + 48);
		buffer[0] = '1';
	}
}

/**
 * helper function to create version info string for print
 * @param buffer  char array to write to
 * @param version version bytestream
 */
static void create_version_string(char *buffer, uint16_t version)
{
	uint8_t major_version = MAJOR_VERSION(version);
	uint8_t minor_version = MINOR_VERSION(version);
	uint8_t revision_version = REVISION_VERSION(version);

	get_version_digits(buffer, major_version);
	buffer[2] = '.';
	get_version_digits(&buffer[3], minor_version);
	buffer[5] = '.';
	get_version_digits(&buffer[6], revision_version);
	buffer[8] = '\0';
}

bool flash_image(uint32_t firmware_address, fw_header_t new_firmware_header)
{
#if DEBUG_LEVEL >= 1
	uint32_t num_bytes_written = 0;
#endif
	enum status_code flash_chip_status;
	uint32_t new_fw_size = new_firmware_header.size;

	//Erase NVM in preparation for flashing
	uint num_rows_to_erase = (new_fw_size % NVMCTRL_ROW_SIZE == 0) ? (new_fw_size / NVMCTRL_ROW_SIZE) : ((new_fw_size / NVMCTRL_ROW_SIZE) + 1);
	for (int i = 0; i < num_rows_to_erase; i++) {
		int offset = NVMCTRL_ROW_SIZE * i;
		do
			flash_chip_status = nvm_erase_row(APP_START_ADDRESS + offset);
		while (STATUS_OK != flash_chip_status);
	}

	uint8_t page_write_buffer[NVMCTRL_PAGE_SIZE];
	uint remaining_bytes = new_fw_size % NVMCTRL_PAGE_SIZE;
	uint num_pages_to_write = new_fw_size / NVMCTRL_PAGE_SIZE;
	for (int i = 0; i < num_pages_to_write; i++) {
		int offset = NVMCTRL_PAGE_SIZE * i;
		flash_chip_status = at25dfx_chip_read_buffer(&at25dfx_chip, firmware_address + offset, page_write_buffer, NVMCTRL_PAGE_SIZE);
		if (STATUS_OK != flash_chip_status) {
			printf("error fetching data from external flash!\r\n");
			return;
		}
		do
			flash_chip_status = nvm_write_buffer(APP_START_ADDRESS + offset, page_write_buffer, NVMCTRL_PAGE_SIZE);
		while (STATUS_OK != flash_chip_status);

#if DEBUG_LEVEL >= 1
		num_bytes_written += NVMCTRL_PAGE_SIZE;
#endif
	}
	//NOW do remaining bytes
	flash_chip_status = at25dfx_chip_read_buffer(&at25dfx_chip, firmware_address + (num_pages_to_write * NVMCTRL_PAGE_SIZE), page_write_buffer, remaining_bytes);
	if (STATUS_OK != flash_chip_status) {
		printf("error fetching data from external flash!\r\n");
		return;
	}
	do
		flash_chip_status = nvm_write_buffer(APP_START_ADDRESS + (NVMCTRL_PAGE_SIZE * num_pages_to_write), page_write_buffer, remaining_bytes);
	while (STATUS_OK != flash_chip_status);

#if DEBUG_LEVEL >= 1
	num_bytes_written += remaining_bytes;
	printf("flash_image: number of bytes flashed: %d\r\n", num_bytes_written);
#endif


	return verify_flash(new_firmware_header.checksum, new_firmware_header.size);
}

/**
 * function to jump to application code
 */
void run_application()
{
	usart_disable(&usart_instance);
	/* Pointer to the Application Section */
	void (*application_code_entry)(void);
	/* Rebase the Stack Pointer */
	__set_MSP(*(uint32_t *)APP_START_ADDRESS);
	/* Rebase the vector table base address TODO: use RAM */
	SCB->VTOR = ((uint32_t)APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);
	/* Load the Reset Handler address of the application */
	application_code_entry = (void (*)(void))(unsigned *)(*(unsigned *)(APP_START_ADDRESS + 4));
	/* Jump to user Reset Handler in the application */
	application_code_entry();
}

/**
 * function that sets up uart and starts bootloader. Able to jump to CLI or new application code
 */
void run_bootloader()
{
	printf("[run_bootloader] started bootloader\r\n");

	if (reset_vector[1] != 0xFFFFFFFF) {
		volatile enum status_code read_nvm_code;
		fw_header_t fw_information;
		
		char fw_version[9];
		char bl_version[9];
		
		create_version_string(fw_version, boot_status->fw_version);
		create_version_string(bl_version, boot_status->bl_version);


		printf("Evergreen Bootloader\r\n"
		       "Current fW version: %s\r\n"
		       "Current BL version: %s\r\n", fw_version, bl_version);

		if (boot_status->new_image_ready) {
			//TODO: Confirm that new image is newer version than current version

			//ready to flash new image
			configure_flash();
			at25dfx_chip_wake(&at25dfx_chip);
			if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK)
				printf("Could not find flash chip.\r\n");
			enum status_code flash_chip_status;
			fw_header_t new_fw_header;
			do
				flash_chip_status = at25dfx_chip_read_buffer(&at25dfx_chip, FW1_HEADER_ADDR, &new_fw_header, sizeof(fw_header_t));
			while (STATUS_OK != flash_chip_status);
#if DEBUG_LEVEL >= 1
			printf("flashing firmware header: \r\n");
			printf("new_fw_header.fw_version: %d\r\n", new_fw_header.fw_version);
			printf("new_fw_header.checksum: %04x\r\n", new_fw_header.checksum);
#endif

			if (flash_image(FW1_ADDR, new_fw_header)) {
				//TODO: change new image flag back to 0
				update_boot_status(new_fw_header.fw_version);
				printf("successfully updated firmware!\r\n");
				at25dfx_chip_sleep(&at25dfx_chip);
				run_application();
			} else {
				printf("error trying to flash image: please power cycle and try again.");
				return;
			}
		} else {
			//no new image, prompt to go to CLI or application
			printf("no new image ready for install. \r\n"
			       "Would you like to go to the Application? (y/n)\r\n");
			volatile enum status_code uart_read_code;
			char singleCharInput = 0;
			do
				uart_read_code = usart_read_buffer_wait(&usart_instance, &singleCharInput, 1);
			while (STATUS_OK != uart_read_code || (singleCharInput != 'y' && singleCharInput != 'n'));
			if ('y' == singleCharInput)
				//TODO: flash cli
				run_application();
		}
	} else {
		printf("No application detected.\r\n");
	}
}

int main(void)
{
	//init system, board, usart, nvm, delay, etc
	system_init();
	system_interrupt_enable_global();
	delay_init();
	configure_usart();
	configure_nvm();
	printf("[main] bootloader started\r\n");
	enum status_code read_nvm_code;
#ifdef BOOTLOADER_INSTALL
//Run only if bootloader has never been installed, sets up bl status header
	update_boot_status(0);
#endif
	
	boot_status = BOOT_STATUS_ADDR;
	
	//Check if there's an application already in memory
	do
		read_nvm_code = nvm_read_buffer(APP_START_ADDRESS, reset_vector, sizeof(reset_vector));
	while (STATUS_OK != read_nvm_code);
	if (reset_vector[1] == 0xFFFFFFFF) {
		run_bootloader();
		return EXIT_SUCCESS;
	}


	configure_port_pins();
	//check if reset button is held
	bool hw_reset = port_pin_get_input_level(PIN_PB23);
	if (!hw_reset) {
		//go into bootloader
		run_bootloader();
		return EXIT_SUCCESS;
	}


	//check if there's a new image ready to be flashed in external memory
	if (boot_status->new_image_ready) {
		//go into bootloader
		run_bootloader();
		return EXIT_SUCCESS;
	}

	//check if boot status has been overwritten (basically MBR)
	if (boot_status->signature[0] != 0xAB
	    || boot_status->signature[1] != 0xAC
	    || boot_status->signature[2] != 0xAB) {
		//flash error, enter bootloader, throw error
		//TODO: write this code
	} else {
		run_application();
		return EXIT_SUCCESS;
	}
}
