#include "wifi.h"

#define FW_VERSION 0

//Flash global vars
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;
uint8_t write_row_buffer[FLASH_ROW_SIZE];       /// buffered write so only write 256 bytes at a time
uint8_t buffer_cursor;                          /// location to write to in row buffer
unsigned int write_address;                     /// current write address in flash memory
crc32_t crc_val = 0;

struct http_client_module http_client_module_inst;
static uint32_t http_file_size;
static uint32_t received_file_size = 0; /// number of bytes successfully written to memory
unsigned int total_bytes_written;
download_state down_state;
uint16_t new_firmware_version = 0;
uint32_t new_firmware_checksum = 0;

struct sw_timer_module swt_module_inst;

uint32_t global_test_counter = 0; /// Test counter for printing debug statements

/**
 * remove state from global download_state @ref down_state
 * @param mask state to clear
 *
 */
//TODO: check and see if it will only clear on mask
static void clear_state(download_state mask)
{
	down_state &= ~mask;
}

/**
 * add a given state to the global download_state @ref down_state
 * @param mask add given state
 */
static void add_state(download_state mask)
{
	down_state |= mask;
	if (mask == CANCELED || mask == COMPLETED)
		at25dfx_chip_sleep(&at25dfx_chip);
	else if (mask == DOWNLOADING)
		at25dfx_chip_wake(&at25dfx_chip);
}
/**
 * initialize global @ref down_state
 */
static void init_state()
{
	down_state = NOT_READY;
	add_state(NOT_CHECKED);
}

/**
 * check is a state is set or not
 * @param  mask state to check
 * @return      true if state is set
 */
static inline bool is_state_set(download_state mask)
{
	return (down_state & mask) != 0;
}

/**
 * ensures proper state before GET Request
 */
static void start_download()
{
	if (!is_state_set(STORAGE_READY)) {
		printf("start_download: MMC storage not ready.\r\n");
		return;
	}

	if (!is_state_set(WIFI_CONNECTED)) {
		printf("start_download: Wi-Fi is not connected.\r\n");
		return;
	}

	if (is_state_set(GET_REQUESTED)) {
		printf("start_download: request is sent already.\r\n");
		return;
	}

	if (is_state_set(DOWNLOADING)) {
		printf("start_download: running download already.\r\n");
		return;
	}
	if (is_state_set(NOT_CHECKED)) {
		printf("start_download [NOT_CHECKED]: sending HTTP request for header\r\n");
		http_client_send_request(&http_client_module_inst, current_wifi_config->firmware_header_http_address, HTTP_METHOD_GET, NULL, NULL);
		return;
	}
	if (is_state_set(UPDATE_AVAILABLE)) {
		printf("start_download [UPDATE_AVAILABLE]: sending HTTP request for firmware\r\n");
		http_client_send_request(&http_client_module_inst, current_wifi_config->firmware_http_address, HTTP_METHOD_GET, NULL, NULL);
		return;
	}
	if (is_state_set(UPDATE_NOT_AVAILABLE)) {
		printf("start_download: no update available\r\n");
		add_state(COMPLETED);
		return;
	}
}

static void write_firmware_metadata(uint32_t firmware_address, uint16_t firmware_version, uint32_t firmware_checksum, uint32_t fw_size)
{
	if (FW1_ADDR == firmware_address)
		firmware_address = FW1_HEADER_ADDR;
	else if (FW2_ADDR == firmware_address)
		firmware_address = FW2_HEADER_ADDR;
	else
		//TODO: throw error
		return;

	//NOTE: existing firmware information has already been erased when chip was being prepped
	//NOTE: this is to prevent metadata corruption or a mismatch in the case of failure

	fw_header_t new_firmware_header = { .fw_version = firmware_version, .checksum = firmware_checksum, .size = fw_size };
	uint8_t write_buffer[FLASH_ROW_SIZE];
	memcpy(write_buffer, &new_firmware_header, sizeof(fw_header_t));
	enum status_code write_code = at25dfx_chip_write_buffer(&at25dfx_chip, firmware_address, write_buffer, FLASH_ROW_SIZE);
	if (STATUS_OK != write_code) {
		printf("write_firmware_metadata: could not write to flash!\r\n");
		return;
	}
}

/**
 * update the boot status struct in internal memory
 */
static void update_boot_status()
{
	struct nvm_parameters nvm_information;

	nvm_get_parameters(&nvm_information);
	uint16_t num_pages = nvm_information.nvm_number_of_pages;
	int page_to_write = BOOT_STATUS_ADDR / NVMCTRL_PAGE_SIZE;
	int row_to_erase = page_to_write / NVMCTRL_ROW_PAGES;
	printf("update_boot_status: row_to_erase: %d\r\n", row_to_erase);
	printf("update_boot_status: page_to_write: %d\r\n", page_to_write);
	uint8_t page_offset = page_to_write - (row_to_erase * NVMCTRL_ROW_PAGES);
	int row_address = row_to_erase * NVMCTRL_ROW_SIZE;
	printf("update_boot_status: row_address: %d\r\n", row_address);
	printf("update_boot_status: page_offset: %d\r\n", page_offset);
	uint8_t row_buffer[NVMCTRL_ROW_SIZE];
	enum status_code read_nvm_code;
	for (int i = 0; i < NVMCTRL_ROW_PAGES; i++) {
		int offset = i * NVMCTRL_PAGE_SIZE;
		do
			read_nvm_code = nvm_read_buffer(row_address + offset, row_buffer + offset, NVMCTRL_PAGE_SIZE);
		while (STATUS_OK != read_nvm_code);
	}
	printf("row: %04x\r\n", row_buffer);
	do
		read_nvm_code = nvm_erase_row(row_address);
	while (STATUS_OK != read_nvm_code);

	fw_status_t write_boot;
	memcpy(&write_boot, row_buffer + page_offset * NVMCTRL_PAGE_SIZE, sizeof(fw_status_t));
	write_boot.new_image_ready = 1;
	printf("update_boot_status write_boot.signature: %02x\r\n", write_boot.signature[0]);
	printf("update_boot_status write_boot.signature: %02x\r\n", write_boot.signature[1]);
	printf("update_boot_status write_boot.signature: %02x\r\n", write_boot.signature[2]);
	memcpy(row_buffer + (page_offset * NVMCTRL_PAGE_SIZE), &write_boot, sizeof(fw_status_t));

	memcpy(row_buffer + (page_offset * NVMCTRL_PAGE_SIZE), &write_boot, sizeof(fw_status_t));
	for (int i = 0; i < NVMCTRL_ROW_PAGES; i++) {
		int offset = i * NVMCTRL_PAGE_SIZE;
		do
			read_nvm_code = nvm_write_buffer(row_address + offset, row_buffer + offset, NVMCTRL_PAGE_SIZE);
		while (STATUS_OK != read_nvm_code);
	}
}

/**
 * calculates crc for firmware to ensure flash wrote correctly
 * @param  known_checksum checksum calculated during download
 * @return                true if checksums match
 */
bool verify_flash(crc32_t known_checksum)
{
	//TODO: write this
	crc32_t flash_checksum = 0;

	printf("starting seed: %04x\r\n", flash_checksum);
	enum status_code read_status;
	for (int i = 0; i < total_bytes_written / FLASH_ROW_SIZE; i++) {
		read_status = at25dfx_chip_read_buffer(&at25dfx_chip, FW1_ADDR + (i * FLASH_ROW_SIZE), write_row_buffer, FLASH_ROW_SIZE);
		if (STATUS_OK != read_status) {
			printf("verify_flash: error trying to read external flash. %d", read_status);
			return false;
		}
		//printf("verify_flash [flash_dump]: %d. %s\r\n", i, write_row_buffer);
		if (!flash_checksum)
			crc32_calculate(write_row_buffer, FLASH_ROW_SIZE, &flash_checksum);
		else
			crc32_recalculate(write_row_buffer, FLASH_ROW_SIZE, &flash_checksum);
		printf("%d. verify_flash: crc_val: %d\r\n", i, flash_checksum);
	}
	int remaining_bytes = total_bytes_written % FLASH_ROW_SIZE;
	printf("verify_flash: remaining bytes %d\r\n", remaining_bytes);
	read_status = at25dfx_chip_read_buffer(&at25dfx_chip, FW1_ADDR + (total_bytes_written - remaining_bytes), write_row_buffer, remaining_bytes);
	crc32_recalculate(write_row_buffer, remaining_bytes, &flash_checksum);
	printf("verify_flash: calculated crc32 val: %d\r\n", flash_checksum);
	return flash_checksum == known_checksum;
}

/**
 * erase given firmware in flash Memory
 * @param firmware_starter_address starting address of firmware
 */
static void erase_firmware_in_flash(uint32_t firmware_starter_address)
{
	if (firmware_starter_address == FW1_ADDR)
		firmware_starter_address = FW1_ERASE_ADDR;
	else if (firmware_starter_address == FW2_ADDR)
		firmware_starter_address = FW2_ERASE_ADDR;
	else
		//TODO: Throw error
		return;
	printf("FW1_ERASE_ADDR: %d\r\n", firmware_starter_address);
	//Clear flash for max firmware size here
	at25dfx_chip_wake(&at25dfx_chip);

	//check if chip is there
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		printf("store_file_packet: No flash chip.\r\n");
		return;
	}
	//max fw size is ~252k = 3x 64kb sectors 1x 32kb sector 7x4kb sectors
	//disable protection
	enum status_code status_val;
	//for (int i = 0; i < FW_MAX_SIZE / SIXTY_FOUR_KB; i++) {
	status_val = at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error clearing sector protect at address %d \r\n", SIXTY_FOUR_KB);
		return;
	}
	//}
	//erase block (sets to FF's)
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address, AT25DFX_BLOCK_SIZE_64KB);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address);
		return;
	}
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address + (SIXTY_FOUR_KB * 1), AT25DFX_BLOCK_SIZE_64KB);
	if (STATUS_OK != status_val) {
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address + (SIXTY_FOUR_KB * 1));
		return;
	}
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address + (SIXTY_FOUR_KB * 2), AT25DFX_BLOCK_SIZE_64KB);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address + (SIXTY_FOUR_KB * 2));
		return;
	}
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address + (SIXTY_FOUR_KB * 3) + (THIRTY_TWO_KB * 0), AT25DFX_BLOCK_SIZE_32KB);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address + (SIXTY_FOUR_KB * 3) + (THIRTY_TWO_KB * 0));
		return;
	}
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 0), AT25DFX_BLOCK_SIZE_4KB);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 0));
		return;
	}
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 1), AT25DFX_BLOCK_SIZE_4KB);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 1));
		return;
	}
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 2), AT25DFX_BLOCK_SIZE_4KB);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 2));
		return;
	}
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 3), AT25DFX_BLOCK_SIZE_4KB);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 3));
		return;
	}
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 4), AT25DFX_BLOCK_SIZE_4KB);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 4));
		return;
	}
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 5), AT25DFX_BLOCK_SIZE_4KB);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 5));
		return;
	}
	status_val = at25dfx_chip_erase_block(&at25dfx_chip, firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 6), AT25DFX_BLOCK_SIZE_4KB);
	if (STATUS_OK != status_val) {
		add_state(CANCELED);
		printf("store_file_packet: error erasing sector at address %d \r\n", firmware_starter_address + (SIXTY_FOUR_KB * 2) + (THIRTY_TWO_KB * 1) + (FOUR_KB * 3));
		return;
	}
	at25dfx_chip_sleep(&at25dfx_chip);
}

//TODO: write documentation on this
static void check_set_firmware_metadata(fw_header_t firmware_header)
{
	printf("new firmware version: %d\r\n", firmware_header.fw_version);
	printf("new firmware checksum: %04x\r\n", firmware_header.checksum);
	if (firmware_header.fw_version > FW_VERSION) {
		clear_state(GET_REQUESTED);
		clear_state(NOT_CHECKED);
		add_state(UPDATE_AVAILABLE);
		new_firmware_checksum = firmware_header.checksum;
		new_firmware_version = firmware_header.fw_version;
		return;
	} else {
		clear_state(NOT_CHECKED & GET_REQUESTED);
		add_state(UPDATE_NOT_AVAILABLE & COMPLETED);
		return;
	}
}

/**
 * puts the given packet in flash memory
 * @param data   data to put in flash memory
 * @param length size in bytes of data
 */
static void store_file_packet(char *data, uint32_t length)
{
	if ((data == NULL) || (length < 1)) {
		printf("store_file_packet: empty data.\r\n");
		return;
	}
	if (is_state_set(NOT_CHECKED)) {
		if (6 != length) {
			printf("store_file_packet [UPDATE_CHECK]: file [%d bytes] does not match header [%d bytes] size\r\n", length, sizeof(fw_header_t));
			return;
		}
		fw_header_t firmware_header;
		memcpy(&firmware_header.fw_version, data, 2);
		memcpy(&firmware_header.checksum, data + 2, 4);
		check_set_firmware_metadata(firmware_header);
		add_state(COMPLETED);
		return;
	} else if (is_state_set(UPDATE_AVAILABLE)) {
		enum status_code status_val;
		//hasn't started downloading yet, first packet. set values
		if (!is_state_set(DOWNLOADING)) {
			erase_firmware_in_flash(FW1_ADDR);
			received_file_size = 0;
			write_address = FW1_ADDR; //TODO: MAKE GENERIC SO CAN OTA CLI
			buffer_cursor = 0;
			total_bytes_written = 0;
			printf("starting val: ", crc_val);
			add_state(DOWNLOADING);
		}

		if (data != NULL) {
			int bytes_written = 0;
			//printf("length of packet: %d\r\n", length);
			while (bytes_written < length) {
				//TOOD: Check to see if should look at bytes_written + 256 or + 255
				uint16_t num_bytes_to_buffer = (bytes_written + FLASH_ROW_SIZE - 1 < length) ? ((FLASH_ROW_SIZE)-buffer_cursor) : length - bytes_written;
				//printf("store_file_packet: num_bytes_to_buffer %d\r\n", num_bytes_to_buffer);
				unsigned int data_cursor = data + bytes_written;
				memcpy(write_row_buffer + buffer_cursor, data_cursor, num_bytes_to_buffer);
				bytes_written += num_bytes_to_buffer;
				buffer_cursor = buffer_cursor + num_bytes_to_buffer;
				if (buffer_cursor == 0)
					buffer_cursor = (FLASH_ROW_SIZE - 1);
				if (buffer_cursor == (FLASH_ROW_SIZE - 1)) {
					status_val = at25dfx_chip_write_buffer(&at25dfx_chip, write_address, write_row_buffer, FLASH_ROW_SIZE);
					if (STATUS_OK != status_val) {
						add_state(CANCELED);
						printf("store_file_packet: error writing row at address %d \r\n", write_address);
						return;
					}
					buffer_cursor = 0;
					write_address += FLASH_ROW_SIZE;
#if DEBUG_LEVEL >= 3
					if (!global_test_counter)
						hexDump("hexDump", write_row_buffer, 256);

#endif
					if (!crc_val)
						crc32_calculate(write_row_buffer, FLASH_ROW_SIZE, &crc_val);
					else
						crc32_recalculate(write_row_buffer, FLASH_ROW_SIZE, &crc_val);

#if DEBUG_LEVEL >= 2
					printf("%d. store_file_packet [crc_calc]: %d\r\n", global_test_counter++, crc_val);
#endif
				}
			}

			total_bytes_written += bytes_written;
			received_file_size += length;
			printf("store_file_packet: received[%lu], file size[%lu]\r\n", (unsigned long)received_file_size, (unsigned long)http_file_size);
			if (received_file_size >= http_file_size) {
#if DEBUG_LEVEL >= 2
				printf("store_file_packet: leftover write_buffer %s\r\n", write_row_buffer);
#endif
				status_val = at25dfx_chip_write_buffer(&at25dfx_chip, write_address, write_row_buffer, buffer_cursor);
				if (STATUS_OK != status_val) {
					add_state(CANCELED);
					printf("store_file_packet: error writing row at address %d \r\n", write_address);
					return;
				}
				crc32_recalculate(write_row_buffer, buffer_cursor, &crc_val);
#if DEBUG_LEVEL >= 1
				printf("store_file_packet: buffer_cursor size: %d\r\n", buffer_cursor);
				printf("store_file_packet: calculated crc32 val from packets: %d\r\n", crc_val);
#endif
				bytes_written += buffer_cursor;
				printf("store_file_packet: file downloaded successfully.\r\n");
#if DEBUG_LEVEL >= 1
				printf("store_file_packet: received_file_size: %d\r\n", received_file_size);
				printf("store_file_packet: num bytes written to flash memory: %d\r\n", total_bytes_written);
#endif
				if (verify_flash(crc_val)) {
					printf("store_file_packet: flash successfully written with no errors\r\n");
				} else {
					printf("store_file_packet: flash corrupted.\r\n");
					add_state(CANCELED);
					return;
				}
				if (crc_val != new_firmware_checksum) {
					printf("store_file_packet: file checksums don't match.\r\n Expected checksum %04x\r\n Received checksum %04x\r\n", new_firmware_checksum, crc_val);
					add_state(CANCELED);
					return;
				} else {
					update_boot_status();
					write_firmware_metadata(FW1_ADDR, new_firmware_version, new_firmware_checksum, received_file_size);
					add_state(COMPLETED);
				}
				return;
			}
		}
	} else {
		//TODO: shouldn't get here
	}
}

/**
 * callback for http_client on state change
 * @param module_inst instantiated http_client struct
 * @param evt         event change (enum)
 * @param data        data from http_client
 */
static void http_client_callback(struct http_client_module *module_inst, int evt, union http_client_data *data)
{
	switch (evt) {
	case HTTP_CLIENT_CALLBACK_SOCK_CONNECTED: {
		printf("http_client_callback: HTTP client socket connected.\r\n");
		break;
	}
	case HTTP_CLIENT_CALLBACK_REQUESTED: {
		printf("http_client_callback: request completed.\r\n");
		add_state(GET_REQUESTED);
		break;
	}
	case HTTP_CLIENT_CALLBACK_RECV_RESPONSE: {
		printf("http_client_callback: received response %u data size %u\r\n",
		       (unsigned int)data->recv_response.response_code,
		       (unsigned int)data->recv_response.content_length);
		if ((unsigned int)data->recv_response.response_code == 200) {
			http_file_size = data->recv_response.content_length;
			received_file_size = 0;
		} else {
			add_state(CANCELED);
			return;
		}
		if (data->recv_response.content_length <= MTU_HTTP)
			store_file_packet(data->recv_response.content, data->recv_response.content_length);
		//add_state(COMPLETED);
		break;
	}
	case HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA: {
		store_file_packet(data->recv_chunked_data.data, data->recv_chunked_data.length);
		//if (data->recv_chunked_data.is_complete)
		//add_state(COMPLETED);
		break;
	}
	case HTTP_CLIENT_CALLBACK_DISCONNECTED: {
		printf("http_client_callback: disconnection reason:%d\r\n", data->disconnected.reason);

		/* If disconnect reason is equal to -ECONNRESET(-104),
		 * It means the server has closed the connection (timeout).
		 * This is normal operation.
		 */
		if (data->disconnected.reason == -EAGAIN) {
			/* Server has not responded. Retry immediately. */
			if (is_state_set(DOWNLOADING))
				clear_state(DOWNLOADING);

			if (is_state_set(GET_REQUESTED))
				clear_state(GET_REQUESTED);
			break;
		}
	}
	default: {
		break;
	}
	}
}

/**
 * http_client configuration setup
 */
void configure_http_client(void)
{
	struct http_client_config httpc_conf;
	int ret;

	http_client_get_config_defaults(&httpc_conf);
	httpc_conf.recv_buffer_size = MTU_HTTP;
	httpc_conf.timer_inst = &swt_module_inst;

	ret = http_client_init(&http_client_module_inst, &httpc_conf);
	if (ret < 0) {
		printf("configure_http_client: HTTP client initialization failed! (res %d)\r\n", ret);
		while (1) {
		}         /* Loop forever. */
	}

	http_client_register_callback(&http_client_module_inst, http_client_callback);
}

/**
 * TCP socket callback that re-routes to http_client
 * @param sock    open socket
 * @param evt     event change that pulled callback
 * @param evt_msg event data
 */
void socket_callback(SOCKET sock, uint8_t evt, void *evt_msg)
{
	http_client_socket_event_handler(sock, evt, evt_msg);
}
/**
 * callback to resolve http address to an IP address
 * @param pu8DomainName unknown??
 * @param u32ServerIP   unknown??
 */
void resolve_cb(uint8_t *pu8DomainName, uint32_t u32ServerIP)
{
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", pu8DomainName,
	       (int)IPV4_BYTE(u32ServerIP, 0), (int)IPV4_BYTE(u32ServerIP, 1),
	       (int)IPV4_BYTE(u32ServerIP, 2), (int)IPV4_BYTE(u32ServerIP, 3));
	http_client_socket_resolve_handler(pu8DomainName, u32ServerIP);
}

/**
 * Callback for wifi state changes
 * @param evt     event change
 * @param evt_msg information about wifi state change
 */
void wifi_callback(uint8_t evt, void *evt_msg)
{
	switch (evt) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED: {
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)evt_msg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_callback [M2M_WIFI_RESP_CON_STATE_CHANGED]: connected.\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_callback [M2M_WIFI_RESP_CON_STATE_CHANGED] disconnected.\r\n");
			clear_state(WIFI_CONNECTED);
			if (is_state_set(DOWNLOADING))
				clear_state(DOWNLOADING);

			if (is_state_set(GET_REQUESTED))
				clear_state(GET_REQUESTED);
			m2m_wifi_connect(current_wifi_config->ssid, strlen(current_wifi_config->ssid), current_wifi_config->auth_type, current_wifi_config->password, M2M_WIFI_CH_ALL);
		}

		break;
	}
	case M2M_WIFI_REQ_DHCP_CONF: {
		uint8_t *pu8IPAddress = (uint8_t *)evt_msg;
		printf("wifi_callback [M2M_WIFI_REQ_DHCP_CONF]: IP address is %u.%u.%u.%u\r\n",
		       pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		add_state(WIFI_CONNECTED);
		start_download();
		break;
	}
	default:
		printf("wifi_callback [default]: ERROR evt %d\r\n", evt);
	}
}

static void configure_timer(void)
{
	struct sw_timer_config swt_conf;

	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

void get_default_wifi_config(wifi_config *wifi_configuration)
{
	wifi_configuration->ssid = "AirPennNet-Device";
	wifi_configuration->auth_type = M2M_WIFI_SEC_WPA_PSK;
	wifi_configuration->password = "penn1740wifi";
	wifi_configuration->firmware_header_http_address = "http://www.seas.upenn.edu/~warcher/ese516/metadata.bin";
	wifi_configuration->firmware_http_address = "http://www.seas.upenn.edu/~warcher/ese516/test-firmware.bin";
}
void configure_wifi_module(wifi_config *wifi_configuration)
{
	init_state();
	add_state(STORAGE_READY);
	configure_timer();
	configure_http_client();
	nm_bsp_init();

	tstrWifiInitParam wifi_params;
	memset((uint8_t *)&wifi_params, 0, sizeof(tstrWifiInitParam));
	wifi_params.pfAppWifiCb = wifi_callback;
	int8_t ret = m2m_wifi_init(&wifi_params); // NOTE: m2m = machine to machine
	if (M2M_SUCCESS != ret) {
		printf("failed to initialize wifi parameters\r\n");
		return 0;
	}

	socketInit();
	registerSocketCallback(socket_callback, resolve_cb);
	printf("--- Welcome to ATWINC 1500 Connection Test ---\r\n");
	printf("main: connecting to WiFi AP %s...\r\n", wifi_configuration->ssid);
	ret = m2m_wifi_connect(wifi_configuration->ssid, strlen(wifi_configuration->ssid), wifi_configuration->auth_type, wifi_configuration->password, M2M_WIFI_CH_ALL);
	if (M2M_SUCCESS == ret) {
		printf("successfully connected\r\n");
		current_wifi_config = wifi_configuration;
	}
}

bool check_for_update()
{
	m2m_wifi_request_dhcp_client();
	while (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		sw_timer_task(&swt_module_inst);
	}
	if (is_state_set(UPDATE_AVAILABLE)) {
		clear_state(COMPLETED);
		return true;
	} else {
		return false;
	}
}
bool download_firmware()
{
	if (is_state_set(UPDATE_AVAILABLE)) {
		start_download();
		while (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
			/* Handle pending events from network controller. */
			m2m_wifi_handle_events(NULL);
			/* Checks the timer timeout. */
			sw_timer_task(&swt_module_inst);
		}
		if (is_state_set(COMPLETED))
			return true;
		else
			return false;
	} else {
		return false;
	}
}
