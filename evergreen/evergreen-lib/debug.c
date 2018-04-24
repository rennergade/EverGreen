/*
 * debug.c
 *
 * Created: 4/19/2018 6:12:01 PM
 *  Author: William
 */
#include <asf.h>
#include "debug.h"
#include "memory-app-definitions.h"

void hexDump(char *desc, void *addr, int len)
{
	int i;
	unsigned char buff[17];
	unsigned char *pc = (unsigned char *)addr;

	// Output description if given.
	if (desc != NULL)
		printf("%s:\r\n", desc);

	if (len == 0) {
		printf("  ZERO LENGTH\r\n");
		return;
	}
	if (len < 0) {
		printf("  NEGATIVE LENGTH: %i\r\n", len);
		return;
	}

	// Process every byte in the data.
	for (i = 0; i < len; i++) {
		// Multiple of 16 means new line (with line offset).

		if ((i % 16) == 0) {
			// Just don't print ASCII for the zeroth line.
			if (i != 0)
				printf("  %s\r\n", buff);

			// Output the offset.
			printf("  %04x ", i);
		}

		// Now the hex code for the specific character.
		printf(" %02x", pc[i]);

		// And store a printable ASCII character for later.
		if ((pc[i] < 0x20) || (pc[i] > 0x7e))
			buff[i % 16] = '.';
		else
			buff[i % 16] = pc[i];
		buff[(i % 16) + 1] = '\0';
	}

	// Pad out last line if not exactly 16 characters.
	while ((i % 16) != 0) {
		printf("   ");
		i++;
	}

	// And print the final ASCII bit.
	printf("  %s\r\n", buff);
}

void flash_dump(struct at25dfx_chip_module *at25dfx_chip, int starting_address, int num_bytes)
{
	uint8_t write_row_buffer[FLASH_ROW_SIZE];

	enum status_code read_status;

	for (int i = 0; i < num_bytes / FLASH_ROW_SIZE; i++) {
		read_status = at25dfx_chip_read_buffer(&at25dfx_chip, starting_address + (i * FLASH_ROW_SIZE), write_row_buffer, FLASH_ROW_SIZE);
		if (STATUS_OK != read_status) {
			printf("verify_flash: error trying to read external flash. %d", read_status);
			return false;
		}
		printf("flash_dump: %d. %s\r\n", i, write_row_buffer);
	}
	int remaining_bytes = num_bytes % FLASH_ROW_SIZE;
	read_status = at25dfx_chip_read_buffer(&at25dfx_chip, starting_address + num_bytes - remaining_bytes, write_row_buffer, remaining_bytes);
	if (STATUS_OK != read_status) {
		printf("verify_flash: error trying to read external flash. %d", read_status);
		return false;
	}
	printf("flash_dump: %s\r\n", write_row_buffer);

	at25dfx_chip_sleep(&at25dfx_chip);
}
