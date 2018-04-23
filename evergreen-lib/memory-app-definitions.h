/*
 * memory_app_definitions.h
 *
 * Created: 4/19/2018 6:04:59 PM
 *  Author: William
 */


#ifndef MEMORY-APP-DEFINITIONS_H_
#define MEMORY-APP-DEFINITIONS_H_

//UART Macros
#define BAUD_RATE 115200

//Memory Location Macros
#define FW1_ERASE_ADDR 0x000000 /// Sector to start erasing at for 64kb
#define FW2_ERASE_ADDR 0x3FFFC /// Sector to start erasing at for 64kb
#define FW1_HEADER_ADDR 0x001000                /// Header for current or new firmware
#define FW2_HEADER_ADDR 0x040000                /// Header for safe CLI
#define FW1_ADDR 0x002000                       /// New/current firmware
#define FW2_ADDR 0x041000                       /// CLI firmware
#define FW_MAX_SIZE 0x3DFFC                    /// Max size of each firmware
#define BOOT_STATUS_ADDR 0x3FC0                 /// Boot status address in internal NVM
#define SIXTY_FOUR_KB 0x00FFFF
#define THIRTY_TWO_KB SIXTY_FOUR_KB / 2
#define FOUR_KB THIRTY_TWO_KB / 8

//Flash Macros
#ifdef AT25DFX_H
#define FLASH_ROW_SIZE 256              /// number of bytes per row in flash memory
#define AT25DFX_SPI SERCOM1             /// sercom for external flash
#define AT25DFX_MEM_TYPE AT25DFX_081A   /// what chip is it
#define AT25DFX_SPI_PINMUX_SETTING SPI_SIGNAL_MUX_SETTING_E
#define AT25DFX_SPI_PINMUX_PAD0 PINMUX_PA16C_SERCOM1_PAD0
#define AT25DFX_SPI_PINMUX_PAD1 PINMUX_UNUSED
#define AT25DFX_SPI_PINMUX_PAD2 PINMUX_PA18C_SERCOM1_PAD2
#define AT25DFX_SPI_PINMUX_PAD3 PINMUX_PA19C_SERCOM1_PAD3
#define AT25DFX_CS PIN_PA07
#define AT25DFX_CLOCK_SPEED 1000000 /// spi clock speed
#endif
/**
 * @struct fw_status
 * @brief struct to store bootloader information and statuses
 * @details keeps mbr signature to ensure data integrity (ie not written over)
 *          as well as flags to install new/updated application and the current bootloader information
 * @var fw_status::signature
 * signature used as canary value to ensure integrity (like MBR signature)
 * @var fw_status::new_image_ready
 * flag that is set when there is a new image to be flashed in the applcation Section
 * @var fw_status::bl_version current version of the bootloader, follows same format as application versioning
 * @typedef fw_status_t typed verison of @ref fw_status
 */
typedef struct fw_status {
	uint8_t		signature[3];           ///set values that confirm bootloader is still intact
	uint8_t		new_image_ready;        /// are we ready to write a new image?
	uint16_t	bl_version;							/// version of bootloader
	uint16_t	fw_version;
} fw_status_t;


/**
 * @struct fw_header
 * @brief struct that contains fw_version and checksum information for flashing
 * @var fw_header::fw_version
 * version of the firmware that corresponds to the header
 * @var fw_header::checksum
 * crc32 checksum for the firmware to verify correct flash
 * @typedef fw_header_t typed version of @ref fw_header
 */
typedef struct fw_header {
	uint16_t	fw_version;     /// firmware version
	uint32_t	checksum;       /// precomputed checksum value to verify
	uint32_t	size; 					/// size of firmware in bytes
} fw_header_t;

#endif /* MEMORY-APP-DEFINITIONS_H_ */
