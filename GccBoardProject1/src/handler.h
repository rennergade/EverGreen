#include <asf.h>
#include <usart.h>
#include "delay.h"

//defines
//I2C
#define CONF_I2C_MASTER_MODULE_TEMP SERCOM3
#define CONF_I2C_MASTER_MODULE_LUX SERCOM0

//pwm
#define CONF_PWM_MODULE      TCC1
#define CONF_PWM_CHANNEL     0
#define CONF_PWM_OUTPUT      0
#define CONF_PWM_OUT_PIN     PIN_PA06E_TCC1_WO0
#define CONF_PWM_OUT_MUX     PINMUX_PA06E_TCC1_WO0

#define MOISTURE_ANA_PIN ADC_POSITIVE_INPUT_PIN0



#define DATA_LENGTH 8

//FLASH defines
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





void input_handle(int argc, char **argv);

void configure_usart(void);

void configure_i2c_temp(void);
void configure_i2c_lux(void);

void i2c_write_complete_callback_tsl(struct i2c_master_module *const module);
void i2c_write_complete_callback_hdc(struct i2c_master_module *const module);

void configure_i2c_callbacks_tsl(void);
void configure_i2c_callbacks_hdc(void);



void configure_port_pins_set(int pin);
void configure_port_pins_get(int pin);

void configure_adc(int pin);

static void configure_tcc_pwm(void);
void ramp_tcc_pwm(int duty);

void run_pump(int duration);

float get_moisture(void);

void flash_test(void);

void led1_on(void);
void led1_off(void);
void led2_on(void);
void led2_off(void);

void boost_enable(void);
void boost_disable(void);
void relay1_enable(void);
void relay1_disable(void);
void relay2_enable(void);
void relay2_disable(void);
void gpio5_enable(void);
void gpio5_disable(void);








//flash inits
static uint8_t read_buffer[AT25DFX_BUFFER_SIZE];
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
struct spi_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;

crc32_t crc1;
crc32_t crc2;


//usart adc i2c inits
struct usart_module usart_instance;
struct adc_module adc_instance;
static uint8_t rd_buffer[DATA_LENGTH];
struct i2c_master_module i2c_tsl_instance;
struct i2c_master_module i2c_hdc_instance;

//pwm init
struct tcc_module tcc_instance_pwm;




static uint8_t wr_buffer[DATA_LENGTH] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};
static uint8_t wr_buffer_reversed[DATA_LENGTH] = {
	0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
};

static struct i2c_master_packet wr_packet = {
	.address          = 0,
	.data_length      = DATA_LENGTH,
	.data             = wr_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};

static struct i2c_master_packet rd_packet = {
	.address          = 0,
	.data_length      = DATA_LENGTH,
	.data             = rd_buffer,
	.ten_bit_address  = false,
	.high_speed       = false,
	.hs_master_code   = 0x00,
};


