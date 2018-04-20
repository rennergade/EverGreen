#include <stdint.h>


#define HDC1080_ADDR 0x40

#define STAT_OK 0
#define STAT_ERR 1


/* Register addresses */
#define HDC1080_TEMPERATURE				0x00
#define HDC1080_HUMIDITY 					0x01
#define HDC1080_CONFIG						0x02

#define HDC1080_RH_RES_14					0x00
#define HDC1080_RH_RES_11					0x01
#define HDC1080_RH_RES8						0x02

#define HDC1080_T_RES_14					0x00
#define HDC1080_T_RES_11					0x01


int hdc1080_read_reg(uint16_t delay, uint8_t reg, uint16_t *val);
int hdc1080_write_reg(uint8_t reg, uint16_t val);
int hdc1080_measure(double *temperature, double *humidity);
