/*
 * debug.h
 *
 * Created: 4/19/2018 6:09:47 PM
 *  Author: William
 */ 


#ifndef DEBUG_H_
#define DEBUG_H_

static void hexDump(char *desc, void *addr, int len);
static void flash_dump(struct at25dfx_chip_module *at25dfx_chip, int starting_address, int num_bytes);

#endif /* DEBUG_H_ */