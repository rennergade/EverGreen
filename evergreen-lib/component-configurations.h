/*
 * component_configurations.h
 *
 * Created: 4/19/2018 6:14:30 PM
 *  Author: William
 */


#ifndef COMPONENT-CONFIGURATIONS_H_
#define COMPONENT-CONFIGURATIONS_H_

#ifdef STDIO_SERIAL_H_INCLUDED
/**
 * configure rx/tx usart for 115200 baud on usb pinout
 */
void configure_usart();
#endif
#ifdef AT25DFX_H
/**
 * sets up default flash configuration for AT25DF081A-SSH-T
 */
void configure_flash();
#endif

#ifdef NVM_H_INCLUDED
/**
 * configure non volatile memory struct
 */
void configure_nvm();
#endif

#endif /* COMPONENT-CONFIGURATIONS_H_ */
