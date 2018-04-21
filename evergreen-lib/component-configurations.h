/*
 * component_configurations.h
 *
 * Created: 4/19/2018 6:14:30 PM
 *  Author: William
 */


#ifndef COMPONENT-CONFIGURATIONS_H_
#define COMPONENT-CONFIGURATIONS_H_

/**
 * configure rx/tx usart for 115200 baud on usb pinout
 */
void configure_usart();

/**
 * sets up default flash configuration for AT25DF081A-SSH-T
 */
void configure_flash();

/**
 * configure non volatile memory struct
 */
void configure_nvm();


#endif /* COMPONENT-CONFIGURATIONS_H_ */
