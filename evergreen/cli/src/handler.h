#include <asf.h>
#include <usart.h>
#include "delay.h"
#include "otafu.h"

//pwm
#define CONF_PWM_MODULE      TCC1
#define CONF_PWM_CHANNEL     0
#define CONF_PWM_OUTPUT      0
#define CONF_PWM_OUT_PIN     PIN_PA06E_TCC1_WO0
#define CONF_PWM_OUT_MUX     PINMUX_PA06E_TCC1_WO0

#define MOISTURE_ANA_PIN ADC_POSITIVE_INPUT_PIN0

void input_handle(int argc, char **argv);

void configure_usart(void);

void configure_port_pins_set(int pin);
void configure_port_pins_get(int pin);

void configure_adc(int pin);

static void configure_tcc_pwm(void);
void ramp_tcc_pwm(int duty);

void run_pump(int duration);

float get_moisture(void);

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


struct adc_module adc_instance;
struct tcc_module tcc_instance_pwm;
wifi_config new_wifi_configuration;


