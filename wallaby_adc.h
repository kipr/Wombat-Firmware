#include "stm32f4xx.h"

void configAnalogInPin(uint32_t pin, GPIO_TypeDef* port, uint8_t pullup_enable);

void config_adc_in_from_regs();

uint16_t slow_adc(ADC_TypeDef * bus, uint8_t channel);

int16_t adc_update();