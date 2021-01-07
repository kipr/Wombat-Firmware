#ifndef WALLABY_DIG_H_
#define WALLABY_DIG_H

#include "stm32f4xx.h"

void configDigitalInPin(uint32_t pin, GPIO_TypeDef* port);

void configDigitalOutPin(uint32_t pin, GPIO_TypeDef* port);


void update_dig_pin_from_reg(uint32_t pin, GPIO_TypeDef* port, uint8_t output_enable, uint8_t pullup_enable);

void update_dig_pin_configs();

void update_dig_pin(uint32_t pin, GPIO_TypeDef* port, uint8_t output_enable, uint8_t out_val, uint8_t pin_num);

void update_dig_pins();


#endif