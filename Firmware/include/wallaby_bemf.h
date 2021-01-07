#ifndef WALLABY_BEMF_H_
#define WALLABY_BEMF_H_

#include "stm32f4xx.h"

void configBEMFPin(uint32_t pin, GPIO_TypeDef* port);

void update_bemfs(int32_t * bemf_vals, int32_t * bemf_vals_filt);

#endif