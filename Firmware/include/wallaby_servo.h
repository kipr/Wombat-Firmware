#ifndef WALLABY_SERVO_H_
#define WALLABY_SERVO_H_

#include "stm32f4xx.h"

void configServoPin(uint32_t pin, GPIO_TypeDef* port);

void TIM3_Configuration(void);

void TIM9_Configuration(void);

void TIM3_IRQHandler(void);

void TIM1_BRK_TIM9_IRQHandler(void);

#endif