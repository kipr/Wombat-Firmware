#include "stm32f4xx.h"

#include "wallaby_pid.h"

void configMotorPin(uint32_t pin, GPIO_TypeDef* port);

void update_motor_mode(uint32_t dir1_pin, uint32_t dir2_pin, GPIO_TypeDef* dir1_port, GPIO_TypeDef* dir2_port, uint8_t drive_code);

void update_motor_modes();

void idle_motor_dirs();

void motor_update(int16_t bemf_val, int16_t bemf_val_filt, pid_struct * pids, uint8_t channel, uint8_t motor_mode);


void TIM1_Configuration(void);
void TIM8_Configuration(void);


void TIM1_CC_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
