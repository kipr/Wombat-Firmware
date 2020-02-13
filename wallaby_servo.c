#include "wallaby_servo.h"
#include "wallaby.h"

#include "stm32f4xx.h"


void configServoPin(uint32_t pin, GPIO_TypeDef* port)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(port, &GPIO_InitStructure);
}

void TIM3_Configuration(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 
  /* Time base configuration - SystemCoreClock = 168000000 for 168 MHz board */
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (((SystemCoreClock / 1000000) / 2) - 1); // Shooting for 1 MHz, (1us)
  TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 1 MHz / 20000 = 50 Hz (20ms)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM3, ENABLE);
 
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1500; // Servo Top-Center
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  /* Output Compare PWM1 Mode configuration: Channel2 PD.13 */
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
 
  /* Output Compare PWM1 Mode configuration: Channel3 PD.14 */
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
 
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
 
  /* TIM4 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}


void TIM9_Configuration(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 
  /* Time base configuration - SystemCoreClock = 168000000 for 168 MHz board */
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (((SystemCoreClock / 1000000)) - 1); // Shooting for 1 MHz, (1us)
  TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 1 MHz / 20000 = 50 Hz (20ms)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
 
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM9, ENABLE);
 
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1500; // Servo Top-Center
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TIM9, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
 
  /* Output Compare PWM1 Mode configuration: Channel2 PD.13 */
  TIM_OC2Init(TIM9, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

  /* TIM Interrupts enable */
  TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);
 
  /* TIM4 enable counter */
  TIM_Cmd(TIM9, ENABLE);
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        uint32_t servo0_cmd = (((uint32_t)aTxBuffer[REG_RW_SERVO_0_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_SERVO_0_L]);
        uint32_t servo1_cmd = (((uint32_t)aTxBuffer[REG_RW_SERVO_1_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_SERVO_1_L]);

        if (aTxBuffer[REG_RW_MOT_SRV_ALLSTOP] & 0b00010000) servo0_cmd = 0;
        if (aTxBuffer[REG_RW_MOT_SRV_ALLSTOP] & 0b00100000) servo1_cmd = 0;

        TIM_SetCompare2(TIM3, servo1_cmd);
        TIM_SetCompare3(TIM3, servo0_cmd);
    }
}


void TIM1_BRK_TIM9_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM9, TIM_IT_Update);

        uint32_t servo2_cmd = (((uint32_t)aTxBuffer[REG_RW_SERVO_2_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_SERVO_2_L]);
        uint32_t servo3_cmd = (((uint32_t)aTxBuffer[REG_RW_SERVO_3_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_SERVO_3_L]);

        if (aTxBuffer[REG_RW_MOT_SRV_ALLSTOP] & 0b01000000)
        {
            servo2_cmd = 0;
        }
        if (aTxBuffer[REG_RW_MOT_SRV_ALLSTOP] & 0b10000000) servo3_cmd = 0;

        TIM_SetCompare1(TIM9, servo3_cmd);
        TIM_SetCompare2(TIM9, servo2_cmd);
    }
}
