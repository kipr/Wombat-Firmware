#include "wallaby_motor.h"

#include "wallaby.h"

#include "string.h" // memset


void configMotorPin(uint32_t pin, GPIO_TypeDef* port)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(port, &GPIO_InitStructure);
}



void update_motor_mode(uint32_t dir1_pin, uint32_t dir2_pin, GPIO_TypeDef* dir1_port, GPIO_TypeDef* dir2_port, uint8_t drive_code)
{

    switch(drive_code)
    {
        case 0:
            dir1_port->BSRRH |= dir1_pin; 
            dir2_port->BSRRH |= dir2_pin;
            break;
        case 1:
            dir1_port->BSRRL |= dir1_pin; 
            dir2_port->BSRRH |= dir2_pin;
            break;
        case 2:
            dir1_port->BSRRH |= dir1_pin; 
            dir2_port->BSRRL |= dir2_pin;
            break;
        case 3:
            dir1_port->BSRRL |= dir1_pin; 
            dir2_port->BSRRL |= dir2_pin;
            break;
        default:
            break;
    }
}

void update_motor_modes()
{
    uint8_t motor_0_mode = (aTxBuffer[REG_RW_MOT_DIRS] & 0b00000011);
    uint8_t motor_1_mode = (aTxBuffer[REG_RW_MOT_DIRS] & 0b00001100) >> 2;
    uint8_t motor_2_mode = (aTxBuffer[REG_RW_MOT_DIRS] & 0b00110000) >> 4;
    uint8_t motor_3_mode = (aTxBuffer[REG_RW_MOT_DIRS] & 0b11000000) >> 6;

    update_motor_mode(MOT0_DIR1, MOT0_DIR2, MOT0_DIR1_PORT, MOT0_DIR2_PORT, motor_0_mode);
    update_motor_mode(MOT1_DIR1, MOT1_DIR2, MOT1_DIR1_PORT, MOT1_DIR2_PORT, motor_1_mode);
    update_motor_mode(MOT2_DIR1, MOT2_DIR2, MOT2_DIR1_PORT, MOT2_DIR2_PORT, motor_2_mode);
    update_motor_mode(MOT3_DIR1, MOT3_DIR2, MOT3_DIR1_PORT, MOT3_DIR2_PORT, motor_3_mode);
}

void idle_motor_dirs()
{
    aTxBuffer[REG_RW_MOT_MODES] = 0;
}



void motor_update(int16_t bemf_val, int16_t bemf_val_filt, pid_struct * pids, uint8_t channel, uint8_t motor_mode)
{
    static const uint8_t MOT_SP_REG_STRIDE = 2;
    static const uint8_t MOT_PWM_REG_STRIDE = 2;
    static const uint8_t MOT_POS_REG_STRIDE = 4; // 32 bit position
    static const uint8_t MOT_POS_GOAL_REG_STRIDE = 4;
    
    if ((bemf_val < 8) && (bemf_val > -8)) bemf_val = 0;

    const uint16_t mot_pos_address = REG_RW_MOT_0_B3 + MOT_POS_REG_STRIDE * channel;
    int32_t pos = ((int32_t)aTxBuffer[mot_pos_address] << 24) | ((int32_t)aTxBuffer[mot_pos_address+1] << 16) | ((int32_t)aTxBuffer[mot_pos_address+2] << 8) | ((int32_t)aTxBuffer[mot_pos_address+3]);
    pos += bemf_val_filt;
    aTxBuffer[mot_pos_address]   = (pos & 0xFF000000) >> 24;
    aTxBuffer[mot_pos_address+1] = (pos & 0x00FF0000) >> 16;
    aTxBuffer[mot_pos_address+2] = (pos & 0x0000FF00) >> 8;
    aTxBuffer[mot_pos_address+3] = (pos & 0x000000FF);

        // TODO: more concise way of this?
        // something like   ~(0b11 << (6- 2*channel))
        uint8_t dir_mask = 0xff;
        switch(channel)
        {
            case 0:
                dir_mask = 0b11111100;
                break;
            case 1:
                dir_mask = 0b11110011;
                break;
            case 2:
                dir_mask = 0b11001111;
                break;
            case 3:
                dir_mask = 0b00111111;
                break;
            default:
                break;
        }


    uint8_t motor_done = aTxBuffer[REG_RW_MOT_DONE] & (1 << channel);
    uint8_t motor_stop = aTxBuffer[REG_RW_MOT_SRV_ALLSTOP] & (1 << channel);

    // TODO: maybe move this logic so it integrates with code below better
    if (motor_stop)
    {
        uint16_t ucmd = 0;

        // passive braking
        aTxBuffer[REG_RW_MOT_DIRS] = (aTxBuffer[REG_RW_MOT_DIRS] & dir_mask) | (0b00 << (2*channel));

        // TODO multi channel is a little messy here
        aTxBuffer[REG_RW_MOT_0_PWM_H + MOT_PWM_REG_STRIDE * channel] = (ucmd & 0xFF00) >> 8;
        aTxBuffer[REG_RW_MOT_0_PWM_L + MOT_PWM_REG_STRIDE * channel] = (ucmd & 0x00FF);

        update_motor_modes();
        return;
    }

    if (motor_mode == 0)
    {
        // PWM mode
    }
    else if (motor_done ==0)
    {
        // PID Control Needed   (MAV, MTP, MRP)
        const uint16_t goal_address = REG_RW_MOT_0_SP_H + MOT_SP_REG_STRIDE * channel;
        int32_t goal = (((int16_t)(aTxBuffer[goal_address])) << 8) | ((int16_t)(aTxBuffer[goal_address+1]));
        if (aTxBuffer[goal_address] & 0b10000000) goal |= 0xFFFF0000;// TODO: cleanup negative goal handling
        const int32_t current = (int32_t)bemf_val;
    
        const float pErr =  (float)(goal-current); //(float)(goal - current);
        const float dErr = pErr - pids->prevErr;

        pids->prevErr = pErr;
        pids->iErr += pErr;

        // clamp error integral
        static const float max_iErr = 10000.0f; 
        if (pids->iErr > max_iErr)
        {
            pids->iErr = max_iErr;
        }
        else if (pids->iErr < -max_iErr)
        {
            pids->iErr = -max_iErr;
        }

        int32_t cmd = (int32_t)(pids->kP*pErr + pids->kI*pids->iErr + pids->kD*dErr);

        // Move To Position or  Move Relative Position goal checks
        if (motor_mode == MOT_MODE_MTP || motor_mode == MOT_MODE_MRP)
        {
            const uint16_t mot_pos_goal_address = REG_W_MOT_0_GOAL_B3 + MOT_POS_GOAL_REG_STRIDE * channel;
            int32_t pos_goal = ((int32_t)aTxBuffer[mot_pos_goal_address] << 24) | ((int32_t)aTxBuffer[mot_pos_goal_address+1] << 16) | ((int32_t)aTxBuffer[mot_pos_goal_address+2] << 8) | ((int32_t)aTxBuffer[mot_pos_goal_address+3]);

            if (goal < 0)
            {
                if(pos < pos_goal){
                    cmd = 0;
                    aTxBuffer[REG_RW_MOT_DONE] |= (1 << channel);
                }
            }
            else
            {
                if(pos > pos_goal){
                    cmd = 0;
                    aTxBuffer[REG_RW_MOT_DONE] |= (1 << channel);
                }
            }
        }

        // clamp output signal
        static const uint16_t max_cmd = 400;
        if (cmd > max_cmd) cmd = max_cmd;
        if (cmd < -max_cmd) cmd = -max_cmd;

        // handle direction
        if (cmd < 0)
        {
            cmd = -cmd;
            //motor_mode[0] = 2; // reverse
            aTxBuffer[REG_RW_MOT_DIRS] =  (aTxBuffer[REG_RW_MOT_DIRS] & dir_mask) | (0b10 << (2*channel));
        }
        else
        {
            //motor_mode[0] = 1; //forward
            aTxBuffer[REG_RW_MOT_DIRS] = (aTxBuffer[REG_RW_MOT_DIRS] & dir_mask) | (0b01 << (2*channel));
        }

        // TODO: adjust for a physical deadband?

        // TODO: adjust or nonlinearity?

        // TODO: add simulated deadband?




        // set command
        uint16_t ucmd = (uint16_t)cmd;

        // TODO multi channel is a little messy here
        aTxBuffer[REG_RW_MOT_0_PWM_H + MOT_PWM_REG_STRIDE * channel] = (ucmd & 0xFF00) >> 8;
        aTxBuffer[REG_RW_MOT_0_PWM_L + MOT_PWM_REG_STRIDE * channel] = (ucmd & 0x00FF);
    }

    update_motor_modes();
}



void TIM1_Configuration(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 
  memset(&TIM_OCInitStructure, 0, sizeof TIM_OCInitStructure);
  memset(&TIM_TimeBaseStructure, 0, sizeof TIM_TimeBaseStructure);

  /* Time base configuration - SystemCoreClock = 168000000 for 168 MHz board */
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (((SystemCoreClock / 10000000)) - 1); // Shooting for 1 MHz, (1us)
  TIM_TimeBaseStructure.TIM_Period = 400 - 1; // 1 MHz / 500 = 2 kHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0xFF;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
 
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM1, ENABLE);
 
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 200; // Servo Top-Center
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
 
  /* Output Compare PWM1 Mode configuration: Channel2 PD.13 */
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
 
  /* Output Compare PWM1 Mode configuration: Channel3 PD.14 */
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
 
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
 
  /* TIM4 enable counter */
  TIM_Cmd(TIM1, ENABLE);

  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


void TIM8_Configuration(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  memset(&TIM_OCInitStructure, 0, sizeof TIM_OCInitStructure);
  memset(&TIM_TimeBaseStructure, 0, sizeof TIM_TimeBaseStructure);

  /* Time base configuration - SystemCoreClock = 168000000 for 168 MHz board */
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (((SystemCoreClock / 10000000)) - 1); // Shooting for 1 MHz, (1us)
  TIM_TimeBaseStructure.TIM_Period = 400 - 1; // 1 MHz / 20000 = 50 Hz (20ms)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0xFF;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
 
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM8, ENABLE);
 
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 200; // Servo Top-Center
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
 
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);
 
  /* TIM4 enable counter */
  TIM_Cmd(TIM8, ENABLE);

  TIM_CtrlPWMOutputs(TIM8, ENABLE);
}





void TIM1_CC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1  ) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        uint32_t mot0_cmd = (((uint32_t)(aTxBuffer[REG_RW_MOT_0_PWM_H])) << 8) | ((uint32_t)(aTxBuffer[REG_RW_MOT_0_PWM_L]));
        uint32_t mot1_cmd = (((uint32_t)aTxBuffer[REG_RW_MOT_1_PWM_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_MOT_1_PWM_L]);
        uint32_t mot2_cmd = (((uint32_t)aTxBuffer[REG_RW_MOT_2_PWM_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_MOT_2_PWM_L]);
        
        TIM_SetCompare1(TIM1, mot1_cmd);
        TIM_SetCompare2(TIM1, mot0_cmd);      
        TIM_SetCompare3(TIM1, mot2_cmd);
    }
}



void TIM8_CC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);

        uint32_t mot3_cmd = (((uint32_t)aTxBuffer[REG_RW_MOT_3_PWM_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_MOT_3_PWM_L]);

        TIM_SetCompare1(TIM8, mot3_cmd);
    }
}
