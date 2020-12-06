#ifndef WALLABY_PID_H_
#define WALLABY_PID_H_

#include "stm32f4xx.h"

typedef struct pid_struct_
{
    float kP;
    float kI;
    float kD;
    float prevErr;
    float iErr;

} pid_struct;


void init_pid_struct(pid_struct * pids, uint8_t channel);

void set_pid_reg_defaults(uint8_t channel);

void update_pid_coeffs_from_reg(pid_struct * pids, uint8_t channel);

#endif