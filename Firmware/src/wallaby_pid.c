#include "wallaby_pid.h"

#include "wallaby.h"


void init_pid_struct(pid_struct * pids, uint8_t channel)
{
    pids->prevErr = 0.0f;
    pids->iErr = 0.0f;
    update_pid_coeffs_from_reg(pids, channel);
}



void set_pid_reg_defaults(uint8_t channel)
{
    uint16_t default_p = 2000;
    uint16_t default_pd = 1000;
    uint16_t default_i = 200;
    uint16_t default_id = 1000;
    uint16_t default_d = 1;
    uint16_t default_dd = 1000;

    const uint8_t reg_addys_per_motor = 3*4; // 3 coeffs, each are 4 bytes (2 for numerator, 2 denominator)

    uint8_t p_addr = REG_W_PID_0_P_H + channel * reg_addys_per_motor;
    uint8_t pd_addr = p_addr + 2;
    uint8_t i_addr = pd_addr + 2;
    uint8_t id_addr = i_addr + 2;
    uint8_t d_addr = id_addr + 2;
    uint8_t dd_addr = d_addr + 2;

    aTxBuffer[p_addr]     = (uint8_t)((default_p & 0xFF00) >> 8);
    aTxBuffer[p_addr+1]   = (uint8_t)(default_p & 0x00FF);
    aTxBuffer[pd_addr]    = (uint8_t)((default_pd & 0xFF00) >> 8);
    aTxBuffer[pd_addr+1]  = (uint8_t)(default_pd & 0x00FF);

    aTxBuffer[i_addr]     = (uint8_t)((default_i & 0xFF00) >> 8);
    aTxBuffer[i_addr+1]   = (uint8_t)(default_i & 0x00FF);
    aTxBuffer[id_addr]    = (uint8_t)((default_id & 0xFF00) >> 8);
    aTxBuffer[id_addr+1]  = (uint8_t)(default_id & 0x00FF);

    aTxBuffer[d_addr]     = (uint8_t)((default_d & 0xFF00) >> 8);
    aTxBuffer[d_addr+1]   = (uint8_t)(default_d & 0x00FF);
    aTxBuffer[dd_addr]    = (uint8_t)((default_dd & 0xFF00) >> 8);
    aTxBuffer[dd_addr+1]  = (uint8_t)(default_dd & 0x00FF);
}




void update_pid_coeffs_from_reg(pid_struct * pids, uint8_t channel)
{
        const uint8_t reg_addys_per_motor = 3*4; // 3 coeffs, each are 4 bytes (2 for numerator, 2 denominator)

        uint8_t p_addr = REG_W_PID_0_P_H + channel * reg_addys_per_motor;
        uint8_t pd_addr = p_addr + 2;
        uint8_t i_addr = pd_addr + 2;
        uint8_t id_addr = i_addr + 2;
        uint8_t d_addr = id_addr + 2;
        uint8_t dd_addr = d_addr + 2;

        uint16_t p =  (((uint16_t)(aTxBuffer[p_addr]))  << 8) | ((uint16_t)(aTxBuffer[p_addr+1]));
        uint16_t pd = (((uint16_t)(aTxBuffer[pd_addr])) << 8) | ((uint16_t)(aTxBuffer[pd_addr+1]));
        uint16_t i =  (((uint16_t)(aTxBuffer[i_addr]))  << 8) | ((uint16_t)(aTxBuffer[i_addr+1]));
        uint16_t id = (((uint16_t)(aTxBuffer[id_addr])) << 8) | ((uint16_t)(aTxBuffer[id_addr+1]));
        uint16_t d =  (((uint16_t)(aTxBuffer[d_addr]))  << 8) | ((uint16_t)(aTxBuffer[d_addr+1]));
        uint16_t dd = (((uint16_t)(aTxBuffer[dd_addr])) << 8) | ((uint16_t)(aTxBuffer[dd_addr+1]));

        pids->kP = (float)p / (float)pd;
        pids->kI = (float)i / (float)id;
        pids->kD = (float)d / (float)dd;
}