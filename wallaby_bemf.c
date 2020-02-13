#include "wallaby_bemf.h"

#include "wallaby.h"


void configBEMFPin(uint32_t pin, GPIO_TypeDef* port) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(port, &GPIO_InitStructure);
}




void update_bemfs(int32_t * bemf_vals, int32_t * bemf_vals_filt)
{
    int32_t adc_motor_vals[8];
    adc_motor_vals[0] = slow_adc(MOT0_BEMF_H_ADX, MOT0_BEMF_H_CHAN);
    adc_motor_vals[1] = slow_adc(MOT0_BEMF_L_ADX, MOT0_BEMF_L_CHAN);
    adc_motor_vals[2] = slow_adc(MOT1_BEMF_H_ADX, MOT1_BEMF_H_CHAN);
    adc_motor_vals[3] = slow_adc(MOT1_BEMF_L_ADX, MOT1_BEMF_L_CHAN);
    adc_motor_vals[4] = slow_adc(MOT2_BEMF_H_ADX, MOT2_BEMF_H_CHAN);
    adc_motor_vals[5] = slow_adc(MOT2_BEMF_L_ADX, MOT2_BEMF_L_CHAN);
    adc_motor_vals[6] = slow_adc(MOT3_BEMF_H_ADX, MOT3_BEMF_H_CHAN);
    adc_motor_vals[7] = slow_adc(MOT3_BEMF_L_ADX, MOT3_BEMF_L_CHAN);

    bemf_vals[0] = (adc_motor_vals[0] - adc_motor_vals[1]);
    bemf_vals[1] = (adc_motor_vals[2] - adc_motor_vals[3]);
    bemf_vals[2] = (adc_motor_vals[4] - adc_motor_vals[5]);
    bemf_vals[3] = (adc_motor_vals[6] - adc_motor_vals[7]);

    int j;
    for (j = 0; j < 4; ++j)
    {
        bemf_vals_filt[j] = 0.25f*(float)bemf_vals[j] + 0.75f*(float)bemf_vals_filt[j];
        if (bemf_vals_filt[j] < 10 && bemf_vals_filt[j] > -10) bemf_vals_filt[j] = 0;
    }
    //static int i = 0;
    //i += 1;
    //if (i % 10 == 0) debug_printf("%d   %d   %d   %d\n", bemf_vals[0], bemf_vals[1], bemf_vals[2], bemf_vals[3]);
}
