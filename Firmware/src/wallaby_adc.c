#include "wallaby_adc.h"

#include "wallaby.h"


void configAnalogInPin(uint32_t pin, GPIO_TypeDef* port, uint8_t pullup_enable) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    
    //if (pullup_enable)
    //{
    //    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    //}
    //else
    //{
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //}

    GPIO_Init(port, &GPIO_InitStructure);
}


void config_adc_in_from_regs()
{
    // analog in pins
    configAnalogInPin(AIN0_PIN, AIN0_PORT, aTxBuffer[REG_RW_ADC_PE] & 1);
    configAnalogInPin(AIN1_PIN, AIN1_PORT, aTxBuffer[REG_RW_ADC_PE] & 2);
    configAnalogInPin(AIN2_PIN, AIN2_PORT, aTxBuffer[REG_RW_ADC_PE] & 4);
    configAnalogInPin(AIN3_PIN, AIN3_PORT, aTxBuffer[REG_RW_ADC_PE] & 8);
    configAnalogInPin(AIN4_PIN, AIN4_PORT, aTxBuffer[REG_RW_ADC_PE] & 16);
    configAnalogInPin(AIN5_PIN, AIN5_PORT, aTxBuffer[REG_RW_ADC_PE] & 32);

    // adc also used for battery level sensing
    configAnalogInPin(ADC_BATT_PIN, ADC_BATT_PORT, 0);
}



uint16_t slow_adc(ADC_TypeDef * bus, uint8_t channel)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // ADC Common Init
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1 Channel
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // Manual
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(bus, &ADC_InitStructure);

    // ADC1 regular channel 11 configuration
    ADC_RegularChannelConfig(bus, channel, 1, ADC_SampleTime_28Cycles); // PC1

    // Enable ADC1
    ADC_Cmd(bus, ENABLE);
    delay_us(10);

    ADC_SoftwareStartConv(bus);
    while(ADC_GetFlagStatus(bus, ADC_FLAG_EOC) == RESET);
    uint16_t val = ADC_GetConversionValue(bus);

    // Disable ADC1
    ADC_Cmd(bus, DISABLE);
    delay_us(10);
    return val;
}



int16_t adc_update()
{
    //debug_printf("adc_demo\n");

    static int16_t adc_in[6];
    static int32_t adc_batt = 800; // TODO: int16_t when I can find out how to force a 4-byte aligned output binary

    adc_in[0] = (0.9f)*(float)adc_in[0] + (0.1f)*(float)(slow_adc(AIN0_ADX, AIN0_CHAN));
    adc_in[1] = (0.9f)*(float)adc_in[1] + (0.1f)*(float)(slow_adc(AIN1_ADX, AIN1_CHAN));
    adc_in[2] = (0.9f)*(float)adc_in[2] + (0.1f)*(float)(slow_adc(AIN2_ADX, AIN2_CHAN));
    adc_in[3] = (0.9f)*(float)adc_in[3] + (0.1f)*(float)(slow_adc(AIN3_ADX, AIN3_CHAN));
    adc_in[4] = (0.9f)*(float)adc_in[4] + (0.1f)*(float)(slow_adc(AIN4_ADX, AIN4_CHAN));
    adc_in[5] = (0.9f)*(float)adc_in[5] + (0.1f)*(float)(slow_adc(AIN5_ADX, AIN5_CHAN));

    //static int i = 0;
    //i+=1;
    //if (i % 100 == 0) debug_printf("%d   %d   %d\n", adc_in[0], adc_in[2], adc_in[5]);

    adc_batt = (0.95f)*(float)adc_batt + (0.05f)*(float)slow_adc(ADC_BATT_ADX, ADC_BATT_CHAN);

    aTxBuffer[REG_RW_ADC_0_H] = (adc_in[0] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_0_L] = (adc_in[0] & 0x00FF);
    aTxBuffer[REG_RW_ADC_1_H] = (adc_in[1] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_1_L] = (adc_in[1] & 0x00FF);
    aTxBuffer[REG_RW_ADC_2_H] = (adc_in[2] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_2_L] = (adc_in[2] & 0x00FF);
    aTxBuffer[REG_RW_ADC_3_H] = (adc_in[3] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_3_L] = (adc_in[3] & 0x00FF);
    aTxBuffer[REG_RW_ADC_4_H] = (adc_in[4] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_4_L] = (adc_in[4] & 0x00FF);
    aTxBuffer[REG_RW_ADC_5_H] = (adc_in[5] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_5_L] = (adc_in[5] & 0x00FF);

    aTxBuffer[REG_RW_BATT_H] = (adc_batt & 0xFF00) >> 8;
    aTxBuffer[REG_RW_BATT_L] = (adc_batt & 0x00FF);

    return adc_batt;
}



