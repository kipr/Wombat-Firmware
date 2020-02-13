#include "wallaby_dig.h"

#include "wallaby.h"

void configDigitalInPin(uint32_t pin, GPIO_TypeDef* port)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;		
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(port, &GPIO_InitStructure);
}

void configDigitalOutPin(uint32_t pin, GPIO_TypeDef* port)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(port, &GPIO_InitStructure);
}

void update_dig_pin_from_reg(uint32_t pin, GPIO_TypeDef* port, uint8_t output_enable, uint8_t pullup_enable)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;		
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    if (output_enable)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    }
    else
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    }

#ifdef WALLABY2

    // Wallaby22
    if (pullup_enable)
    {
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    }
    else
    {
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    }
#else

    // Wallaby1
    if (pullup_enable)
    {
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    }
    else
    {
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    }
#endif


    GPIO_Init(port, &GPIO_InitStructure);
}


void update_dig_pin_configs()
{
    update_dig_pin_from_reg(DIG0_PIN, DIG0_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 1, aTxBuffer[REG_RW_DIG_PE_L] & 1);
    update_dig_pin_from_reg(DIG1_PIN, DIG1_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 2, aTxBuffer[REG_RW_DIG_PE_L] & 2);
    update_dig_pin_from_reg(DIG2_PIN, DIG2_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 4, aTxBuffer[REG_RW_DIG_PE_L] & 4);
    update_dig_pin_from_reg(DIG3_PIN, DIG3_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 8, aTxBuffer[REG_RW_DIG_PE_L] & 8);
    update_dig_pin_from_reg(DIG4_PIN, DIG4_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 16, aTxBuffer[REG_RW_DIG_PE_L] & 16);
    update_dig_pin_from_reg(DIG5_PIN, DIG5_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 32, aTxBuffer[REG_RW_DIG_PE_L] & 32);
    update_dig_pin_from_reg(DIG6_PIN, DIG6_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 64, aTxBuffer[REG_RW_DIG_PE_L] & 64);
    update_dig_pin_from_reg(DIG7_PIN, DIG7_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 128, aTxBuffer[REG_RW_DIG_PE_L] & 128);

    update_dig_pin_from_reg(DIG8_PIN, DIG8_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 1, aTxBuffer[REG_RW_DIG_PE_H] & 1);
    update_dig_pin_from_reg(DIG9_PIN, DIG9_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 2, aTxBuffer[REG_RW_DIG_PE_H] & 2);
    update_dig_pin_from_reg(DIG10_PIN, DIG10_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 4, aTxBuffer[REG_RW_DIG_PE_H] & 4);
    update_dig_pin_from_reg(DIG11_PIN, DIG11_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 8, aTxBuffer[REG_RW_DIG_PE_H] & 8);
    update_dig_pin_from_reg(DIG12_PIN, DIG12_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 16, aTxBuffer[REG_RW_DIG_PE_H] & 16);
    update_dig_pin_from_reg(DIG13_PIN, DIG13_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 32, aTxBuffer[REG_RW_DIG_PE_H] & 32);
    update_dig_pin_from_reg(DIG14_PIN, DIG14_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 64, aTxBuffer[REG_RW_DIG_PE_H] & 64);
    update_dig_pin_from_reg(DIG15_PIN, DIG15_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 128, aTxBuffer[REG_RW_DIG_PE_H] & 128);
}

void update_dig_pin(uint32_t pin, GPIO_TypeDef* port, uint8_t output_enable, uint8_t out_val, uint8_t pin_num)
{
    if (output_enable)
    {
        if (out_val)
        {
            // set pin high
            port->BSRRL |= pin;
        }
        else
        {
            // set pin low
            port->BSRRH |= pin;
        }
    }
    else
    {
        uint8_t dig_in_addr;
        uint8_t shift;
        if (pin_num > 7)
        {   
            dig_in_addr = REG_RW_DIG_IN_H;
            shift = pin_num-8;
        }
        else
        {
            dig_in_addr = REG_RW_DIG_IN_L;
            shift = pin_num;
        }

        // IDR is low when pin is high
        if ((port->IDR & pin) == 0)
        {
            // pin is high
            aTxBuffer[dig_in_addr] |= (1<<shift);
        }
        else
        {
            // pin is low
            aTxBuffer[dig_in_addr] &= ~(1<<shift);
        }
    }
}

void update_dig_pins()
{

    update_dig_pin(DIG0_PIN, DIG0_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 1, aTxBuffer[REG_RW_DIG_OUT_L] & 1, 0);
    update_dig_pin(DIG1_PIN, DIG1_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 2, aTxBuffer[REG_RW_DIG_OUT_L] & 2, 1);
    update_dig_pin(DIG2_PIN, DIG2_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 4, aTxBuffer[REG_RW_DIG_OUT_L] & 4, 2);
    update_dig_pin(DIG3_PIN, DIG3_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 8, aTxBuffer[REG_RW_DIG_OUT_L] & 8, 3);
    update_dig_pin(DIG4_PIN, DIG4_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 16, aTxBuffer[REG_RW_DIG_OUT_L] & 16, 4);
    update_dig_pin(DIG5_PIN, DIG5_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 32, aTxBuffer[REG_RW_DIG_OUT_L] & 32, 5);
    update_dig_pin(DIG6_PIN, DIG6_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 64, aTxBuffer[REG_RW_DIG_OUT_L] & 64, 6);
    update_dig_pin(DIG7_PIN, DIG7_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 128, aTxBuffer[REG_RW_DIG_OUT_L] & 128, 7);

    update_dig_pin(DIG8_PIN, DIG8_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 1, aTxBuffer[REG_RW_DIG_OUT_H] & 1, 8);
    update_dig_pin(DIG9_PIN, DIG9_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 2, aTxBuffer[REG_RW_DIG_OUT_H] & 2, 9);
    update_dig_pin(DIG10_PIN, DIG10_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 4, aTxBuffer[REG_RW_DIG_OUT_H] & 4, 10);
    update_dig_pin(DIG11_PIN, DIG11_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 8, aTxBuffer[REG_RW_DIG_OUT_H] & 8, 11);
    update_dig_pin(DIG12_PIN, DIG12_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 16, aTxBuffer[REG_RW_DIG_OUT_H] & 16, 12);
    update_dig_pin(DIG13_PIN, DIG13_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 32, aTxBuffer[REG_RW_DIG_OUT_H] & 32, 13);
    update_dig_pin(DIG14_PIN, DIG14_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 64, aTxBuffer[REG_RW_DIG_OUT_H] & 64, 14);
    update_dig_pin(DIG15_PIN, DIG15_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 128, aTxBuffer[REG_RW_DIG_OUT_H] & 128, 15);
}

