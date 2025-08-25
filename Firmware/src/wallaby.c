/*
*  Modified by: Tobias Madlberger*, Marc Prantl, Matthias, 2025
*  * Corresponding author's email: tobias.madlberger@gmail.com
*/

#include "wallaby.h"

#include <string.h>


volatile uint8_t aTxBuffer[REG_ALL_COUNT];
// =  {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};  //"SPI Master/Slave : Communication between two SPI using DMA";
__IO uint8_t aRxBuffer[REG_ALL_COUNT];


// TODO: timings based on a 32 bit usec clock would overflow and maybe glitch every 1.19 hours
// Maybe have a second clock too?  If so usCount would just be  time % 1_second 
// and we would want helper functions that accept  sec/usec timing pairs
volatile uint32_t usCount;

volatile uint8_t adc_dirty;
volatile uint8_t dig_dirty;

#ifndef USE_CROSS_STUDIO_DEBUG
int debug_printf(const char* format, ...)
{
}

void debug_exit(uint8_t val)
{
}
#endif


void pack_float(const uint8_t reg, const float value)
{
    uint32_t intValue;
    memcpy(&intValue, &value, sizeof(float)); // Safely reinterpret the float

    // Pack in big-endian order:
    aTxBuffer[reg] = intValue >> 24 & 0xFF; // Most significant byte first
    aTxBuffer[reg + 1] = intValue >> 16 & 0xFF;
    aTxBuffer[reg + 2] = intValue >> 8 & 0xFF;
    aTxBuffer[reg + 3] = intValue & 0xFF; // Least significant byte last
}

void initSystick()
{
    usCount = 0;
    SysTick_Config(SystemCoreClock / 1000000);
}

void SysTick_Handler(void)
{
    usCount++;
}


void delay_us(uint32_t delay)
{
    uint32_t target;
    target = usCount + delay;
    while (usCount < target);
}
