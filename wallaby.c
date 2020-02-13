#include "wallaby.h"


volatile uint8_t aTxBuffer[REG_ALL_COUNT]; // =  {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};  //"SPI Master/Slave : Communication between two SPI using DMA";
__IO uint8_t aRxBuffer [REG_ALL_COUNT];
 

// TODO: timings based on a 32 bit usec clock would overflow and maybe glitch every 1.19 hours
// Maybe have a second clock too?  If so usCount would just be  time % 1_second 
// and we would want helper functions that accept  sec/usec timing pairs
volatile uint32_t usCount;

volatile uint8_t adc_dirty;
volatile uint8_t dig_dirty;

#ifdef USE_CROSS_STUDIO_DEBUG
    int debug_printf(const char *format, ...){}
    void debug_exit(uint8_t val){}
#endif



void initSystick()
{
    usCount=0;
    SysTick_Config(SystemCoreClock/1000000);
}

void SysTick_Handler(void) 
{
    usCount++;
}


void delay_us(uint32_t delay)
{
    uint32_t target;
    target=usCount+delay;
    while(usCount<target);
}