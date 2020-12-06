// use a board definition file
// makes it easier to switch things like processor model, board revision, etc 
#include "wallaby_r2.h"
#include "wallaby_spi_r4.h"

#include "wallaby_adc.h"
#include "wallaby_bemf.h"
#include "wallaby_dig.h"
#include "wallaby_dma.h"
#include "wallaby_i2c.h"
#include "wallaby_imu.h"
#include "wallaby_init.h"
#include "wallaby_motor.h"
#include "wallaby_pid.h"
#include "wallaby_servo.h"
#include "wallaby_spi.h"
#include "wallaby_uart.h"


// Wallaby2
#ifndef WALLABY2
#define WALLABY2
#endif


#ifdef USE_CROSS_STUDIO_DEBUG
    #include <__cross_studio_io.h>
#else
    int debug_printf(const char *format, ...);
    void debug_exit(uint8_t val);
#endif



#ifndef WALLABY_H_
#define WALLABY_H_

#define WALLABY_FIRMWARE_VERSION_R  8

#define BUFFERSIZE                       REG_READABLE_COUNT

extern volatile uint8_t aTxBuffer[REG_ALL_COUNT]; // =  {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};  //"SPI Master/Slave : Communication between two SPI using DMA";
extern __IO uint8_t aRxBuffer [REG_ALL_COUNT];

extern volatile uint8_t adc_dirty;
extern volatile uint8_t dig_dirty;

extern volatile uint32_t usCount;


void initSystick();


void SysTick_Handler(void);
 
 
void delay_us(uint32_t delay);

#endif
