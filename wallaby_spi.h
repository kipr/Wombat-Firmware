#ifndef WALLABY_SPI_H_
#define WALLABY_SPI_H

#include "stm32f4xx.h"

void initSPI3();
void initSPI4();


uint8_t SPI3_write(uint8_t data);
uint8_t SPI4_write(uint8_t data);

void spi4_demo();

#endif