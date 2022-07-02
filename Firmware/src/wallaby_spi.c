#include "wallaby_spi.h"
#include "wallaby.h"

#include "stm32f4xx.h"

void initSPI3()
{
    // SPI clock should be enabled below with RCC_AHB1PeriphClockCmd


    // set up pins for  clock, miso, mosi
    {
        // Note: this assumes CLK, MISO, MOSI are on the same port... but they always should be
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI3_CLK | SPI3_MISO | SPI3_MOSI;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(SPI3_CLK_PORT, &GPIO_InitStruct);
    }
    
    // use the alternate function (SPI3)
    GPIO_PinAFConfig(SPI3_CLK_PORT,  SPI3_CLK_SOURCE,  GPIO_AF_SPI3);
    GPIO_PinAFConfig(SPI3_MISO_PORT, SPI3_MISO_SOURCE, GPIO_AF_SPI3);
    GPIO_PinAFConfig(SPI3_MOSI_PORT, SPI3_MOSI_SOURCE, GPIO_AF_SPI3);


    // set up chip select pins, not expected to be on the same port
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI3_CS0;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(SPI3_CS0_PORT, &GPIO_InitStruct);     
    }
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI3_CS1;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(SPI3_CS1_PORT, &GPIO_InitStruct);   
    }


    // set chip selects high  (they are active low)
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0;
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

    // configure SPI3 in Mode 0
    // CPOL 0  (clock low when idle)
    // CPHA 0  (data sampled at first edge)
    {
        SPI_InitTypeDef SPI_InitStruct;
        SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
        SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_Init(SPI3, &SPI_InitStruct);
    }

    SPI_Cmd(SPI3, ENABLE);
}



void initSPI4()
{
    // SPI clock should be enabled below with RCC_AHB1PeriphClockCmd


    // set up pins for  clock, miso, mosi
    {
        // Note: this assumes CLK, MISO, MOSI are on the same port... but they always should be
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI4_CLK | SPI4_MISO | SPI4_MOSI;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(SPI4_CLK_PORT, &GPIO_InitStruct);
    }
    
    // use the alternate function (SPI3)
    GPIO_PinAFConfig(SPI4_CLK_PORT,  SPI4_CLK_SOURCE,  GPIO_AF_SPI4);
    GPIO_PinAFConfig(SPI4_MISO_PORT, SPI4_MISO_SOURCE, GPIO_AF_SPI4);
    GPIO_PinAFConfig(SPI4_MOSI_PORT, SPI4_MOSI_SOURCE, GPIO_AF_SPI4);


    // set up chip select and nirq pins, not expected to be on the same port
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI4_CS0;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(SPI4_CS0_PORT, &GPIO_InitStruct);     
    }
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI4_NIRQ;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(SPI4_NIRQ_PORT, &GPIO_InitStruct);   
    }


    // set chip select and nirq high  (they are active low)
    SPI4_CS0_PORT->BSRRL |= SPI4_CS0;
    SPI4_NIRQ_PORT->BSRRL |= SPI4_NIRQ;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI4, ENABLE);

    // configure SPI3 in Mode 0
    // CPOL 0  (clock low when idle)
    // CPHA 0  (data sampled at first edge)
    {
        SPI_InitTypeDef SPI_InitStruct;
        SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
        SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_Init(SPI4, &SPI_InitStruct);
    }

    SPI_Cmd(SPI4, ENABLE);
}


uint8_t SPI_write(SPI_TypeDef *const spi, uint8_t data)
{
    spi->DR = data; // write data
	while( !(spi->SR & SPI_I2S_FLAG_TXE) ); // wait for transmit
	while( !(spi->SR & SPI_I2S_FLAG_RXNE) ); // wait for receive
	while( spi->SR & SPI_I2S_FLAG_BSY ); // wait unit SPI is done
	return spi->DR; // return data
}

uint8_t SPI3_write(uint8_t data)
{
	return SPI_write(SPI3, data);
}


uint8_t SPI4_write(uint8_t data)
{
	return SPI_write(SPI4, data);
}


void spi4_demo()
{

    SPI4_CS0_PORT->BSRRH |= SPI4_CS0; // chip select low
    delay_us(100); 


    uint8_t regval;
    // request "Who am I)
    SPI4_write(0x1F); // write register
    regval = SPI4_write(0x00); // write dummy val to get contents
    debug_printf("Read a %x\n", regval);

    SPI4_CS0_PORT->BSRRL |= SPI4_CS0; // chip select high
}

