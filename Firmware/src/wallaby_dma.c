#include "wallaby_dma.h"

#include "wallaby.h"

#include "stm32f4xx.h"

void SPI_DMA_Config(unsigned int buffer_len)
{
    GPIO_InitTypeDef GPIO_InitStructure;


    // SPI GPIO Configuration --------------------------------------------------
    // GPIO Deinitialisation
    GPIO_DeInit(SPI2_CLK_PORT);
    GPIO_DeInit(SPI2_MISO_PORT);
    GPIO_DeInit(SPI2_MOSI_PORT);
    GPIO_DeInit(SPI2_CS0_PORT);
  
    // Connect SPI pins to AF
    GPIO_PinAFConfig(SPI2_CLK_PORT, SPI2_CLK_SOURCE, GPIO_AF_SPI2);
    GPIO_PinAFConfig(SPI2_MISO_PORT, SPI2_MISO_SOURCE, GPIO_AF_SPI2);   
    GPIO_PinAFConfig(SPI2_MOSI_PORT, SPI2_MOSI_SOURCE, GPIO_AF_SPI2);
    GPIO_PinAFConfig(SPI2_CS0_PORT, SPI2_CS0_SOURCE, GPIO_AF_SPI2);

    // SPI  MISO pin configuration
    GPIO_InitStructure.GPIO_Pin =  SPI2_MISO;
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

    GPIO_Init(SPI2_MISO_PORT, &GPIO_InitStructure);




    // SPI  MOSI pin configuration
    GPIO_InitStructure.GPIO_Pin =  SPI2_MOSI;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(SPI2_MOSI_PORT, &GPIO_InitStructure);


    // SPI SCK pin configuration
    //    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Pin = SPI2_CLK;
    GPIO_Init(SPI2_CLK_PORT, &GPIO_InitStructure);



    GPIO_InitStructure.GPIO_Pin =  SPI2_CS0;
     //   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(SPI2_CS0_PORT, &GPIO_InitStructure);

    spi2_dma_init(buffer_len);
}


// Doesn't touch GPIO, so spi2_dma_resync() can call it at runtime without
// resetting the other pins on port B.
void spi2_dma_init(unsigned int buffer_len)
{
    DMA_InitTypeDef DMA_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;


    // SPI configuration -------------------------------------------------------
    SPI_I2S_DeInit(SPI2);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;


    // Peripheral Clock Enable -------------------------------------------------
    // Enable the SPI clock 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    // Enable GPIO clocks 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Enable DMA clock 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);



    // DMA configuration -------------------------------------------------------
    // Deinitialize DMA Streams
    DMA_DeInit(DMA1_Stream4); // TX stream
    DMA_DeInit(DMA1_Stream3); // RX stream



    // Configure DMA Initialization Structure 
    DMA_InitStructure.DMA_BufferSize = buffer_len;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI2->DR));
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // Configure TX DMA
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)aTxBuffer;
    DMA_Init(DMA1_Stream4, &DMA_InitStructure);
    // Configure RX DMA
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)aRxBuffer;
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);

    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    SPI_Init(SPI2, &SPI_InitStructure);
}


void spi2_dma_start()
{
    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

    // Enable DMA SPI TX Stream
    DMA_Cmd(DMA1_Stream4,ENABLE);

    // Enable DMA SPI RX Stream
    DMA_Cmd(DMA1_Stream3,ENABLE);

    // Enable SPI DMA TX Requsts
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

    // Enable SPI DMA RX Requsts
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);

    // Enable the SPI peripheral
    SPI_Cmd(SPI2, ENABLE);
}


// The circular DMA never realigns with the host's frames by itself: after one
// transfer of the wrong length, or a glitch on SCK or chip select, every later
// frame is offset until SPI2 and both streams are reset.
void spi2_dma_resync()
{
    // Only reset between transfers (chip select is active low). If the host
    // starts one mid-reset, that frame fails framing and sets spi_resync again.
    if ((SPI2_CS0_PORT->IDR & SPI2_CS0) == 0) return;

    // Disabling a stream only pauses it at its current count, and its config
    // registers can't be rewritten until it has actually stopped. Interrupts
    // stay off so handle_dma() can't run against a half-reset link.
    __disable_irq();
    DMA_Cmd(DMA1_Stream4, DISABLE);
    DMA_Cmd(DMA1_Stream3, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE || DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);

    // Also resets SPI2 through RCC, which drops any partial byte.
    spi2_dma_init(BUFFERSIZE);
    spi2_dma_start();
    spi_resync = 0;
    __enable_irq();
}


void spi2_dma_cleanup()
{
    //dma cleanup
    // Clear DMA Transfer Complete Flags66
    DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4); // tx
    DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3); // rx

    // Disable DMA SPI TX Stream
    DMA_Cmd(DMA1_Stream4,DISABLE);

    // Disable DMA SPI RX Stream
    DMA_Cmd(DMA1_Stream3,DISABLE);

    // Disable SPI DMA TX Requsts
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);

    // Disable SPI DMA RX Requsts
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, DISABLE);

    // Disable the SPI peripheral
    SPI_Cmd(SPI2, DISABLE);
}



void clear_rx_buffer()
{
    for(uint16_t i=0;i<BUFFERSIZE;i++)
    {
        aRxBuffer[i]=0x00;
    }
}


void clear_tx_buffer()
{
    for(uint16_t i=0;i<BUFFERSIZE;i++)
    {
        aTxBuffer[i]=0x00;
    }
}


void init_rx_buffer()
{
    clear_rx_buffer();
}



void init_tx_buffer()
{
    clear_tx_buffer();
    
    aTxBuffer[REG_R_START] = 'J';
    aTxBuffer[REG_R_VERSION_H] = (((uint16_t)WALLABY_FIRMWARE_VERSION_R) & 0xFF00) >> 8;
    aTxBuffer[REG_R_VERSION_L] = (((uint16_t)WALLABY_FIRMWARE_VERSION_R) & 0x00FF);

    const uint16_t init_servo_cmd = 1500;
    aTxBuffer[REG_RW_SERVO_0_H] = (init_servo_cmd & 0xFF00) >> 8; 
    aTxBuffer[REG_RW_SERVO_0_L] = (init_servo_cmd & 0x00FF); 
    aTxBuffer[REG_RW_SERVO_1_H] = (init_servo_cmd & 0xFF00) >> 8; 
    aTxBuffer[REG_RW_SERVO_1_L] = (init_servo_cmd & 0x00FF); 
    aTxBuffer[REG_RW_SERVO_2_H] = (init_servo_cmd & 0xFF00) >> 8; 
    aTxBuffer[REG_RW_SERVO_2_L] = (init_servo_cmd & 0x00FF); 
    aTxBuffer[REG_RW_SERVO_3_H] = (init_servo_cmd & 0xFF00) >> 8; 
    aTxBuffer[REG_RW_SERVO_3_L] = (init_servo_cmd & 0x00FF); 
    aTxBuffer[REG_RW_MOT_SRV_ALLSTOP] = 0b11110000;

    uint8_t i;
    for (i = 0; i < 4; ++i) set_pid_reg_defaults(i);
}


void handle_dma()
{
    static uint8_t expected = 0;

    if ( (DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET) &&  (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET))
    {
        // TODO: make this interrupt-based
        if (aRxBuffer[0] == 'J'  && aRxBuffer[REG_READABLE_COUNT-1] == 'S' && aRxBuffer[1] == WALLABY_SPI_VERSION)
        {
            expected += 1;
            //if (aRxBuffer[2] != expected) debug_printf("Missed packet(s) got ID %d expected %d\n", aRxBuffer[2], expected);

            // handle recently compled DMA transfer
            uint8_t num_regs = aRxBuffer[3];
            // (address, value) pairs run from byte 4 up to the trailing 'S', so a
            // larger count is corrupt. Clamp rather than drop the packet: libwallaby
            // zero-pads, and the address check below skips those (0, 0) pairs.
            const uint8_t max_regs = (REG_READABLE_COUNT - 1 - 4) / 2;
            if (num_regs > max_regs) num_regs = max_regs;
            uint8_t j;
            if (num_regs != 0)
            {
                //debug_printf("got good packet with %d\n", num_regs);
                //debug_printf("%x %x %x %x %x %x %x %x %x %x\n", aRxBuffer[0], aRxBuffer[1], aRxBuffer[2], aRxBuffer[3], aRxBuffer[4], aRxBuffer[5], aRxBuffer[6], aRxBuffer[7], aRxBuffer[8], aRxBuffer[9]);    
    
            }
            uint8_t stop = 2 * num_regs;
            for (j = 0; j < stop; j+=2)
            {
                uint8_t address = aRxBuffer[4+j];
                uint8_t value = aRxBuffer[4+j+1];

                // Registers below REG_RW_DIG_IN_H are the read-only start byte and
                // firmware version. libwallaby checks the start byte on every transfer.
                if (address < REG_RW_DIG_IN_H || address >= REG_ALL_COUNT) continue;

                // handle motors modes clearing done bits
                // TODO: cleanup
                if (address == REG_RW_MOT_MODES)
                {
                    if (value & 0b00000011) aTxBuffer[REG_RW_MOT_DONE] &= ~1;
                    if (value & 0b00001100) aTxBuffer[REG_RW_MOT_DONE] &= ~2;
                    if (value & 0b00110000) aTxBuffer[REG_RW_MOT_DONE] &= ~4;
                    if (value & 0b11000000) aTxBuffer[REG_RW_MOT_DONE] &= ~8;
                }

                aTxBuffer[address] = value;

                if (address == REG_RW_ADC_PE) adc_dirty = 1;

                if (address >= REG_RW_DIG_PE_H && address <= REG_RW_DIG_OE_L) dig_dirty = 1;

                // TODO: handle PID coefficients
            }
        }
        else
        {
            debug_printf("got bad spi packet with start %d version %d tot %d\n", aRxBuffer[0], aRxBuffer[1], aRxBuffer[REG_READABLE_COUNT-1]);
            if (aRxBuffer[1] == WALLABY_SPI_VERSION)
            {
               debug_printf("SPI protocol version mismatch\n");
            }

            // A wrong start or end byte means the DMA has lost the frame
            // boundary. A wrong version alone is a protocol mismatch, which a
            // reset can't fix.
            if (aRxBuffer[0] != 'J' || aRxBuffer[REG_READABLE_COUNT-1] != 'S') spi_resync = 1;
        }
        // Clear DMA Transfer Complete Flags so we can notice when a transfer happens again
        DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4); // tx
        DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3); // rx
    }
}


void DMA1_Stream3_IRQHandler()
{
    handle_dma();
}


void DMA1_Stream4_IRQHandler()
{
    handle_dma();
}

