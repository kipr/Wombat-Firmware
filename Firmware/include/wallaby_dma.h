#include "stm32f4xx.h"

void SPI_DMA_Config(unsigned int buffer_len);

void spi2_dma_cleanup();


void clear_rx_buffer();
void clear_tx_buffer();

void init_rx_buffer();
void init_tx_buffer();

void handle_dma();

void DMA1_Stream3_IRQHandler();
void DMA1_Stream4_IRQHandler();

