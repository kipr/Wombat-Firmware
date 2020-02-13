#include "wallaby_uart.h"

#include "wallaby.h"

// serial communication over external uart header
void uart2_demo()
{
    debug_printf("uart2 demo\n");
    // Wait until a byte is received 
    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET)
    {
    }

    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
    {
        unsigned char rd = USART_ReceiveData(USART2);
        debug_printf("got %c\n", rd);
    }

    // read byte and print via usart2
    //unsigned char rd = USART_ReceiveData(USART2);
    //debug_printf("got %c\n", rd);
    //debug_printf("sending\n");
    USART_SendData(USART2, 'D');        
    debug_printf("done with uart2 demo\n");
}


// serial communication with main processor
void uart3_demo()
{
    //debug_printf("uart3 demo\n");
    // Wait until a byte is received 
    while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
    {
    }

    // read byte and print via usart2
    unsigned char rd = USART_ReceiveData(USART3);
    //debug_printf("got %c\n", rd);
    USART_SendData(USART3, 'D');        
}



void setupUART2()
{
    // UART2 to external header
    
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    // set up serial on UART2 to main processor
    // PD5 TX   PD6 RX
    USART_InitTypeDef usartConfig;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // RCC_APB1Periph_AFIO
    usartConfig.USART_BaudRate = 115200;
    usartConfig.USART_WordLength = USART_WordLength_8b;
    usartConfig.USART_StopBits = USART_StopBits_1;
    usartConfig.USART_Parity = GPIO_PuPd_NOPULL;
    usartConfig.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usartConfig.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &usartConfig);

    USART_Cmd(USART2, ENABLE);
    //USART_ITConfig(USART2, USART_IT_RXNE |USART_IT_TXE, ENABLE);
}

void setupUART3()
{
    // UART3 to main proc
    
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    // set up serial on UART3 to main processor
    // PB10 TX   PB11 RX
    USART_InitTypeDef usartConfig;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // RCC_APB1Periph_AFIO
    usartConfig.USART_BaudRate = 115200;
    usartConfig.USART_WordLength = USART_WordLength_8b;
    usartConfig.USART_StopBits = USART_StopBits_1;
    usartConfig.USART_Parity = USART_Parity_No;
    usartConfig.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usartConfig.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &usartConfig);

    USART_Cmd(USART3, ENABLE);
    //USART_ITConfig(USART3, USART_IT_RXNE |USART_IT_TXE, ENABLE);
}
