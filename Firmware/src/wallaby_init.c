#include "wallaby_init.h"
#include "wallaby.h"

#include "stm32f4xx.h"

void init180MHz()
{
    RCC_DeInit();

    // Enable HSE
    RCC_HSEConfig(RCC_HSE_ON);
    //RCC_HSEConfig(RCC_HSE_Bypass);
    while(RCC_WaitForHSEStartUp() != SUCCESS);

    FLASH_PrefetchBufferCmd(ENABLE);
    FLASH_SetLatency(FLASH_Latency_5);

    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
    RCC_PCLK1Config(RCC_HCLK_Div4);
    RCC_PCLK2Config(RCC_HCLK_Div2); 

    uint32_t PLL_M = 24;  // ( (24 MHz / 24) * 360 ) / 2 = 180 MHz
    uint32_t PLL_N = 360;
    uint32_t PLL_P = 2;
    uint32_t PLL_Q = 7;

    RCC_PLLConfig(RCC_PLLSource_HSE,PLL_M,PLL_N,PLL_P,PLL_Q);
    RCC_PLLCmd(ENABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    // Wait till PLL is used as system clock source
    while(RCC_GetSYSCLKSource() != 0x08);
}




void init()
{
    init180MHz(); // switch over to 180 MHz via the external 24 MHz crystal and a PLL

    initSystick(); // get a wallclock started, ticking at 1ms

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_DMA1 , ENABLE);

    // TIM1,8 for Motors
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    // TIM3,9 for Servos
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

    SPI_DMA_Config(BUFFERSIZE);

    // Slave board configuration
    // Initializes the SPI communication
    
    //SPI_InitTypeDef  SPI_InitStructure;
    //SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    //SPI_Init(SPI2, &SPI_InitStructure);

    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

    // Enable DMA SPI TX Stream 
    DMA_Cmd(DMA1_Stream4,ENABLE);

    // Enable DMA SPI RX Stream 
    DMA_Cmd(DMA1_Stream3,ENABLE); 

    {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM1 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
        {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM1 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    // Enable SPI DMA TX Requsts 
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

    // Enable SPI DMA RX Requsts 
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);

    //debug_printf("enable spi\n");
    // Enable the SPI peripheral
    SPI_Cmd(SPI2, ENABLE);

    // motor 0 
    configMotorPin(MOT0_DIR1, MOT0_DIR1_PORT);
    configMotorPin(MOT0_DIR2, MOT0_DIR2_PORT);
    //configMotorPin(MOT0_PWM, MOT0_PWM_PORT);
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = MOT0_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(MOT0_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(MOT0_PWM_PORT, MOT0_PWM_PinSource, GPIO_AF_TIM1);
    }

    // motor 1
    configMotorPin(MOT1_DIR1, MOT1_DIR1_PORT);
    configMotorPin(MOT1_DIR2, MOT1_DIR2_PORT);
    //configMotorPin(MOT1_PWM, MOT1_PWM_PORT);
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = MOT1_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(MOT1_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(MOT1_PWM_PORT, MOT1_PWM_PinSource, GPIO_AF_TIM1);
    }

    // motor 2
    configMotorPin(MOT2_DIR1, MOT2_DIR1_PORT);
    configMotorPin(MOT2_DIR2, MOT2_DIR2_PORT);
    //configMotorPin(MOT2_PWM, MOT2_PWM_PORT);
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = MOT2_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(MOT2_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(MOT2_PWM_PORT, MOT2_PWM_PinSource, GPIO_AF_TIM1);
    }

    // motor 3
    configMotorPin(MOT3_DIR1, MOT3_DIR1_PORT);
    configMotorPin(MOT3_DIR2, MOT3_DIR2_PORT);
    //configMotorPin(MOT3_PWM, MOT3_PWM_PORT);
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = MOT3_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(MOT3_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(MOT3_PWM_PORT, MOT3_PWM_PinSource, GPIO_AF_TIM8);
    }

    // motor bemf adc sensing
    configBEMFPin(MOT0_BEMF_H, MOT0_BEMF_H_PORT);
    configBEMFPin(MOT0_BEMF_L, MOT0_BEMF_L_PORT);
    configBEMFPin(MOT1_BEMF_H, MOT1_BEMF_H_PORT);
    configBEMFPin(MOT1_BEMF_L, MOT1_BEMF_L_PORT);

    configBEMFPin(MOT2_BEMF_H, MOT2_BEMF_H_PORT);
    configBEMFPin(MOT2_BEMF_L, MOT2_BEMF_L_PORT);
    configBEMFPin(MOT3_BEMF_H, MOT3_BEMF_H_PORT);
    configBEMFPin(MOT3_BEMF_L, MOT3_BEMF_L_PORT);

    // TODO: cleaner servo init and all 4
    // TODO for now just 0,1 on TIM3
    // TODO: later, 2,3 on TIM9
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = SRV0_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(SRV0_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(SRV0_PWM_PORT, SRV0_PWM_PinSource, GPIO_AF_TIM3);
    }

    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = SRV1_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(SRV1_PWM_PORT, &GPIO_InitStructure);
        
        GPIO_PinAFConfig(SRV1_PWM_PORT, SRV1_PWM_PinSource, GPIO_AF_TIM3);
    }

    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = SRV2_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(SRV2_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(SRV2_PWM_PORT, SRV2_PWM_PinSource, GPIO_AF_TIM9);
    }

    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = SRV3_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(SRV3_PWM_PORT, &GPIO_InitStructure);
        
        GPIO_PinAFConfig(SRV3_PWM_PORT, SRV3_PWM_PinSource, GPIO_AF_TIM9);
    }
    
    TIM1_Configuration(); // motors 0,1,2

    // TODO move tim1 interrupt setup for motor control
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM1 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    TIM8_Configuration(); // motor 3
    
    // TODO move tim8 interrupt setup for motor control
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM8 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }    

    TIM3_Configuration(); // servos 0,1
        // TODO move tim3 interrupt setup for servo control
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM3 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    TIM9_Configuration(); // servos 2,3

        // TODO move tim9 interrupt setup for servo control
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM9 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }



    // Button S1 is also a digital in
    // TODO: make this another digital pin
    configDigitalInPin(BUTTON_S1_PIN, BUTTON_S1_PORT);

    // Init SPI3 (plus chip selects) for the IMU sensors
    initSPI3(); 

    // Init SPI4 (SPI header)
    //initSPI4(); 
 
    //setup_I2C1();
   
    // wait a bit
    delay_us(5000);

    //setupAccelMag();

    //setupGyro();
    debug_printf("calling setupIMU()\r\n");
    wallaby_imu_init();
/*
    setupUART2();
    setupUART3();
*/
    // wait a bit
    delay_us(5);
}
