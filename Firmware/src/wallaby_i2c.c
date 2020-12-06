#include "wallaby_i2c.h"

#include "wallaby.h"

void setup_I2C1(void)
{
    //debug_printf("Setup I2C1\n");
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // TODO: already did this
    
    GPIO_InitStruct.GPIO_Pin = I2C1_SDA | I2C1_SCL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;// GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_PinAFConfig(I2C1_SDA_PORT, I2C1_SDA_SOURCE, GPIO_AF_I2C1);	
    GPIO_PinAFConfig(I2C1_SCL_PORT, I2C1_SCL_SOURCE, GPIO_AF_I2C1);
    GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct); // TODO: assumes both pins on same port

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    I2C_DeInit(I2C1);
    I2C_Cmd(I2C1, DISABLE);


    I2C_InitStruct.I2C_ClockSpeed = 100000;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0xA1;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &I2C_InitStruct);
}



//https://github.com/devthrash/STM32F4-examples/blob/master/I2C%20Master/main.c#L40
/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy anymore
        //debug_printf("a\n");
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	//debug_printf("b\n");

	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	//debug_printf("c\n");

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	//debug_printf("d\n");

	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	  
	/* wait for I2C1 EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
        //debug_printf("e\n");

	if(direction == I2C_Direction_Transmitter){
                //debug_printf("transmitter\n");
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
                //debug_printf("receiver\n");
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}

        I2C_Cmd(I2Cx, ENABLE);

	//debug_printf("f\n");
}


void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

void I2C_stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition 
	I2C_GenerateSTOP(I2Cx, ENABLE);
}


void i2c1_test(void)
{
    debug_printf("i2c1_test\n");
    const uint8_t SLAVE_ADDRESS = 0xA0; // FIXME:0xA0;
    uint8_t received_data[2];

   while(1)
   {
		
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2C1, 0x20); // write one byte to the slave
		//I2C_write(I2C1, 0x03); // write another byte to the slave
		I2C_stop(I2C1); // stop the transmission
		
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
		received_data[0] = I2C_read_ack(I2C1); // read one byte and request another byte
		received_data[1] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
                debug_printf("Read %x, %x\n", received_data[0], received_data[1]);
  }
}


