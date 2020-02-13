#include "wallaby_imu.h"
#include "wallaby.h"

#define WALLABY2

#define MPU9250_WHO_AMI_I_REG  0x75
#define MPU9250_WHO_AMI_I_RES  0x71
#define MPU9250_CONFIG_REG 0x1A
#define MPU9250_GYRO_CONFIG_REG 0x1B
#define MPU9250_ACCEL_CONFIG_REG 0x1C
#define MPU9250_ACCEL_CONFIG2_REG 0x1D
#define MPU9250_ACCEL_START_REG 0x3B // xh xl yh yl zh zl
#define MPU9250_GYRO_START_REG 0x43 // xh xl yh yl zh zl
#define MPU9250_MAGN_START_REG 0x03 // xl xh yl yh zl zh
#define MPU9250_MAGN_CONTROL1_REG 0x0A // control register 1
#define MPU9250_MAGN_SENS_SCALING ((float)0.15f)
#define MPU9250_EXT_SENS_DATA_00_REG 0x49 // magnetometer can be available here

float MAGN_SCALE_FACTORS[3] = {0.0f, 0.0f, 0.0f};


uint8_t IMU_write(uint8_t address, uint8_t val)
{
    uint8_t ret;
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(address);
    ret = SPI3_write(val);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
    delay_us(2000);//TODO:remove
    return ret;
}

uint8_t IMU_read(uint8_t address)
{
    uint8_t ret;
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x80 | address);
    ret = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
    delay_us(2000); // TODO:remove
    return ret;
}

void setupIMU()
{
    debug_printf("inside setupIMU\r\n");
    // select Accel/Mag
   // SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    delay_us(200); 

    uint8_t regval;

    // request "Who am I"
    regval = IMU_read(MPU9250_WHO_AMI_I_REG);

    if (regval == MPU9250_WHO_AMI_I_RES){ //mpu9250
        debug_printf("IMU identified itself\r\n");
    }else{
        debug_printf("IMU did not respond/identify itself (regval=%d)\r\n",regval);
        return;
    }

    //{0x80, MPUREG_PWR_MGMT_1},     // Reset Device
    IMU_write(0x6B, 0x80);

    // wake up
    IMU_write(0x6B, 0x00); // power management 1, 50hz continuous, all axes
    delay_us(100);

    IMU_write(0x6B, 0x01); // power management 1, select time source
    delay_us(200);

    IMU_write(MPU9250_CONFIG_REG, 0x03); // config gyro/accel sample rate 200Hz

    IMU_write(0x19, 0x04); // samplerate div 200 Hz


    // gyro config
    regval = IMU_read(MPU9250_GYRO_CONFIG_REG); 
    regval &= (~0x3); // clear fchoise
    regval &= (~0x18); // clear afs
    regval |= (0x3 << 3);
    //TODO: scale

    // fchoice unchanged
    debug_printf("gyro config = %x\r\n",regval);
    IMU_write(MPU9250_GYRO_CONFIG_REG, regval);

    // accel config
    regval = IMU_read(MPU9250_ACCEL_CONFIG_REG);
    regval &= (~0x18); // clear afs
    regval |= (0x1<<3);// scale =/-4g

    // fchoice unchanges
    debug_printf("accel config = %x\r\n",regval);
    IMU_write(MPU9250_ACCEL_CONFIG_REG, regval);


    //
    // magnetometer config
    // See https://developer.mbed.org/users/kylongmu/code/MPU9250_SPI/file/084e8ba240c1/MPU9250.cpp

    //   {0x20, MPUREG_USER_CTRL},       // 0x6A =  0x20 I2C Master mode
    IMU_write(0x6A, 0x20);
    //   {0x0D, MPUREG_I2C_MST_CTRL}, //  0x24 = 0x0D I2C configuration multi-master  IIC 400KHz
    IMU_write(0x24, 0x0D);
    //    {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //0x25 = 0x0c  Set the I2C slave addres of AK8963 and set for write.
    IMU_write(0x25, 0x0C);
    //   {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //0x26 =0x0B   I2C slave 0 register address from where to begin data transfer
    IMU_write(0x26, 0x0B);
    //    {0x01, MPUREG_I2C_SLV0_DO}, // 0x63 = 0x01 Reset AK8963
    IMU_write(0x63, 0x01);
    //    {0x81, MPUREG_I2C_SLV0_CTRL},  // 0x27 = 0x81 Enable I2C and set 1 byte
    IMU_write(0x27, 0x81);

    //    {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //  0x26 = 0x0A I2C slave 0 register address from where to begin data transfer
    IMU_write(0x26, 0x0A);
    //    {0x12, MPUREG_I2C_SLV0_DO}, // 0x63 = 0x12 Register value to continuous measurement in 16bit
    IMU_write(0x63, 0x12);
    //    {0x81, MPUREG_I2C_SLV0_CTRL}  // 0x27 = 0x81 Enable I2C and set 1 byte
    IMU_write(0x27, 0x81);


    delay_us(1000);


    //uint8_t response;
    //WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    IMU_write(0x25, 0x0C | 0x80);
    //WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    IMU_write(0x26, 0x00);
    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
    IMU_write(0x27, 0x01 | 0x80);

    delay_us(1000);

    //wait(0.001);
    //response=WriteReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);    //Read I2C     

    // MPU9250_CONTROL1_REG set to 1 for continuous measurement mode 1
    regval = IMU_read(MPU9250_EXT_SENS_DATA_00_REG);
    uint8_t magn_expected_id = 0x48;
    if (regval != magn_expected_id){
        debug_printf("MAGN who am i?: result %d should be %d\n", regval, magn_expected_id);
        return;
    }

    // calibrate magnetometer
    //WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    IMU_write(0x25, 0x0C | 0x80);
    //WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); //I2C slave 0 register address from where to begin data transfer
    IMU_write(0x26, 0x10);
    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer
    IMU_write(0x27, 0x03 | 0x80);
    
 
    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    delay_us(1000);
    //ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);
    uint8_t buff[3];
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x80 | MPU9250_EXT_SENS_DATA_00_REG);
    uint8_t i;
    for (i = 0; i<3; ++i)
    {
        regval = SPI3_write(0x00);
        int16_t mag =  (int16_t)regval;
        float fmag = ((float)mag-128.0f)/256.0f + 1.0f;
        MAGN_SCALE_FACTORS[i] = fmag * MPU9250_MAGN_SENS_SCALING;
    }
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
}

void readIMU()
{
    uint16_t accel_x, accel_y, accel_z;
    uint16_t gyro_x, gyro_y, gyro_z;
    uint16_t magn_x, magn_y, magn_z;

    uint8_t buff[7];
    int i;


    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x80 | MPU9250_ACCEL_START_REG);
    for (i = 0; i<6; ++i) buff[i] = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    accel_x = (uint16_t)buff[0]<< 8 | buff[1];
    accel_y = (uint16_t)buff[2]<< 8 | buff[3];
    accel_z = (uint16_t)buff[4]<< 8 | buff[5];

    // we already have these in buff..
    aTxBuffer[REG_RW_ACCEL_X_H]  = (accel_x & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ACCEL_X_L] = (accel_x & 0x00FF);
    aTxBuffer[REG_RW_ACCEL_Y_H] = (accel_y & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ACCEL_Y_L] = (accel_y & 0x00FF);
    aTxBuffer[REG_RW_ACCEL_Z_H] = (accel_z & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ACCEL_Z_L] = (accel_z & 0x00FF);

    //debug_printf("%d %d %d\r\n", accel_x, accel_y, accel_z);
    float ax = (float)((int16_t)accel_x) / 8000.0;
    float ay = (float)((int16_t)accel_y) / 8000.0;
    float az = (float)((int16_t)accel_z) / 8000.0;
    //debug_printf("accel %d %d %d \n", (int16_t)accel_x, (int16_t)accel_y, (int16_t)accel_z);
    debug_printf("accel %f %f %f\n", ax, ay, az);


    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x80 | MPU9250_GYRO_START_REG);
    for (i = 0; i<6; ++i) buff[i] = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    gyro_x = (uint16_t)buff[0]<< 8 | buff[1];
    gyro_y = (uint16_t)buff[2]<< 8 | buff[3];
    gyro_z = (uint16_t)buff[4]<< 8 | buff[5];

    // we already have these in buff..
    aTxBuffer[REG_RW_GYRO_X_H]  = (gyro_x & 0xFF00) >> 8;
    aTxBuffer[REG_RW_GYRO_X_L] = (gyro_x & 0x00FF);
    aTxBuffer[REG_RW_GYRO_Y_H] = (gyro_y & 0xFF00) >> 8;
    aTxBuffer[REG_RW_GYRO_Y_L] = (gyro_y & 0x00FF);
    aTxBuffer[REG_RW_GYRO_Z_H] = (gyro_z & 0xFF00) >> 8;
    aTxBuffer[REG_RW_GYRO_Z_L] = (gyro_z & 0x00FF);

    //debug_printf("%d %d %d\r\n", gyro_x, gyro_y, gyro_z);
    // FIXME: scaling
    float gx = (float)((int16_t)gyro_x / 1000.0f);
    float gy = (float)((int16_t)gyro_y / 1000.0f);
    float gz = (float)((int16_t)gyro_z / 1000.0f);
    //debug_printf("accel %d %d %d \n", (int16_t)accel_x, (int16_t)accel_y, (int16_t)accel_z);
    debug_printf("gyro %f %f %f\n", gx, gy, gz);



    // read from magnetometer
    //WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    IMU_write(0x25, 0x0C | 0x80);
    //WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    IMU_write(0x26,0x03);
    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer
    IMU_write(0x27, 0x80 | 0x07);

    // a sleep is needed here while the control die inside the mpu9250 gets data from the accelerometer
    delay_us(1000);

    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x80 | MPU9250_EXT_SENS_DATA_00_REG);
    for (i = 0; i<7; ++i) buff[i] = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    magn_x = (((uint16_t)buff[1]) << 8) | buff[0];
    magn_y = (((uint16_t)buff[3]) << 8) | buff[2];
    magn_z = (((uint16_t)buff[5]) << 8 )| buff[4];

    // TODO: already have these in buff
    aTxBuffer[REG_RW_MAG_X_H]  = (magn_x & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MAG_X_L] = (magn_x & 0x00FF);
    aTxBuffer[REG_RW_MAG_Y_H] = (magn_y & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MAG_Y_L] = (magn_y & 0x00FF);
    aTxBuffer[REG_RW_MAG_Z_H] = (magn_z & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MAG_Z_L] = (magn_z & 0x00FF);


    //TODO real scale, calibration
    float mx = (float)((int16_t)magn_x) * MAGN_SCALE_FACTORS[0];
    float my = (float)((int16_t)magn_y) * MAGN_SCALE_FACTORS[1];
    float mz = (float)((int16_t)magn_z) * MAGN_SCALE_FACTORS[2];

    debug_printf("mag %f %f %f\n",mx, my, mz);

    // TODO: put mx, my mz back into wallaby registers
}


void setupAccelMag()
{
    configDigitalInPin(ACCEL_INT1, ACCEL_INT1_PORT);  
    configDigitalInPin(ACCEL_INT2, ACCEL_INT2_PORT);  

    // select Accel/Mag
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    delay_us(100); 

    uint8_t regval;
    // request "Who am I)
    SPI3_write(0x8F); // write register
    regval = SPI3_write(0x00); // write dummy val to get contents
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // chip select low

    if (regval == 73){
        debug_printf("Accel/Magn identified itself\n");
    }else{
        debug_printf("Accel/Magn did not respond/identify itself (regval=%d)\n",regval);
        return;
    }

    // accel
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x20);
    SPI3_write(0x57); // 50hz continuous, all axes
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x21);
    SPI3_write(0x00);  // 773Hz filter, +/- 2g scale, no self test, full duplex SPI
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    // accel data ready on int1
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0;
    SPI3_write(0x22);
    SPI3_write(0b00000100);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    // mag data ready on int1
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0;
    SPI3_write(0x23);
    SPI3_write(0b00000100);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    // magnetometer
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x24);
    SPI3_write(0x70); // temp disabled, high res, 50Hz, no interrupts
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x25);
    SPI3_write(0x20);  // +/- 4g
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x26);
    SPI3_write(0x00);  // normal
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
}

void readAccel()
{
    uint16_t accel_x, accel_y, accel_z;

    //x  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xA8);
    accel_x = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xA9);
    accel_x |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip


    //Y  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xAA);
    accel_y = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xAB);
    accel_y |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip


    //Z  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xAC);
    accel_z = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xAD);
    accel_z |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    aTxBuffer[REG_RW_ACCEL_X_H]  = (accel_x & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ACCEL_X_L] = (accel_x & 0x00FF);
    aTxBuffer[REG_RW_ACCEL_Y_H] = (accel_y & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ACCEL_Y_L] = (accel_y & 0x00FF);
    aTxBuffer[REG_RW_ACCEL_Z_H] = (accel_z & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ACCEL_Z_L] = (accel_z & 0x00FF);

    //float ax = (float)((int16_t)accel_x) / 16000.0;
    //float ay = (float)((int16_t)accel_y) / 16000.0;
    //float az = (float)((int16_t)accel_z) / 16000.0;
    //debug_printf("accel %d %d %d \n", (int16_t)accel_x, (int16_t)accel_y, (int16_t)accel_z);
    //debug_printf("accel %f %f %f\n", ax, ay, az);
}



void readMag()
{
    uint16_t mag_x, mag_y, mag_z;

    uint32_t int1 = ACCEL_INT1_PORT->IDR & ACCEL_INT1;
    uint32_t int2 = ACCEL_INT2_PORT->IDR & ACCEL_INT2;

    //x  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x88);
    mag_x = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x89);
    mag_x |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip


    //Y  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x8A);
    mag_y = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x8B);
    mag_y |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip


    //Z  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x8C);
    mag_z = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x8D);
    mag_z |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip


    aTxBuffer[REG_RW_MAG_X_H] = (mag_x & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MAG_X_L] = (mag_x & 0x00FF);
    aTxBuffer[REG_RW_MAG_Y_H] = (mag_y & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MAG_Y_L] = (mag_y & 0x00FF);
    aTxBuffer[REG_RW_MAG_Z_H] = (mag_z & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MAG_Z_L] = (mag_z & 0x00FF);

    //float mx = (float)((int16_t)mag_x) / 16000.0;
    //float my = (float)((int16_t)mag_y) / 16000.0;
    //float mz = (float)((int16_t)mag_z) / 16000.0;
    //debug_printf("accel %d %d %d \n", (int16_t)accel_x, (int16_t)accel_y, (int16_t)accel_z);
    //debug_printf("mag %f %f %f  (int1 %d   int2 %d)\n", mx, my, mz, int1, int2);
}




void setupGyro()
{
    configDigitalInPin(GYRO_DATA_READY, GYRO_DATA_READY_PORT);    

    // select Accel/Mag
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    delay_us(100); 

    uint8_t regval;
    // request "Who am I)
    SPI3_write(0x80 | 0x0F); // write register
    regval = SPI3_write(0x00); // write dummy val to get contents
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // chip select low

    if (regval == GYRO_ID){
      //debug_printf("Gyro identified itself\n");
    }else{
      //debug_printf("Gyro did not respond/identify itself (regval=%d)\n",regval);
      return;
    }

    // ctrl1
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1;
    SPI3_write(0x20);
    SPI3_write(0b00001111);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;

    // ctrl2
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1;
    SPI3_write(0x21);
    SPI3_write(0b00000000);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;

    // ctrl3
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1;
    SPI3_write(0x22);
    SPI3_write(0b00001000); // use data ready pin
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;

    // ctrl4
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1;
    SPI3_write(0x23);
    SPI3_write(0b00000000);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;

    // ctrl5
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1;
    SPI3_write(0x24);
    SPI3_write(0b00000000);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;
}


void readGyro()
{
    uint16_t gyro_x, gyro_y, gyro_z;

    uint32_t gyro_data_ready = GYRO_DATA_READY_PORT->IDR & GYRO_DATA_READY;

    //x  TODO high or low first
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x28);
    gyro_x = SPI3_write(0x00);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip
  
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x29);
    gyro_x |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip


    //y  TODO high or low first
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x2A);
    gyro_y = SPI3_write(0x00);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip
  
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x2B);
    gyro_y |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip



    //z  TODO high or low first
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x2C);
    gyro_z = SPI3_write(0x00);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip
  
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x2D);
    gyro_z |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip


    aTxBuffer[REG_RW_GYRO_X_H] = (gyro_x & 0xFF00) >> 8;
    aTxBuffer[REG_RW_GYRO_X_L] = (gyro_x & 0x00FF);
    aTxBuffer[REG_RW_GYRO_Y_H] = (gyro_y & 0xFF00) >> 8;
    aTxBuffer[REG_RW_GYRO_Y_L] = (gyro_y & 0x00FF);
    aTxBuffer[REG_RW_GYRO_Z_H] = (gyro_z & 0xFF00) >> 8;
    aTxBuffer[REG_RW_GYRO_Z_L] = (gyro_z & 0x00FF);
}


