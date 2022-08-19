#include "wallaby_imu.h"
#include "wallaby.h"

#define WALLABY2

#define MPU9250_WHO_AMI_I_REG 0x75
#define MPU9250_WHO_AMI_I_RES 0x71
#define MPU9250_CONFIG_REG 0x1A
#define MPU9250_GYRO_CONFIG_REG 0x1B
#define MPU9250_ACCEL_CONFIG_REG 0x1C
#define MPU9250_ACCEL_CONFIG2_REG 0x1D
#define MPU9250_ACCEL_START_REG 0x3B   // xh xl yh yl zh zl
#define MPU9250_GYRO_START_REG 0x43    // xh xl yh yl zh zl
#define MPU9250_MAGN_START_REG 0x03    // xl xh yl yh zl zh
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
    return ret;
}

uint8_t IMU_read(uint8_t address)
{
    uint8_t ret;
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x80 | address);
    ret = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
    return ret;
}

void setupIMU()
{
    debug_printf("inside setupIMU\r\n");
    // select Accel/Mag
    // SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    delay_us(200);

    uint8_t regval;
    uint8_t sensitivity;
    uint8_t sensitivity_byte;

    // request "Who am I"
    regval = IMU_read(MPU9250_WHO_AMI_I_REG);

    if (regval == MPU9250_WHO_AMI_I_RES)
    { // mpu9250
        debug_printf("IMU identified itself\r\n");
    }
    else
    {
        debug_printf("IMU did not respond/identify itself (regval=%d)\r\n", regval);
        return;
    }

    //{0x80, MPUREG_PWR_MGMT_1},     // Reset Device
    IMU_write(0x6B, 0x80);

    // wake up
    IMU_write(0x6B, 0x00); // power management 1, 50hz continuous, all axes
    delay_us(100);

    IMU_write(0x6B, 0x01); // power management 1, select time source
    delay_us(200);

    // IMU_write(MPU9250_CONFIG_REG, 0x03); // config gyro/accel sample rate 200Hz
    IMU_write(MPU9250_CONFIG_REG, 0x00);

    // IMU_write(0x19, 0x04); // samplerate div 200 Hz
    IMU_write(0x19, 0x00);

    // ---------- gyro config ----------
    // config sensitivity
    // valid values are 0, 1, 2, 3
    // they correspond to [250, 500, 1000, 2000]
    /*
        Gyro Full Scale Select:
        00 = +250dps
        01= +500 dps
        10 = +1000 dps
        11 = +2000 dps
    */
    sensitivity = 0; // 250 dps
    switch (sensitivity)
    {
    case 0:
    {
        sensitivity_byte = 0b00000;
        break;
    }
    case 1:
    {
        sensitivity_byte = 0b01000;
        break;
    }
    case 2:
    {
        sensitivity_byte = 0b10000;
        break;
    }
    case 3:
    {
        sensitivity_byte = 0b11000;
        break;
    }
    }

    debug_printf("gyro config = %x\r\n", sensitivity_byte);
    IMU_write(MPU9250_GYRO_CONFIG_REG, sensitivity_byte);
    aTxBuffer[REG_GYRO_SENSITIVITY] = sensitivity; // 0, 1, 2, or 3

    // ---------- accel config ----------
    // config sensitivity
    // valid values are 0, 1, 2, 3
    // they correspond to [2g, 4g, 8g, 16g]
    /*
        Accel Full Scale Select:
        ±2g (00), ±4g (01), ±8g (10), ±16g (11)
    */
    sensitivity = 0;
    switch (sensitivity)
    {
    case 0:
    {
        sensitivity_byte = 0b00000;
        break;
    }
    case 1:
    {
        sensitivity_byte = 0b01000;
        break;
    }
    case 2:
    {
        sensitivity_byte = 0b10000;
        break;
    }
    case 3:
    {
        sensitivity_byte = 0b11000;
        break;
    }
    }

    // fchoice unchanges
    debug_printf("accel config = %x\r\n", sensitivity_byte);
    IMU_write(MPU9250_ACCEL_CONFIG_REG, sensitivity_byte);
    aTxBuffer[REG_ACCEL_SENSITIVITY] = sensitivity; // 0, 1, 2, or 3

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

    // uint8_t response;
    // WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    IMU_write(0x25, 0x0C | 0x80);
    // WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    IMU_write(0x26, 0x00);
    // WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
    IMU_write(0x27, 0x01 | 0x80);

    delay_us(1000);

    // wait(0.001);
    // response=WriteReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);    //Read I2C

    // MPU9250_CONTROL1_REG set to 1 for continuous measurement mode 1
    regval = IMU_read(MPU9250_EXT_SENS_DATA_00_REG);
    uint8_t magn_expected_id = 0x48;
    if (regval != magn_expected_id)
    {
        debug_printf("MAGN who am i?: result %d should be %d\n", regval, magn_expected_id);
        return;
    }

    // calibrate magnetometer
    // WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    IMU_write(0x25, 0x0C | 0x80);
    // WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); //I2C slave 0 register address from where to begin data transfer
    IMU_write(0x26, 0x10);
    // WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer
    IMU_write(0x27, 0x03 | 0x80);

    // WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    delay_us(1000);
    // ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);
    // uint8_t buff[3];  // Eliot - commented this out bc was an unused variable
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x80 | MPU9250_EXT_SENS_DATA_00_REG);
    uint8_t i;
    for (i = 0; i < 3; ++i)
    {
        regval = SPI3_write(0x00);
        int16_t mag = (int16_t)regval;
        float fmag = ((float)mag - 128.0f) / 256.0f + 1.0f;
        MAGN_SCALE_FACTORS[i] = fmag * MPU9250_MAGN_SENS_SCALING;
    }
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
}

void readIMU()
{
    uint8_t buff[7];
    int i;

    // ---------- accelerometer ----------
    // read accel
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x80 | MPU9250_ACCEL_START_REG);
    for (i = 0; i < 6; ++i)
        buff[i] = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    // write to the buffer that the wombat uses
    aTxBuffer[REG_RW_ACCEL_X_H] = buff[0];
    aTxBuffer[REG_RW_ACCEL_X_L] = buff[1];
    aTxBuffer[REG_RW_ACCEL_Y_H] = buff[2];
    aTxBuffer[REG_RW_ACCEL_Y_L] = buff[3];
    aTxBuffer[REG_RW_ACCEL_Z_H] = buff[4];
    aTxBuffer[REG_RW_ACCEL_Z_L] = buff[5];

    // ---------- gyrometer ----------
    // read gyro
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x80 | MPU9250_GYRO_START_REG);
    for (i = 0; i < 6; ++i)
        buff[i] = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    // write to the buffer that the wombat uses
    aTxBuffer[REG_RW_GYRO_X_H] = buff[0];
    aTxBuffer[REG_RW_GYRO_X_L] = buff[1];
    aTxBuffer[REG_RW_GYRO_Y_H] = buff[2];
    aTxBuffer[REG_RW_GYRO_Y_L] = buff[3];
    aTxBuffer[REG_RW_GYRO_Z_H] = buff[4];
    aTxBuffer[REG_RW_GYRO_Z_L] = buff[5];

    // ---------- magnetometer ----------
    // read magnetometer
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x80 | MPU9250_EXT_SENS_DATA_00_REG);
    for (i = 0; i < 7; ++i)
        buff[i] = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    // write to the buffer that the wombat uses
    aTxBuffer[REG_RW_MAG_X_H] = buff[0];
    aTxBuffer[REG_RW_MAG_X_L] = buff[1];
    aTxBuffer[REG_RW_MAG_Y_H] = buff[2];
    aTxBuffer[REG_RW_MAG_Y_L] = buff[3];
    aTxBuffer[REG_RW_MAG_Z_H] = buff[4];
    aTxBuffer[REG_RW_MAG_Z_L] = buff[5];
}
