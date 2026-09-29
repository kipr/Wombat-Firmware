#include "wallaby_imu.h"
#include "wallaby.h"
#include "mpu9250regmap.h"
#include <stdint.h>

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

/**
    The defualt sensitivity of the gyroscope
    valid values are 0, 1, 2, 3
    they correspond to [250, 500, 1000, 2000]

    sensitivity value | gyroscope sensitivity | byte
    0 | 250  | 0b00000000
    1 | 500  | 0b00001000
    2 | 1000 | 0b00010000
    3 | 2000 | 0b00011000
*/
#define GYRO_DEFAULT_SENSITIVITY_BYTE 0b00000000

// when we invert this, we configure gyroscope to be at 1khz and bandwidth 41Hz
#define GYRO_FCHOICE_BYTE 0b00000011

/*
    The default sensitivity of the accelerometer
    valid values are 0, 1, 2, 3
    they correspond to [±2g, ±4g, ±8g, ±16g]

    sensitivity value | gyroscope sensitivity | byte
    0 | ±2g | 0b00000
    1 | ±4g | 0b01000
    2 | ±8g | 0b10000
    3 | ±16g | 0b11000
*/
#define ACCEL_DEFAULT_SENSITIVITY_BYTE 0b00000

// modes are determined by bytes 0-3
//  0000 = power down
//  0001 = single measurement
//  0010 = continuous measurement @ 8hz
//  0110 = continuous measurement @ 100Hz
//  0100 = external trigger measurement mode
//  1000 = self-test mode
//  1111 = Fuse ROM access mode
// #bits is determined by byte 4
//      value of 0 = 14 bits
//      value of 1 = 16 bits
#define MAGNETOMETER_CONFIG_BYTE 0b00010110

/**
 * This is the flag used by SPI to indicate
 * that the transfer is a read and not a write
 *
 */
#define READ_FLAG 0b10000000

float magn_scale_factors[3] = {0.0f, 0.0f, 0.0f};

/**
 * @brief Write a byte to the given address. This doesn't
 * take care of sleeping.
 *
 * @param address
 * @param val
 * @return uint8_t
 */
uint8_t IMU_write(uint8_t address, uint8_t val)
{
    uint8_t ret;
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(address);
    ret = SPI3_write(val);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
    delay_us(10);
    return ret;
}

/**
 * @brief Read a byte from the given address. This doesn't
 * take care of sleeping.
 *
 * @param address
 * @return uint8_t
 */
uint8_t IMU_read(uint8_t address)
{
    uint8_t ret;
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(READ_FLAG | address);
    ret = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
    return ret;
}

void enter_magnetometer_write_mode()
{
    // the first byte is whether it is a write or read
    // 1 for read, 0 for write
    IMU_write(I2C_SLV0_ADDR, AK8963_PHYSICAL_LOCAT);
}
/**
 * @brief Use this if you won't be needing to access
 * DO
 *
 */
void enter_magnetometer_read_mode()
{
    // the first byte is whether
    // 1 for read
    IMU_write(I2C_SLV0_ADDR, AK8963_PHYSICAL_LOCAT | READ_FLAG);
}

/**
 * @brief Use this in write mode to write
 * to the DO
 *
 * @param ak8963_addr
 * @param val
 * @return uint8_t
 */
uint8_t magnetometer_write(uint8_t ak8963_addr, uint8_t val)
{
    IMU_write(I2C_SLV0_REG, ak8963_addr);
    return IMU_write(I2C_SLV0_DO, val);
}

/**
 * @brief Read `num_bytes` bytes, starting from `reg_start` into the provided buffer
 *
 * @param reg_start the starting register
 * @param num_bytes how many bytes to read
 * @param[out] out the buffer to write to
 */
void read_bytes(uint8_t reg_start, uint8_t num_bytes, uint8_t *out)
{
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0;
    SPI3_write(reg_start | READ_FLAG); // start reading from the external sensor data
    for (uint8_t i = 0; i < num_bytes; ++i)
    {
        out[i] = SPI3_write(0x00); // write null byte to request data
    }
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0;
}

/**
 * @brief Read `num_bytes` from the magnetometer
 *
 * @param ak8963_addr_start the address on the ak8963 to start reading from
 * @param num_bytes number of bytes to read, from 1 to 15 inclusive
 * @param[out] out the buffer to write to
 */
void magnetometer_read_bytes(uint8_t ak8963_addr_start, uint8_t num_bytes, uint8_t *out)
{
    // enter read mode
    enter_magnetometer_read_mode();
    // go to the register where we start reading bytes from so that data will be available at external sensor data
    IMU_write(I2C_SLV0_REG, ak8963_addr_start);
    // configure to read `num_bytes` bytes
    IMU_write(I2C_SLV0_CTRL, READ_FLAG | num_bytes);

    // read the data:
    read_bytes(EXT_SENS_DATA_00, num_bytes, out);
}

void setup_mpu9250()
{
    // reset device
    IMU_write(PWR_MGMT_1, 0x80);
    delay_us(100);

    // wake up device
    IMU_write(PWR_MGMT_1, 0x00);
    delay_us(100);

    // get stable time source
    IMU_write(PWR_MGMT_1, 0x01);
    delay_us(200);

    // set SR to 200Hz rate
    IMU_write(MPU_CONFIG, 0x03);
    IMU_write(SMPLRT_DIV, 0x04);
}

void setup_gyro()
{
    uint8_t regval = IMU_read(GYRO_CONFIG);
    regval &= ~0x03;                         // clear fchoise [1:0]
    regval &= ~0x18;                         // clear GYRO_FS_SEL bits [4:3]
    regval |= GYRO_DEFAULT_SENSITIVITY_BYTE; // set sensitivity
    regval |= (~GYRO_FCHOICE_BYTE);          // set fchoice
    IMU_write(GYRO_CONFIG, regval);
}

void setup_accel()
{
    // accel sensitivity
    uint8_t regval = IMU_read(ACCEL_CONFIG);
    regval &= ~0xE0;                          // clear self-test bits [7:5]
    regval &= ~0x18;                          // clear ACCEL_FS_SEL bits [4:3]
    regval |= ACCEL_DEFAULT_SENSITIVITY_BYTE; // set sensitivity
    IMU_write(ACCEL_CONFIG, regval);
}

void setup_magnetometer()
{
    // setup using the mpu9250 as master i2c reader
    IMU_write(USER_CTRL, 0b00100000);    // enable I2C master mode
    IMU_write(I2C_MST_CTRL, 0b00001101); // set I2C Master clock to 400kHz

    // put AK8963 in I2C slv 0
    enter_magnetometer_write_mode();

    // setup AK8963
    magnetometer_write(AK8963_CNTL, 0x00); // turn off magnetometer
    delay_us(10);
    magnetometer_write(AK8963_CNTL, 0x0F); // enter Fuse ROM access mode
    delay_us(10);

    // read bias factors
    uint8_t raw_data[3];
    magnetometer_read_bytes(AK8963_ASAX, 3, (uint8_t *)raw_data);
    for (uint8_t i = 0; i < 3; ++i)
    {
        // according to the datasheet, multiply this by the read magnetometer
        // data to get the adjusted data
        magn_scale_factors[i] = (float)(raw_data[0] - 128) / 256. + 1.;
    }

    // power down magnetometer again
    enter_magnetometer_write_mode();
    magnetometer_write(AK8963_CNTL, 0x00); // write the power-down byte
    delay_us(10);

    // go into continuous measurement @ 100Hz and use 16 bit values
    magnetometer_write(AK8963_CNTL, MAGNETOMETER_CONFIG_BYTE);
    delay_us(10);
}

void setupIMU()
{
    delay_us(200); // wait a little before starting

    // setup mpu9250 devices
    setup_mpu9250();
    setup_gyro();
    setup_accel();

    delay_us(200); // necessary wait before setting up the AK8963 (magnetometer)
    setup_magnetometer();

    delay_us(200); // wait a little before exiting
}

void readIMU(uint32_t count)
{
    const uint8_t accel_gyro_update_mod = 10; // at about 200 hz
    const uint8_t magneto_update_mod = 20;    // at about 100 hz

    if (count % accel_gyro_update_mod == 0)
    {
        // ---------- accelerometer ----------
        read_bytes(ACCEL_XOUT_H, 6, ((uint8_t *)aTxBuffer) + REG_RW_ACCEL_X_H);

        // according to datasheet, need 1.88 ms in total to finalize data for accel
        delay_us(1880);

        // ---------- gyrometer ----------
        read_bytes(GYRO_XOUT_H, 6, ((uint8_t *)aTxBuffer) + REG_RW_GYRO_X_H);

        // we give it a little more just in case
        delay_us(100);
    }
    if (count % magneto_update_mod == 0)
    {
        // ---------- magnetometer ----------
        uint8_t buff[7];
        // ignore reading from data status 1 since, during testing, it didn't respond
        magnetometer_read_bytes(AK8963_XOUT_L, 7, buff); // read magneto data and data status 2

        // data is in little endian (little byte, then big byte)
        // so we have to swap the order (put it in big
        // endian form) before putting it in buffer
        aTxBuffer[REG_RW_MAG_X_H] = buff[1];
        aTxBuffer[REG_RW_MAG_X_L] = buff[0];
        aTxBuffer[REG_RW_MAG_Y_H] = buff[3];
        aTxBuffer[REG_RW_MAG_Y_L] = buff[2];
        aTxBuffer[REG_RW_MAG_Z_H] = buff[5];
        aTxBuffer[REG_RW_MAG_Z_L] = buff[4];
    }
}