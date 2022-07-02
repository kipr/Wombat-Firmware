#include "wallaby_imu.h"
#include "wallaby.h"

#include "mpu9250.h"
#include "stm32f4xx.h"


#define WALLABY2

#define MPU9250_WHO_AMI_I_REG  0x75
#define MPU9250_WHO_AMI_I_RES  0x71
#define MPU9250_CONFIG_REG 0x1A
#define MPU9250_GYRO_CONFIG_REG 0x1B
#define MPU9250_ACCEL_CONFIG_REG 0x1C
#define MPU9250_ACCEL_CONFIG2_REG 0x1D
#define MPU9250_LP_ACCEL_ODR_REG 0x1E
#define MPU9250_ACCEL_START_REG 0x3B // xh xl yh yl zh zl
#define MPU9250_GYRO_START_REG 0x43 // xh xl yh yl zh zl
#define MPU9250_MAGN_START_REG 0x03 // xl xh yl yh zl zh
#define MPU9250_MAGN_CONTROL1_REG 0x0A // control register 1
#define MPU9250_MAGN_SENS_SCALING ((float)0.15f)
#define MPU9250_EXT_SENS_DATA_00_REG 0x49 // magnetometer can be available here

float MAGN_SCALE_FACTORS[3] = {0.0f, 0.0f, 0.0f};

mpu9250 *mpu;

void wallaby_imu_init()
{
  debug_printf("inside setupIMU\r\n");

  mpu = mpu9250_create(SPI3, SPI3_CS0_PORT, SPI3_CS0);
  if (!mpu)
  {
    debug_printf("mpu9250_create failed\r\n");
    return;
  }

  // Reset device
  mpu9250_pwr_mgmt_1_set(mpu, &(mpu9250_pwr_mgmt_1) {
    .h_reset = 1,
    .sleep = 0,
    .cycle = 0,
    .gyro_standby = 0,
    .pd_ptat = 0,
    .clksel = 0,
  });
  
  delay_us(100 * 1000);

  // Wake up
  mpu9250_pwr_mgmt_1_set(mpu, &(mpu9250_pwr_mgmt_1) {
    .h_reset = 0,
    .sleep = 1,
    .cycle = 0,
    .gyro_standby = 0,
    .pd_ptat = 0,
    .clksel = 0,
  });
  delay_us(100 * 1000);


  // Configure clock
  mpu9250_pwr_mgmt_1_set(mpu, &(mpu9250_pwr_mgmt_1) {
    .h_reset = 0,
    .sleep = 0,
    .cycle = 0,
    .gyro_standby = 0,
    .pd_ptat = 0,
    .clksel = 1,
  });
  delay_us(100 * 1000);

  // Enable Accelerometer and Gyroscope
  mpu9250_pwr_mgmt_2_set(mpu, &(mpu9250_pwr_mgmt_2) {
    .disable_xa = 0,
    .disable_ya = 0,
    .disable_za = 0,
    .disable_xg = 0,
    .disable_yg = 0,
    .disable_zg = 0,
  });

  
  // Set config
  mpu9250_config_set(mpu, &(mpu9250_config) {
    .fifo_mode = 0,
    .ext_sync_set = 0,
    // 41 Hz
    .dlpf_cfg = 3,
  });

  // Set sample rate
  mpu9250_smplrt_div_set(mpu, 4);

  mpu9250_gyro_config_set(mpu, &(mpu9250_gyro_config) {
    .xgyro_cten = 0,
    .ygyro_cten = 0,
    .zgyro_cten = 0,
    // +2000 dps
    .gyro_fs_sel = 0x3,
    .fchoice_b = 0x3,
  });

  mpu9250_accel_config_set(mpu, &(mpu9250_accel_config) {
    .ax_st_en = 0,
    .ay_st_en = 0,
    .az_st_en = 0,
    // +/- 4g
    .accel_fs_sel = 0x1,
  });

  // magnetometer config
  // See https://developer.mbed.org/users/kylongmu/code/MPU9250_SPI/file/084e8ba240c1/MPU9250.cpp

  // I2C Master mode
  mpu9250_user_ctrl_set(mpu, &(mpu9250_user_ctrl) {
    .fifo_en = 0,
    .i2c_mst_en = 1,
    .i2c_if_dis = 0,
    .fifo_rst = 0,
    .i2c_mst_rst = 0,
    .sig_cond_rst = 0
  });

  // I2C configuration multi-master  IIC 400KHz
  mpu9250_i2c_mst_ctrl_set(mpu, &(mpu9250_i2c_mst_ctrl) {
    .mult_mst_en = 0,
    .wait_for_es = 0,
    .slv_3_fifo_en = 0,
    .i2c_mst_p_nsr = 0,
    .i2c_mst_clk = 0xD,
  });

  // Set the I2C slave addres of AK8963 and set for write.
  mpu9250_i2c_slvn_addr_set(mpu, 0, 0x0C);

  // I2C slave 0 register address from where to begin data transfer
  mpu9250_i2c_slvn_reg_set(mpu, 0, 0x0B);

  // Reset AK8963
  mpu9250_i2c_slvn_do_set(mpu, 0, 0x91);

  // Enable I2C and send 1 byte
  mpu9250_i2c_slvn_ctrl_set(mpu, 0, &(mpu9250_i2c_ctrl) {
    .i2c_slv_en = 1,
    .slv_done_int_en = 0,
    .i2c_slv_reg_dis = 0,
    .i2c_mst_dly = 1,
  });

  // I2C slave 0 register address from where to begin data transfer
  mpu9250_i2c_slvn_reg_set(mpu, 0, 0x0A);

  // Register value to continuous measurement in 16bit
  mpu9250_i2c_slvn_do_set(mpu, 0, 0x12);

  // Enable I2C and send 1 byte
  mpu9250_i2c_slvn_ctrl_set(mpu, 0, &(mpu9250_i2c_ctrl) {
    .i2c_slv_en = 1,
    .slv_done_int_en = 0,
    .i2c_slv_reg_dis = 0,
    .i2c_mst_dly = 1,
  });
  
  delay_us(1000);

  // Set the I2C slave addres of AK8963 and set for read.
  mpu9250_i2c_slvn_addr_set(mpu, 0, 0x0C | 0x80);

  // I2C slave 0 register address from where to begin data transfer
  mpu9250_i2c_slvn_reg_set(mpu, 0, 0);

  // Read 1 byte from magnetometer
  mpu9250_i2c_slvn_ctrl_set(mpu, 0, &(mpu9250_i2c_ctrl) {
    .i2c_slv_en = 1,
    .slv_done_int_en = 0,
    .i2c_slv_reg_dis = 0,
    .i2c_mst_dly = 1,
  });

  delay_us(1000); 

  // MPU9250_CONTROL1_REG set to 1 for continuous measurement mode 1
  uint8_t regval = mpu9250_read(mpu, MPU9250_EXT_SENS_DATA_00);
  uint8_t magn_expected_id = 0x48;
  if (regval != magn_expected_id){
      debug_printf("MAGN who am i?: result %d should be %d\n", regval, magn_expected_id);
      return;
  }

  // calibrate magnetometer

  // Set the I2C slave addres of AK8963 and set for read.
  mpu9250_i2c_slvn_addr_set(mpu, 0, 0x0C | 0x80);

  // I2C slave 0 register address from where to begin data transfer
  mpu9250_i2c_slvn_reg_set(mpu, 0, 0x10);

  // Read 3 bytes from the magnetometer
  mpu9250_i2c_slvn_ctrl_set(mpu, 0, &(mpu9250_i2c_ctrl) {
    .i2c_slv_en = 1,
    .slv_done_int_en = 0,
    .i2c_slv_reg_dis = 0,
    .i2c_mst_dly = 3,
  });

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

uint32_t last_us_count = 0;
void wallaby_imu_update()
{
  uint32_t start = usCount;
  mpu9250_sample accel_sample;
  mpu9250_accel_sample_read(mpu, &accel_sample);
  mpu9250_accel_sample_write_regs(mpu, &accel_sample, aTxBuffer);

  mpu9250_sample gyro_sample;
  mpu9250_gyro_sample_read(mpu, &gyro_sample);
  mpu9250_gyro_sample_write_regs(mpu, &gyro_sample, aTxBuffer);

  mpu9250_sample magneto_sample;
  mpu9250_magneto_sample_read(mpu, &magneto_sample);
  uint32_t end = usCount;

  uint32_t delta = usCount - last_us_count;
  magneto_sample.x = delta / 1000;
  magneto_sample.y = delta > INT16_MAX ? INT16_MAX : INT16_MIN;
  magneto_sample.z = (end - start);
  mpu9250_magneto_sample_write_regs(mpu, &magneto_sample, aTxBuffer);
  last_us_count = usCount;
}

