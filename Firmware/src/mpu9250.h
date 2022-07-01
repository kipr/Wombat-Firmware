/**
 * @file mpu9250.h
 * @author Braden McDorman (bmcdorman@gmail.com)
 * @brief An implementation of SPI communication with the MPU9250.
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "stm32f4xx.h"
#include <stdint.h>

extern const uint8_t MPU9250_SELF_TEST_X_GYRO;
extern const uint8_t MPU9250_SELF_TEST_Y_GYRO;
extern const uint8_t MPU9250_SELF_TEST_Z_GYRO;
extern const uint8_t MPU9250_SELF_TEST_X_ACCEL;
extern const uint8_t MPU9250_SELF_TEST_Y_ACCEL;
extern const uint8_t MPU9250_SELF_TEST_Z_ACCEL;
extern const uint8_t MPU9250_XG_OFFSET_H;
extern const uint8_t MPU9250_XG_OFFSET_L;
extern const uint8_t MPU9250_YG_OFFSET_H;
extern const uint8_t MPU9250_YG_OFFSET_L;
extern const uint8_t MPU9250_ZG_OFFSET_H;
extern const uint8_t MPU9250_ZG_OFFSET_L;
extern const uint8_t MPU9250_SMPLRT_DIV;
extern const uint8_t MPU9250_CONFIG;
extern const uint8_t MPU9250_GYRO_CONFIG;
extern const uint8_t MPU9250_ACCEL_CONFIG;
extern const uint8_t MPU9250_ACCEL_CONFIG_2;
extern const uint8_t MPU9250_LP_ACCEL_ODR;
extern const uint8_t MPU9250_WOM_THR;
extern const uint8_t MPU9250_FIFO_EN;
extern const uint8_t MPU9250_I2C_MST_CTRL;
extern const uint8_t MPU9250_I2C_SLV0_ADDR;
extern const uint8_t MPU9250_I2C_SLV0_REG;
extern const uint8_t MPU9250_I2C_SLV0_CTRL;
extern const uint8_t MPU9250_I2C_SLV1_ADDR;
extern const uint8_t MPU9250_I2C_SLV1_REG;
extern const uint8_t MPU9250_I2C_SLV1_CTRL;
extern const uint8_t MPU9250_I2C_SLV2_ADDR;
extern const uint8_t MPU9250_I2C_SLV2_REG;
extern const uint8_t MPU9250_I2C_SLV2_CTRL;
extern const uint8_t MPU9250_I2C_SLV3_ADDR;
extern const uint8_t MPU9250_I2C_SLV3_REG;
extern const uint8_t MPU9250_I2C_SLV4_ADDR;
extern const uint8_t MPU9250_I2C_SLV4_REG;
extern const uint8_t MPU9250_I2C_SLV4_DO;
extern const uint8_t MPU9250_I2C_SLV4_CTRL;
extern const uint8_t MPU9250_I2C_SLV4_DI;
extern const uint8_t MPU9250_I2C_MST_STATUS;
extern const uint8_t MPU9250_INT_PIN_CFG;
extern const uint8_t MPU9250_INT_ENABLE;
extern const uint8_t MPU9250_INT_STATUS;
extern const uint8_t MPU9250_ACCEL_XOUT_H;
extern const uint8_t MPU9250_ACCEL_XOUT_L;
extern const uint8_t MPU9250_ACCEL_YOUT_H;
extern const uint8_t MPU9250_ACCEL_YOUT_L;
extern const uint8_t MPU9250_ACCEL_ZOUT_H;
extern const uint8_t MPU9250_ACCEL_ZOUT_L;
extern const uint8_t MPU9250_TEMP_OUT_H;
extern const uint8_t MPU9250_TEMP_OUT_L;
extern const uint8_t MPU9250_GYRO_XOUT_H;
extern const uint8_t MPU9250_GYRO_XOUT_L;
extern const uint8_t MPU9250_GYRO_YOUT_H;
extern const uint8_t MPU9250_GYRO_YOUT_L;
extern const uint8_t MPU9250_GYRO_ZOUT_H;
extern const uint8_t MPU9250_GYRO_ZOUT_L;
extern const uint8_t MPU9250_EXT_SENS_DATA_00;
extern const uint8_t MPU9250_EXT_SENS_DATA_01;
extern const uint8_t MPU9250_EXT_SENS_DATA_02;
extern const uint8_t MPU9250_EXT_SENS_DATA_03;
extern const uint8_t MPU9250_EXT_SENS_DATA_04;
extern const uint8_t MPU9250_EXT_SENS_DATA_05;
extern const uint8_t MPU9250_EXT_SENS_DATA_06;
extern const uint8_t MPU9250_EXT_SENS_DATA_07;
extern const uint8_t MPU9250_EXT_SENS_DATA_08;
extern const uint8_t MPU9250_EXT_SENS_DATA_09;
extern const uint8_t MPU9250_EXT_SENS_DATA_10;
extern const uint8_t MPU9250_EXT_SENS_DATA_11;
extern const uint8_t MPU9250_EXT_SENS_DATA_12;
extern const uint8_t MPU9250_EXT_SENS_DATA_13;
extern const uint8_t MPU9250_EXT_SENS_DATA_14;
extern const uint8_t MPU9250_EXT_SENS_DATA_15;
extern const uint8_t MPU9250_EXT_SENS_DATA_16;
extern const uint8_t MPU9250_EXT_SENS_DATA_17;
extern const uint8_t MPU9250_EXT_SENS_DATA_18;
extern const uint8_t MPU9250_EXT_SENS_DATA_19;
extern const uint8_t MPU9250_EXT_SENS_DATA_20;
extern const uint8_t MPU9250_EXT_SENS_DATA_21;
extern const uint8_t MPU9250_EXT_SENS_DATA_22;
extern const uint8_t MPU9250_EXT_SENS_DATA_23;
extern const uint8_t MPU9250_I2C_SLV0_D0;
extern const uint8_t MPU9250_I2C_SLV1_D0;
extern const uint8_t MPU9250_I2C_SLV2_D0;
extern const uint8_t MPU9250_I2C_SLV3_D0;
extern const uint8_t MPU9250_I2C_MST_DELAY_CTRL;
extern const uint8_t MPU9250_SIGNAL_PATH_RESET;
extern const uint8_t MPU9250_MOT_DETECT_CTRL;
extern const uint8_t MPU9250_USER_CTRL;
extern const uint8_t MPU9250_PWR_MGMT_1;
extern const uint8_t MPU9250_PWR_MGMT_2;
extern const uint8_t MPU9250_FIFO_COUNTH;
extern const uint8_t MPU9250_FIFO_COUNTL;
extern const uint8_t MPU9250_FIFO_R_W;
extern const uint8_t MPU9250_WHO_AM_I;
extern const uint8_t MPU9250_XA_OFFSET_H;
extern const uint8_t MPU9250_XA_OFFSET_L;
extern const uint8_t MPU9250_YA_OFFSET_H;
extern const uint8_t MPU9250_YA_OFFSET_L;
extern const uint8_t MPU9250_ZA_OFFSET_H;
extern const uint8_t MPU9250_ZA_OFFSET_L;

typedef struct
{
  SPI_TypeDef *spi;
  uint16_t port;
  GPIO_TypeDef *cs0;
  uint16_t cs0_pin;
} mpu9250;

mpu9250 *mpu9250_create(SPI_TypeDef *spi, uint16_t port, GPIO_TypeDef *cs0, uint16_t cs0_pin);
void mpu9250_destroy(mpu9250 *const device);

uint8_t mpu9250_write(mpu9250 *const device, uint8_t address, uint8_t val);
uint8_t mpu9250_read(mpu9250 *const device, uint8_t address);
uint8_t mpu9250_read_buffer(mpu9250 *const device, const uint8_t address, uint8_t *const buffer, const uint8_t length);

// PWR_MGMT_1

typedef struct
{
  /// 1 – Reset the internal registers and restores the default settings. Write a 1 to
  /// set the reset, the bit will auto clear.
  uint8_t h_reset : 1;

  /// When set, the chip is set to sleep mode (After OTP loads, the
  /// PU_SLEEP_MODE bit will be written here)
  uint8_t sleep : 1;
  
  /// When set, and SLEEP and STANDBY are not set, the chip will cycle
  /// between sleep and taking a single sample at a rate determined by
  /// LP_ACCEL_ODR register.
  /// NOTE: When all accelerometer axis are disabled via PWR_MGMT_2
  /// register bits and cycle is enabled, the chip will wake up at the rate
  /// determined by the respective registers above, but will not take any samples. 
  uint8_t cycle : 1;
  
  /// When set, the gyro drive and pll circuitry are enabled, but the sense paths
  /// are disabled. This is a low power mode that allows quick enabling of the
  /// gyros.
  uint8_t gyro_standby : 1;
  
  /// Power down internal PTAT voltage generator and PTAT ADC
  uint8_t pd_ptat : 1;

  /// Code Clock Source
  /// 0 - Internal 20MHz oscillator
  /// 1 - Auto selects the best available clock source – PLL if ready, else
  /// use the Internal oscillator
  /// 2 - Auto selects the best available clock source – PLL if ready, else
  /// use the Internal oscillator
  /// 3 - Auto selects the best available clock source – PLL if ready, else
  /// use the Internal oscillator
  /// 4 - Auto selects the best available clock source – PLL if ready, else
  /// use the Internal oscillator
  /// 5 - Auto selects the best available clock source – PLL if ready, else
  /// use the Internal oscillator
  /// 6 - Internal 20MHz oscillator
  /// 7 - Stops the clock and keeps timing generator in reset
  /// (After OTP loads, the inverse of PU_SLEEP_MODE bit will be written to
  /// CLKSEL[0])
  uint8_t clksel : 3;
} mpu9250_pwr_mgmt_1;

uint8_t mpu9250_pwr_mgmt_1_pack(const mpu9250_pwr_mgmt_1 *const pwr_mgmt_1);
uint8_t mpu9250_pwr_mgmt_1_unpack(const uint8_t val, mpu9250_pwr_mgmt_1 *const pwr_mgmt_1);
uint8_t mpu9250_pwr_mgmt_1_set(mpu9250 *const device, const mpu9250_pwr_mgmt_1 *const pwr_mgmt_1);
uint8_t mpu9250_pwr_mgmt_1_get(mpu9250 *const device, mpu9250_pwr_mgmt_1 *const pwr_mgmt_1);

// PWR_MGMT_2

typedef struct
{
  uint8_t disable_xa : 1;
  uint8_t disable_ya : 1;
  uint8_t disable_za : 1;
  uint8_t disable_xg : 1;
  uint8_t disable_yg : 1;
  uint8_t disable_zg : 1;
} mpu9250_pwr_mgmt_2;

uint8_t mpu9250_pwr_mgmt_2_pack(const mpu9250_pwr_mgmt_2 *const pwr_mgmt_2);
uint8_t mpu9250_pwr_mgmt_2_unpack(const uint8_t val, mpu9250_pwr_mgmt_2 *const pwr_mgmt_2);
uint8_t mpu9250_pwr_mgmt_2_set(mpu9250 *const device, const mpu9250_pwr_mgmt_2 *const pwr_mgmt_2);
uint8_t mpu9250_pwr_mgmt_2_get(mpu9250 *const device, mpu9250_pwr_mgmt_2 *const pwr_mgmt_2);

// CONFIG

typedef struct
{
  /// When set to ‘1’, when the fifo is full, additional writes will not be written to fifo.
  /// When set to ‘0’, when the fifo is full, additional writes will be written to the fifo,
  /// replacing the oldest data.
  uint8_t fifo_mode : 1;

  /// Enables the FSYNC pin data to be sampled.
  /// EXT_SYNC_SET FSYNC bit location
  /// 0 function disabled
  /// 1 TEMP_OUT_L[0]
  /// 2 GYRO_XOUT_L[0]
  /// 3 GYRO_YOUT_L[0]
  /// 4 GYRO_ZOUT_L[0]
  /// 5 ACCEL_XOUT_L[0]
  /// 6 ACCEL_YOUT_L[0]
  /// 7 ACCEL_ZOUT_L[0]
  /// Fsync will be latched to capture short strobes. This will be done such that if
  /// Fsync toggles, the latched value toggles, but won’t toggle again until the new
  /// latched value is captured by the sample rate strobe. This is a requirement for
  /// working with some 3rd party devices that have fsync strobes shorter than our
  /// sample rate.
  uint8_t ext_sync_set : 3;

  /// For the DLPF to be used, fchoice[1:0] must be set to 2’b11, fchoice_b[1:0] is
  /// 2’b00.
  uint8_t dlpf_cfg : 3;
} mpu9250_config;

uint8_t mpu9250_config_pack(const mpu9250_config *const config);
uint8_t mpu9250_config_unpack(const uint8_t val, mpu9250_config *const config);
uint8_t mpu9250_config_set(mpu9250 *const device, const mpu9250_config *const config);
uint8_t mpu9250_config_get(mpu9250 *const device, mpu9250_config *const config);

// GYRO_CONFIG

typedef struct
{
  /// X Gyro self-test
  uint8_t xgyro_cten : 1;
  
  /// Y Gyro self-test
  uint8_t ygyro_cten : 1;

  /// Z Gyro self-test
  uint8_t zgyro_cten : 1;

  /// Gyro Full Scale Select:
  /// 00 = +250dps
  /// 01= +500 dps
  /// 10 = +1000 dps
  /// 11 = +2000 dps
  uint8_t gyro_fs_sel : 2;

  /// Used to bypass DLPF.
  uint8_t fchoice_b : 1;
} mpu9250_gyro_config;

uint8_t mpu9250_gyro_config_pack(const mpu9250_gyro_config *const gyro_config);
uint8_t mpu9250_gyro_config_unpack(const uint8_t val, mpu9250_gyro_config *const gyro_config);
uint8_t mpu9250_gyro_config_set(mpu9250 *const device, const mpu9250_gyro_config *const gyro_config);
uint8_t mpu9250_gyro_config_get(mpu9250 *const device, mpu9250_gyro_config *const gyro_config);

// ACCEL_CONFIG

typedef struct
{
  /// X Accel self-test
  uint8_t ax_st_en : 1;
  
  /// Y Accel self-test
  uint8_t ay_st_en : 1;
  
  /// Z Accel self-test
  uint8_t az_st_en : 1;
  
  /// Accel Full Scale Select:
  /// 00 = +2g
  /// 01 = +4g
  /// 10 = +8g
  /// 11 = +16g
  uint8_t accel_fs_sel : 2;
} mpu9250_accel_config;

uint8_t mpu9250_accel_config_pack(const mpu9250_accel_config *const accel_config);
uint8_t mpu9250_accel_config_unpack(const uint8_t val, mpu9250_accel_config *const accel_config);
uint8_t mpu9250_accel_config_set(mpu9250 *const device, const mpu9250_accel_config *const accel_config);
uint8_t mpu9250_accel_config_get(mpu9250 *const device, mpu9250_accel_config *const accel_config);

// USER_CTRL

typedef struct
{
  /// 1 – Enable FIFO operation mode.
  /// 0 – Disable FIFO access from serial interface. To disable FIFO writes by
  /// dma, use FIFO_EN register. To disable possible FIFO writes from DMP,
  /// disable the DMP.
  uint8_t fifo_en : 1;
  
  /// 1 – Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated
  /// from pins SDA/SDI and SCL/ SCLK.
  /// 0 – Disable I2C Master I/F module; pins ES_DA and ES_SCL are logically
  /// driven by pins SDA/SDI and SCL/ SCLK.
  /// NOTE: DMP will run when enabled, even if all internal sensors are disabled,
  /// except when the sample rate is set to 8Khz.
  uint8_t i2c_mst_en : 1;
  
  /// 1 – Reset I2C Slave module and put the serial interface in SPI mode only.
  /// This bit auto clears after one clock cycle.
  uint8_t i2c_if_dis : 1;
  
  /// 1 – Reset FIFO module. Reset is asynchronous. This bit auto clears after
  /// one clock cycle
  uint8_t fifo_rst : 1;
  
  /// 1 – Reset I2C Master module. Reset is asynchronous. This bit auto clears
  /// after one clock cycle.
  /// NOTE: This bit should only be set when the I2C master has hung. If this bit
  /// is set during an active I2C master transaction, the I2C slave will hang, which
  /// will require the host to reset the slave.
  uint8_t i2c_mst_rst : 1;
  
  /// 1 – Reset all gyro digital signal path, accel digital signal path, and temp
  /// digital signal path. This bit also clears all the sensor registers.
  /// SIG_COND_RST is a pulse of one clk8M wide
  uint8_t sig_cond_rst : 1;
} mpu9250_user_ctrl;

uint8_t mpu9250_user_ctrl_pack(const mpu9250_user_ctrl *const user_ctrl);
uint8_t mpu9250_user_ctrl_unpack(const uint8_t val, mpu9250_user_ctrl *const user_ctrl);
uint8_t mpu9250_user_ctrl_set(mpu9250 *const device, const mpu9250_user_ctrl *const user_ctrl);
uint8_t mpu9250_user_ctrl_get(mpu9250 *const device, mpu9250_user_ctrl *const user_ctrl);

// I2C_MST_CTRL

typedef struct
{
  uint8_t mult_mst_en : 1;

  uint8_t wait_for_es : 1;

  uint8_t slv_3_fifo_en : 1;

  uint8_t i2c_mst_p_nsr : 1;

  uint8_t i2c_mst_clk : 3;
} mpu9250_i2c_mst_ctrl;

uint8_t mpu9250_i2c_mst_ctrl_pack(const mpu9250_i2c_mst_ctrl *const i2c_mst_ctrl);
uint8_t mpu9250_i2c_mst_ctrl_unpack(const uint8_t val, mpu9250_i2c_mst_ctrl *const i2c_mst_ctrl);
uint8_t mpu9250_i2c_mst_ctrl_set(mpu9250 *const device, const mpu9250_i2c_mst_ctrl *const i2c_mst_ctrl);
uint8_t mpu9250_i2c_mst_ctrl_get(mpu9250 *const device, mpu9250_i2c_mst_ctrl *const i2c_mst_ctrl);

// I2C_*_ADDR

typedef struct
{
  uint8_t i2c_slv_rnw : 1;
  uint8_t i2c_id : 7;
} mpu9250_i2c_addr;

uint8_t mpu9250_i2c_addr_pack(const mpu9250_i2c_addr *const i2c_addr);
uint8_t mpu9250_i2c_addr_unpack(const uint8_t val, mpu9250_i2c_addr *const i2c_addr);

// I2C_SLVN_ADDR

uint8_t mpu9250_i2c_slvn_addr_set(mpu9250 *const device, const uint8_t n, const mpu9250_i2c_addr *const i2c_addr);
uint8_t mpu9250_i2c_slvn_addr_get(mpu9250 *const device, const uint8_t n, mpu9250_i2c_addr *const i2c_addr);

// I2C_SLVN_REG

uint8_t mpu9250_i2c_slvn_reg_set(mpu9250 *const device, const uint8_t n, const uint8_t i2c_reg);
uint8_t mpu9250_i2c_slvn_reg_get(mpu9250 *const device, const uint8_t n);

// I2C_SLVN_DO

uint8_t mpu9250_i2c_slvn_do_set(mpu9250 *const device, const uint8_t n, const uint8_t i2c_do);
uint8_t mpu9250_i2c_slvn_do_get(mpu9250 *const device, const uint8_t n);

// I2C_CTRL

typedef struct
{
  uint8_t i2c_slv_en : 1;
  uint8_t slv_done_int_en : 1;
  uint8_t i2c_slv_reg_dis : 1;
  uint8_t i2c_mst_dly : 4;
} mpu9250_i2c_ctrl;

uint8_t mpu9250_i2c_ctrl_pack(const mpu9250_i2c_ctrl *const i2c_ctrl);
uint8_t mpu9250_i2c_ctrl_unpack(const uint8_t val, mpu9250_i2c_ctrl *const i2c_ctrl);

// I2C_SLVN_CTRL

uint8_t mpu9250_i2c_slvn_ctrl_set(mpu9250 *const device, const uint8_t n, const mpu9250_i2c_ctrl *const i2c_ctrl);
uint8_t mpu9250_i2c_slvn_ctrl_get(mpu9250 *const device, const uint8_t n, mpu9250_i2c_ctrl *const i2c_ctrl);

// Sample

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} mpu9250_sample;

// Accelerometer

uint8_t mpu9250_accel_sample_read(mpu9250 *const device, mpu9250_sample *const accel_sample);

#ifdef MPU9250_WALLABY
void mpu9250_accel_sample_write_regs(mpu9250 *const device, const mpu9250_sample *const accel_sample, uint8_t *const regs);
#endif

// Gyro

uint8_t mpu9250_gyro_sample_read(mpu9250 *const device, mpu9250_sample *const gyro_sample);

#ifdef MPU9250_WALLABY
void mpu9250_gyro_sample_write_regs(mpu9250 *const device, const mpu9250_sample *const gyro_sample, uint8_t *const regs);
#endif

// Magnetometer

uint8_t mpu9250_magneto_sample_read(mpu9250 *const device, mpu9250_sample *const magneto_sample);

#ifdef MPU9250_WALLABY
void mpu9250_magneto_sample_write_regs(mpu9250 *const device, const mpu9250_sample *const magneto_sample, uint8_t *const regs);
#endif

#endif