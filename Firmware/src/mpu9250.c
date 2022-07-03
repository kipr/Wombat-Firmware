/**
 * @file mpu9250.c
 * @author Braden McDorman (bmcdorman@gmail.com)
 * @brief An implementation of SPI communication with the MPU9250.
 */

#include "mpu9250.h"
#include "wallaby.h"

#include <stdlib.h>

const uint8_t MPU9250_SELF_TEST_X_GYRO = 0x00;
const uint8_t MPU9250_SELF_TEST_Y_GYRO = 0x01;
const uint8_t MPU9250_SELF_TEST_Z_GYRO = 0x02;
const uint8_t MPU9250_SELF_TEST_X_ACCEL = 0x0D;
const uint8_t MPU9250_SELF_TEST_Y_ACCEL = 0x0E;
const uint8_t MPU9250_SELF_TEST_Z_ACCEL = 0x0F;
const uint8_t MPU9250_XG_OFFSET_H = 0x13;
const uint8_t MPU9250_XG_OFFSET_L = 0x14;
const uint8_t MPU9250_YG_OFFSET_H = 0x15;
const uint8_t MPU9250_YG_OFFSET_L = 0x16;
const uint8_t MPU9250_ZG_OFFSET_H = 0x17;
const uint8_t MPU9250_ZG_OFFSET_L = 0x18;
const uint8_t MPU9250_SMPLRT_DIV = 0x19;
const uint8_t MPU9250_CONFIG = 0x1A;
const uint8_t MPU9250_GYRO_CONFIG = 0x1B;
const uint8_t MPU9250_ACCEL_CONFIG = 0x1C;
const uint8_t MPU9250_ACCEL_CONFIG_2 = 0x1D;
const uint8_t MPU9250_LP_ACCEL_ODR = 0x1E;
const uint8_t MPU9250_WOM_THR = 0x1F;
const uint8_t MPU9250_FIFO_EN = 0x23;
const uint8_t MPU9250_I2C_MST_CTRL = 0x24;
const uint8_t MPU9250_I2C_SLV0_ADDR = 0x25;
const uint8_t MPU9250_I2C_SLV0_REG = 0x26;
const uint8_t MPU9250_I2C_SLV0_CTRL = 0x27;
const uint8_t MPU9250_I2C_SLV1_ADDR = 0x28;
const uint8_t MPU9250_I2C_SLV1_REG = 0x29;
const uint8_t MPU9250_I2C_SLV1_CTRL = 0x2A;
const uint8_t MPU9250_I2C_SLV2_ADDR = 0x2B;
const uint8_t MPU9250_I2C_SLV2_REG = 0x2C;
const uint8_t MPU9250_I2C_SLV2_CTRL = 0x2D;
const uint8_t MPU9250_I2C_SLV3_ADDR = 0x2E;
const uint8_t MPU9250_I2C_SLV3_REG = 0x2F;
const uint8_t MPU9250_I2C_SLV3_CTRL = 0x30;
const uint8_t MPU9250_I2C_SLV4_ADDR = 0x31;
const uint8_t MPU9250_I2C_SLV4_REG = 0x32;
const uint8_t MPU9250_I2C_SLV4_DO = 0x33;
const uint8_t MPU9250_I2C_SLV4_CTRL = 0x34;
const uint8_t MPU9250_I2C_SLV4_DI = 0x35;
const uint8_t MPU9250_I2C_MST_STATUS = 0x36;
const uint8_t MPU9250_INT_PIN_CFG = 0x37;
const uint8_t MPU9250_INT_ENABLE = 0x38;
const uint8_t MPU9250_INT_STATUS = 0x3A;
const uint8_t MPU9250_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU9250_ACCEL_XOUT_L = 0x3C;
const uint8_t MPU9250_ACCEL_YOUT_H = 0x3D;
const uint8_t MPU9250_ACCEL_YOUT_L = 0x3E;
const uint8_t MPU9250_ACCEL_ZOUT_H = 0x3F;
const uint8_t MPU9250_ACCEL_ZOUT_L = 0x40;
const uint8_t MPU9250_TEMP_OUT_H = 0x41;
const uint8_t MPU9250_TEMP_OUT_L = 0x42;
const uint8_t MPU9250_GYRO_XOUT_H = 0x43;
const uint8_t MPU9250_GYRO_XOUT_L = 0x44;
const uint8_t MPU9250_GYRO_YOUT_H = 0x45;
const uint8_t MPU9250_GYRO_YOUT_L = 0x46;
const uint8_t MPU9250_GYRO_ZOUT_H = 0x47;
const uint8_t MPU9250_GYRO_ZOUT_L = 0x48;
const uint8_t MPU9250_EXT_SENS_DATA_00 = 0x49;
const uint8_t MPU9250_EXT_SENS_DATA_01 = 0x4A;
const uint8_t MPU9250_EXT_SENS_DATA_02 = 0x4B;
const uint8_t MPU9250_EXT_SENS_DATA_03 = 0x4C;
const uint8_t MPU9250_EXT_SENS_DATA_04 = 0x4D;
const uint8_t MPU9250_EXT_SENS_DATA_05 = 0x4E;
const uint8_t MPU9250_EXT_SENS_DATA_06 = 0x4F;
const uint8_t MPU9250_EXT_SENS_DATA_07 = 0x50;
const uint8_t MPU9250_EXT_SENS_DATA_08 = 0x51;
const uint8_t MPU9250_EXT_SENS_DATA_09 = 0x52;
const uint8_t MPU9250_EXT_SENS_DATA_10 = 0x53;
const uint8_t MPU9250_EXT_SENS_DATA_11 = 0x54;
const uint8_t MPU9250_EXT_SENS_DATA_12 = 0x55;
const uint8_t MPU9250_EXT_SENS_DATA_13 = 0x56;
const uint8_t MPU9250_EXT_SENS_DATA_14 = 0x57;
const uint8_t MPU9250_EXT_SENS_DATA_15 = 0x58;
const uint8_t MPU9250_EXT_SENS_DATA_16 = 0x59;
const uint8_t MPU9250_EXT_SENS_DATA_17 = 0x5A;
const uint8_t MPU9250_EXT_SENS_DATA_18 = 0x5B;
const uint8_t MPU9250_EXT_SENS_DATA_19 = 0x5C;
const uint8_t MPU9250_EXT_SENS_DATA_20 = 0x5D;
const uint8_t MPU9250_EXT_SENS_DATA_21 = 0x5E;
const uint8_t MPU9250_EXT_SENS_DATA_22 = 0x5F;
const uint8_t MPU9250_EXT_SENS_DATA_23 = 0x60;
const uint8_t MPU9250_I2C_SLV0_D0 = 0x63;
const uint8_t MPU9250_I2C_SLV1_D0 = 0x64;
const uint8_t MPU9250_I2C_SLV2_D0 = 0x65;
const uint8_t MPU9250_I2C_SLV3_D0 = 0x66;
const uint8_t MPU9250_I2C_MST_DELAY_CTRL = 0x67;
const uint8_t MPU9250_SIGNAL_PATH_RESET = 0x68;
const uint8_t MPU9250_MOT_DETECT_CTRL = 0x69;
const uint8_t MPU9250_USER_CTRL = 0x6A;
const uint8_t MPU9250_PWR_MGMT_1 = 0x6B;
const uint8_t MPU9250_PWR_MGMT_2 = 0x6C;
const uint8_t MPU9250_FIFO_COUNTH = 0x72;
const uint8_t MPU9250_FIFO_COUNTL = 0x73;
const uint8_t MPU9250_FIFO_R_W = 0x74;
const uint8_t MPU9250_WHO_AM_I = 0x75;
const uint8_t MPU9250_XA_OFFSET_H = 0x77;
const uint8_t MPU9250_XA_OFFSET_L = 0x78;
const uint8_t MPU9250_YA_OFFSET_H = 0x7A;
const uint8_t MPU9250_YA_OFFSET_L = 0x7B;
const uint8_t MPU9250_ZA_OFFSET_H = 0x7D;
const uint8_t MPU9250_ZA_OFFSET_L = 0x7E;

uint8_t mpu9250_check_who_am_i(mpu9250 *const device)
{
  return mpu9250_read(device, MPU9250_WHO_AM_I) == 0x71;
}

mpu9250 *mpu9250_create(SPI_TypeDef *spi, GPIO_TypeDef *cs0, uint16_t cs0_pin)
{
  mpu9250 *device = malloc(sizeof(mpu9250));
  if (!device) return 0;

  // Configure device
  device->spi = spi;
  device->cs0 = cs0;
  device->cs0_pin = cs0_pin;

  // Check WHO_AM_I
  if (!mpu9250_check_who_am_i(device))
  {
    free(device);
    return 0;
  }

  return device;
}

void mpu9250_destroy(mpu9250 *const device)
{
  free(device);
}

uint8_t mpu9250_write(mpu9250 *const device, uint8_t address, uint8_t val)
{
  device->cs0->BSRRH = device->cs0_pin;
  SPI_write(device->spi, address);
  const uint8_t ret = SPI_write(device->spi, val);
  device->cs0->BSRRL = device->cs0_pin;
  return ret;
}

uint8_t mpu9250_read(mpu9250 *const device, uint8_t address)
{
  device->cs0->BSRRH = device->cs0_pin;
  SPI_write(device->spi, address | 0x80);
  const uint8_t ret = SPI_write(device->spi, 0x00);
  device->cs0->BSRRL = device->cs0_pin;
  return ret;
}

uint8_t mpu9250_read_buffer(
  mpu9250 *const device,
  const uint8_t address,
  uint8_t *const buffer,
  const uint8_t length
)
{
  device->cs0->BSRRH = device->cs0_pin;
  SPI_write(device->spi, address | 0x80);
  
  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = SPI_write(device->spi, 0x00);
  }
  device->cs0->BSRRL = device->cs0_pin;
  
  return 1;
}

// PWR_MGMT_1

uint8_t mpu9250_pwr_mgmt_1_pack(const mpu9250_pwr_mgmt_1 *const pwr_mgmt_1)
{
  return (
    pwr_mgmt_1->h_reset << 7 |
    pwr_mgmt_1->sleep << 6 |
    pwr_mgmt_1->cycle << 5 |
    pwr_mgmt_1->gyro_standby << 4 |
    pwr_mgmt_1->pd_ptat << 3 |
    pwr_mgmt_1->clksel << 0
  );
}

uint8_t mpu9250_pwr_mgmt_1_unpack(const uint8_t val, mpu9250_pwr_mgmt_1 *const pwr_mgmt_1)
{
  pwr_mgmt_1->h_reset = (val >> 7) & 0x01;
  pwr_mgmt_1->sleep = (val >> 6) & 0x01;
  pwr_mgmt_1->cycle = (val >> 5) & 0x01;
  pwr_mgmt_1->gyro_standby = (val >> 4) & 0x01;
  pwr_mgmt_1->pd_ptat = (val >> 3) & 0x01;
  pwr_mgmt_1->clksel = (val >> 0) & 0x07;
  return val;
}

uint8_t mpu9250_pwr_mgmt_1_set(mpu9250 *const device, const mpu9250_pwr_mgmt_1 *const pwr_mgmt_1)
{
  return mpu9250_write(device, MPU9250_PWR_MGMT_1, mpu9250_pwr_mgmt_1_pack(pwr_mgmt_1));
}

uint8_t mpu9250_pwr_mgmt_1_get(mpu9250 *const device, mpu9250_pwr_mgmt_1 *const pwr_mgmt_1)
{
  return mpu9250_pwr_mgmt_1_unpack(mpu9250_read(device, MPU9250_PWR_MGMT_1), pwr_mgmt_1);
}

// PWR_MGMT_2

uint8_t mpu9250_pwr_mgmt_2_pack(const mpu9250_pwr_mgmt_2 *const pwr_mgmt_2)
{
  return (
    pwr_mgmt_2->disable_xa << 5 |
    pwr_mgmt_2->disable_ya << 4 |
    pwr_mgmt_2->disable_za << 3 |
    pwr_mgmt_2->disable_xg << 2 |
    pwr_mgmt_2->disable_yg << 1 |
    pwr_mgmt_2->disable_zg << 0
  );
}

uint8_t mpu9250_pwr_mgmt_2_unpack(const uint8_t val, mpu9250_pwr_mgmt_2 *const pwr_mgmt_2)
{
  pwr_mgmt_2->disable_xa = (val >> 5) & 0x01;
  pwr_mgmt_2->disable_ya = (val >> 4) & 0x01;
  pwr_mgmt_2->disable_za = (val >> 3) & 0x01;
  pwr_mgmt_2->disable_xg = (val >> 2) & 0x01;
  pwr_mgmt_2->disable_yg = (val >> 1) & 0x01;
  pwr_mgmt_2->disable_zg = (val >> 0) & 0x01;
  return val;
}

uint8_t mpu9250_pwr_mgmt_2_set(mpu9250 *const device, const mpu9250_pwr_mgmt_2 *const pwr_mgmt_2)
{
  return mpu9250_write(device, MPU9250_PWR_MGMT_2, mpu9250_pwr_mgmt_2_pack(pwr_mgmt_2));
}

uint8_t mpu9250_pwr_mgmt_2_get(mpu9250 *const device, mpu9250_pwr_mgmt_2 *const pwr_mgmt_2)
{
  return mpu9250_pwr_mgmt_2_unpack(mpu9250_read(device, MPU9250_PWR_MGMT_2), pwr_mgmt_2);
}

// CONFIG

const uint8_t MPU9250_CONFIG_DLPF_CFG_250_HZ = 0;
const uint8_t MPU9250_CONFIG_DLPF_CFG_184_HZ = 1;
const uint8_t MPU9250_CONFIG_DLPF_CFG_92_HZ = 2;
const uint8_t MPU9250_CONFIG_DLPF_CFG_41_HZ = 3;
const uint8_t MPU9250_CONFIG_DLPF_CFG_20_HZ = 4;
const uint8_t MPU9250_CONFIG_DLPF_CFG_10_HZ = 5;
const uint8_t MPU9250_CONFIG_DLPF_CFG_5_HZ = 6;
const uint8_t MPU9250_CONFIG_DLPF_CFG_3600_HZ = 7;

uint8_t mpu9250_config_pack(const mpu9250_config *const config)
{
  return (
    config->fifo_mode << 6 |
    config->ext_sync_set << 3 |
    config->dlpf_cfg << 0
  );
}

uint8_t mpu9250_config_unpack(const uint8_t val, mpu9250_config *const config)
{
  config->fifo_mode = (val >> 6) & 0x01;
  config->ext_sync_set = (val >> 3) & 0x07;
  config->dlpf_cfg = (val >> 0) & 0x07;
  return val;
}

uint8_t mpu9250_config_set(mpu9250 *const device, const mpu9250_config *const config)
{
  return mpu9250_write(device, MPU9250_CONFIG, mpu9250_config_pack(config));
}

uint8_t mpu9250_config_get(mpu9250 *const device, mpu9250_config *const config)
{
  return mpu9250_config_unpack(mpu9250_read(device, MPU9250_CONFIG), config);
}

// GYRO_CONFIG

const uint8_t MPU9250_GYRO_CONFIG_GYRO_FS_SEL_250_DPS = 0;
const uint8_t MPU9250_GYRO_CONFIG_GYRO_FS_SEL_500_DPS = 1;
const uint8_t MPU9250_GYRO_CONFIG_GYRO_FS_SEL_1000_DPS = 2;
const uint8_t MPU9250_GYRO_CONFIG_GYRO_FS_SEL_2000_DPS = 3;

uint8_t mpu9250_gyro_config_pack(const mpu9250_gyro_config *const gyro_config)
{
  return (
    gyro_config->xgyro_cten << 7 |
    gyro_config->ygyro_cten << 6 |
    gyro_config->zgyro_cten << 5 |
    gyro_config->gyro_fs_sel << 3 |
    gyro_config->fchoice_b << 0
  );
}

uint8_t mpu9250_gyro_config_unpack(const uint8_t val, mpu9250_gyro_config *const gyro_config)
{
  gyro_config->xgyro_cten = (val >> 7) & 0x01;
  gyro_config->ygyro_cten = (val >> 6) & 0x01;
  gyro_config->zgyro_cten = (val >> 5) & 0x01;
  gyro_config->gyro_fs_sel = (val >> 3) & 0x03;
  gyro_config->fchoice_b = (val >> 0) & 0x01;
  return val;
}

uint8_t mpu9250_gyro_config_set(mpu9250 *const device, const mpu9250_gyro_config *const gyro_config)
{
  return mpu9250_write(device, MPU9250_GYRO_CONFIG, mpu9250_gyro_config_pack(gyro_config));
}

uint8_t mpu9250_gyro_config_get(mpu9250 *const device, mpu9250_gyro_config *const gyro_config)
{
  return mpu9250_gyro_config_unpack(mpu9250_read(device, MPU9250_GYRO_CONFIG), gyro_config);
}

// ACCEL_CONFIG

const uint8_t MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_2_G = 0;
const uint8_t MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_4_G = 1;
const uint8_t MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_8_G = 2;
const uint8_t MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_16_G = 3;

uint8_t mpu9250_accel_config_pack(const mpu9250_accel_config *const accel_config)
{
  return (
    accel_config->ax_st_en << 7 |
    accel_config->ay_st_en << 6 |
    accel_config->az_st_en << 5 |
    accel_config->accel_fs_sel << 3
  );
}

uint8_t mpu9250_accel_config_unpack(const uint8_t val, mpu9250_accel_config *const accel_config)
{
  accel_config->ax_st_en = (val >> 7) & 0x01;
  accel_config->ay_st_en = (val >> 6) & 0x01;
  accel_config->az_st_en = (val >> 5) & 0x01;
  accel_config->accel_fs_sel = (val >> 3) & 0x03;
  return val;
}

uint8_t mpu9250_accel_config_set(mpu9250 *const device, const mpu9250_accel_config *const accel_config)
{
  return mpu9250_write(device, MPU9250_ACCEL_CONFIG, mpu9250_accel_config_pack(accel_config));
}

uint8_t mpu9250_accel_config_get(mpu9250 *const device, mpu9250_accel_config *const accel_config)
{
  return mpu9250_accel_config_unpack(mpu9250_read(device, MPU9250_ACCEL_CONFIG), accel_config);
}

// USER_CTRL

uint8_t mpu9250_user_ctrl_pack(const mpu9250_user_ctrl *const user_ctrl)
{
  return (
    user_ctrl->fifo_en << 6 |
    user_ctrl->i2c_mst_en << 5 |
    user_ctrl->i2c_if_dis << 4 |
    user_ctrl->fifo_rst << 2 |
    user_ctrl->i2c_mst_rst << 1 |
    user_ctrl->sig_cond_rst << 0
  );
}

uint8_t mpu9250_user_ctrl_unpack(const uint8_t val, mpu9250_user_ctrl *const user_ctrl)
{
  user_ctrl->fifo_en = (val >> 6) & 0x01;
  user_ctrl->i2c_mst_en = (val >> 5) & 0x01;
  user_ctrl->i2c_if_dis = (val >> 4) & 0x01;
  user_ctrl->fifo_rst = (val >> 2) & 0x01;
  user_ctrl->i2c_mst_rst = (val >> 1) & 0x01;
  user_ctrl->sig_cond_rst = (val >> 0) & 0x01;
  return val;
}

uint8_t mpu9250_user_ctrl_set(mpu9250 *const device, const mpu9250_user_ctrl *const user_ctrl)
{
  return mpu9250_write(device, MPU9250_USER_CTRL, mpu9250_user_ctrl_pack(user_ctrl));
}

uint8_t mpu9250_user_ctrl_get(mpu9250 *const device, mpu9250_user_ctrl *const user_ctrl)
{
  return mpu9250_user_ctrl_unpack(mpu9250_read(device, MPU9250_USER_CTRL), user_ctrl);
}

// I2C_MST_CTRL

uint8_t mpu9250_i2c_mst_ctrl_pack(const mpu9250_i2c_mst_ctrl *const i2c_mst_ctrl)
{
  return (
    i2c_mst_ctrl->mult_mst_en << 7 |
    i2c_mst_ctrl->wait_for_es << 6 |
    i2c_mst_ctrl->slv_3_fifo_en << 5 |
    i2c_mst_ctrl->i2c_mst_p_nsr << 4 |
    i2c_mst_ctrl->i2c_mst_clk << 0
  );
}

uint8_t mpu9250_i2c_mst_ctrl_unpack(const uint8_t val, mpu9250_i2c_mst_ctrl *const i2c_mst_ctrl)
{
  i2c_mst_ctrl->mult_mst_en = (val >> 7) & 0x01;
  i2c_mst_ctrl->wait_for_es = (val >> 6) & 0x01;
  i2c_mst_ctrl->slv_3_fifo_en = (val >> 5) & 0x01;
  i2c_mst_ctrl->i2c_mst_p_nsr = (val >> 4) & 0x01;
  i2c_mst_ctrl->i2c_mst_clk = (val >> 0) & 0x0F;
  return val;
}

uint8_t mpu9250_i2c_mst_ctrl_set(mpu9250 *const device, const mpu9250_i2c_mst_ctrl *const i2c_mst_ctrl)
{
  return mpu9250_write(device, MPU9250_I2C_MST_CTRL, mpu9250_i2c_mst_ctrl_pack(i2c_mst_ctrl));
}

uint8_t mpu9250_i2c_mst_ctrl_get(mpu9250 *const device, mpu9250_i2c_mst_ctrl *const i2c_mst_ctrl)
{
  return mpu9250_i2c_mst_ctrl_unpack(mpu9250_read(device, MPU9250_I2C_MST_CTRL), i2c_mst_ctrl);
}

// I2C_*_ADDR

uint8_t mpu9250_i2c_addr_pack(const mpu9250_i2c_addr *const i2c_slv0_addr)
{
  return (
    i2c_slv0_addr->i2c_slv_rnw << 7 |
    i2c_slv0_addr->i2c_id << 0
  );
}

uint8_t mpu9250_i2c_addr_unpack(const uint8_t val, mpu9250_i2c_addr *const i2c_slv0_addr)
{
  i2c_slv0_addr->i2c_slv_rnw = (val >> 7) & 0x01;
  i2c_slv0_addr->i2c_id = (val >> 0) & 0x7F;
  return val;
}

// I2C_SLVN_ADDR

static const uint8_t I2C_SLVN_ADDR[4] =
{
  MPU9250_I2C_SLV0_ADDR,
  MPU9250_I2C_SLV1_ADDR,
  MPU9250_I2C_SLV2_ADDR,
  MPU9250_I2C_SLV3_ADDR
};

uint8_t mpu9250_i2c_slvn_addr_set(mpu9250 *const device, const uint8_t n, const mpu9250_i2c_addr *const i2c_addr)
{
  return mpu9250_write(device, I2C_SLVN_ADDR[n], mpu9250_i2c_addr_pack(i2c_addr));
}

uint8_t mpu9250_i2c_slvn_addr_get(mpu9250 *const device, const uint8_t n, mpu9250_i2c_addr *const i2c_addr)
{
  return mpu9250_i2c_addr_unpack(mpu9250_read(device, I2C_SLVN_ADDR[n]), i2c_addr);
}

// I2C_SLVN_REG

static const uint8_t I2C_SLVN_REG[4] =
{
  MPU9250_I2C_SLV0_REG,
  MPU9250_I2C_SLV1_REG,
  MPU9250_I2C_SLV2_REG,
  MPU9250_I2C_SLV3_REG
};

uint8_t mpu9250_i2c_slvn_reg_set(mpu9250 *const device, const uint8_t n, const uint8_t i2c_reg)
{
  return mpu9250_write(device, I2C_SLVN_REG[n], i2c_reg);
}

uint8_t mpu9250_i2c_slvn_reg_get(mpu9250 *const device, const uint8_t n)
{
  return mpu9250_read(device, I2C_SLVN_REG[n]);
}

// I2C_SLVN_DO

static const uint8_t I2C_SLVN_DO[4] =
{
  MPU9250_I2C_SLV0_REG,
  MPU9250_I2C_SLV1_REG,
  MPU9250_I2C_SLV2_REG,
  MPU9250_I2C_SLV3_REG
};

uint8_t mpu9250_i2c_slvn_do_set(mpu9250 *const device, const uint8_t n, const uint8_t i2c_do)
{
  return mpu9250_write(device, I2C_SLVN_DO[n], i2c_do);
}

uint8_t mpu9250_i2c_slvn_do_get(mpu9250 *const device, const uint8_t n)
{
  return mpu9250_read(device, I2C_SLVN_DO[n]);
}

uint8_t mpu9250_i2c_ctrl_pack(const mpu9250_i2c_ctrl *const i2c_ctrl)
{
  return (
    i2c_ctrl->i2c_slv_en << 7 |
    i2c_ctrl->slv_done_int_en << 6 |
    i2c_ctrl->i2c_slv_reg_dis << 5 |
    i2c_ctrl->i2c_mst_dly << 0
  );
}

uint8_t mpu9250_i2c_ctrl_unpack(const uint8_t val, mpu9250_i2c_ctrl *const i2c_ctrl)
{
  i2c_ctrl->i2c_slv_en = (val >> 7) & 0x01;
  i2c_ctrl->slv_done_int_en = (val >> 6) & 0x01;
  i2c_ctrl->i2c_slv_reg_dis = (val >> 5) & 0x01;
  i2c_ctrl->i2c_mst_dly = (val >> 0) & 0x0F;
  return val;
}

// I2C_SLVN_CTRL

static const uint8_t I2C_SLVN_CTRL[4] =
{
  MPU9250_I2C_SLV0_CTRL,
  MPU9250_I2C_SLV1_CTRL,
  MPU9250_I2C_SLV2_CTRL,
  MPU9250_I2C_SLV3_CTRL
};

uint8_t mpu9250_i2c_slvn_ctrl_set(mpu9250 *const device, const uint8_t n, const mpu9250_i2c_ctrl *const i2c_ctrl)
{
  return mpu9250_write(device, I2C_SLVN_CTRL[n], mpu9250_i2c_ctrl_pack(i2c_ctrl));
}

uint8_t mpu9250_i2c_slvn_ctrl_get(mpu9250 *const device, const uint8_t n, mpu9250_i2c_ctrl *const i2c_ctrl)
{
  return mpu9250_i2c_ctrl_unpack(mpu9250_read(device, I2C_SLVN_CTRL[n]), i2c_ctrl);
}

// SMPLRT_DIV

const uint8_t MPU9250_SMPLRT_DIV_1000_HZ = 0;
const uint8_t MPU9250_SMPLRT_DIV_500_HZ = 1;
const uint8_t MPU9250_SMPLRT_DIV_250_HZ = 2;
const uint8_t MPU9250_SMPLRT_DIV_200_HZ = 3;
const uint8_t MPU9250_SMPLRT_DIV_167_HZ = 4;
const uint8_t MPU9250_SMPLRT_DIV_143_HZ = 5;
const uint8_t MPU9250_SMPLRT_DIV_125_HZ = 6;

uint8_t mpu9250_smplrt_div_set(mpu9250 *const device, const uint8_t smplrt_div)
{
  return mpu9250_write(device, MPU9250_SMPLRT_DIV, smplrt_div);
}

uint8_t mpu9250_smplrt_div_get(mpu9250 *const device)
{
  return mpu9250_read(device, MPU9250_SMPLRT_DIV);
}

// sample

void mpu9250_sample_low_pass_filter(
  mpu9250_sample *const acc,
  const mpu9250_sample *const sample,
  const float alpha
)
{
  acc->x = alpha * acc->x + (1.0f - alpha) * sample->x;
  acc->y = alpha * acc->y + (1.0f - alpha) * sample->y;
  acc->z = alpha * acc->z + (1.0f - alpha) * sample->z;
}

// Accelerometer

uint8_t mpu9250_accel_sample_read(mpu9250 *const device, mpu9250_sample *const accel_sample)
{
  uint8_t buffer[8];
  if (!mpu9250_read_buffer(device, MPU9250_ACCEL_XOUT_H, buffer, sizeof (buffer))) return 0;

  accel_sample->x = (int16_t)((buffer[0] << 8) | buffer[1]);
  accel_sample->y = (int16_t)((buffer[2] << 8) | buffer[3]);
  accel_sample->z = (int16_t)((buffer[4] << 8) | buffer[5]);
  
  return 1;
}

#ifdef MPU9250_WALLABY

void mpu9250_accel_sample_write_regs(
  mpu9250 *const device,
  const mpu9250_sample *const accel_sample,
  volatile uint8_t *const regs
)
{
  regs[REG_RW_ACCEL_X_H] = (accel_sample->x & 0xFF00) >> 8;
  regs[REG_RW_ACCEL_X_L] = (accel_sample->x & 0x00FF);
  regs[REG_RW_ACCEL_Y_H] = (accel_sample->y & 0xFF00) >> 8;
  regs[REG_RW_ACCEL_Y_L] = (accel_sample->y & 0x00FF);
  regs[REG_RW_ACCEL_Z_H] = (accel_sample->z & 0xFF00) >> 8;
  regs[REG_RW_ACCEL_Z_L] = (accel_sample->z & 0x00FF);
}

#endif

// Gyro

uint8_t mpu9250_gyro_sample_read(mpu9250 *const device, mpu9250_sample *const gyro_sample)
{
  uint8_t buffer[8];
  if (!mpu9250_read_buffer(device, MPU9250_ACCEL_XOUT_H, buffer, sizeof (buffer))) return 0;

  gyro_sample->x = (int16_t)((buffer[0] << 8) | buffer[1]);
  gyro_sample->y = (int16_t)((buffer[2] << 8) | buffer[3]);
  gyro_sample->z = (int16_t)((buffer[4] << 8) | buffer[5]);
  
  return 1;
}

#ifdef MPU9250_WALLABY

void mpu9250_gyro_sample_write_regs(
  mpu9250 *const device,
  const mpu9250_sample *const gyro_sample,
  volatile uint8_t *const regs
)
{
  regs[REG_RW_GYRO_X_H] = (gyro_sample->x & 0xFF00) >> 8;
  regs[REG_RW_GYRO_X_L] = (gyro_sample->x & 0x00FF);
  regs[REG_RW_GYRO_Y_H] = (gyro_sample->y & 0xFF00) >> 8;
  regs[REG_RW_GYRO_Y_L] = (gyro_sample->y & 0x00FF);
  regs[REG_RW_GYRO_Z_H] = (gyro_sample->z & 0xFF00) >> 8;
  regs[REG_RW_GYRO_Z_L] = (gyro_sample->z & 0x00FF);
}

#endif

// Magnetometer

uint8_t mpu9250_magneto_sample_read(mpu9250 *const device, mpu9250_sample *const magneto_sample)
{
  mpu9250_i2c_slvn_addr_set(device, 0, &(mpu9250_i2c_addr) {
    .i2c_slv_rnw = 1,
    .i2c_id = 0x0C,
  });
  mpu9250_i2c_slvn_reg_set(device, 0, 0x03);
  mpu9250_i2c_slvn_ctrl_set(device, 0, &(mpu9250_i2c_ctrl) {
    .i2c_slv_en = 1,
    .slv_done_int_en = 0,
    .i2c_slv_reg_dis = 0,
    .i2c_mst_dly = 6,
  });

  // a sleep is needed here while the control die inside the mpu9250 gets data from the accelerometer
  delay_us(1000);


  uint8_t buffer[8];
  if (!mpu9250_read_buffer(device, MPU9250_EXT_SENS_DATA_00, buffer, sizeof (buffer))) return 0;

  magneto_sample->x = (int16_t)((buffer[0] << 8) | buffer[1]);
  magneto_sample->y = (int16_t)((buffer[2] << 8) | buffer[3]);
  magneto_sample->z = (int16_t)((buffer[4] << 8) | buffer[5]);
  
  return 1;
}

#ifdef MPU9250_WALLABY

void mpu9250_magneto_sample_write_regs(
  mpu9250 *const device,
  const mpu9250_sample *const magneto_sample,
  volatile uint8_t *const regs
)
{
  regs[REG_RW_MAG_X_H] = (magneto_sample->x & 0xFF00) >> 8;
  regs[REG_RW_MAG_X_L] = (magneto_sample->x & 0x00FF);
  regs[REG_RW_MAG_Y_H] = (magneto_sample->y & 0xFF00) >> 8;
  regs[REG_RW_MAG_Y_L] = (magneto_sample->y & 0x00FF);
  regs[REG_RW_MAG_Z_H] = (magneto_sample->z & 0xFF00) >> 8;
  regs[REG_RW_MAG_Z_L] = (magneto_sample->z & 0x00FF);
}

#endif

// FIFO_EN

// FIFO_EN

uint8_t mpu9250_fifo_en_pack(const mpu9250_fifo_en *const fifo_en)
{
  return (
      (fifo_en->temp_fifo_en << 7) |
      (fifo_en->gyro_xout << 6) |
      (fifo_en->gyro_yout << 5) |
      (fifo_en->gyro_zout << 4) |
      (fifo_en->accel << 3) |
      (fifo_en->slv2 << 2) |
      (fifo_en->slv1 << 1) |
      (fifo_en->slv0 << 0)
  );
}

uint8_t mpu9250_fifo_en_unpack(const uint8_t val, mpu9250_fifo_en *const fifo_en)
{
  fifo_en->temp_fifo_en = (val >> 7) & 1;
  fifo_en->gyro_xout = (val >> 6) & 1;
  fifo_en->gyro_yout = (val >> 5) & 1;
  fifo_en->gyro_zout = (val >> 4) & 1;
  fifo_en->accel = (val >> 3) & 1;
  fifo_en->slv2 = (val >> 2) & 1;
  fifo_en->slv1 = (val >> 1) & 1;
  fifo_en->slv0 = (val >> 0) & 1;
  return 1;
}

uint8_t mpu9250_fifo_en_set(mpu9250 *const device, const mpu9250_fifo_en *const fifo_en)
{
  return mpu9250_write(device, MPU9250_FIFO_EN, mpu9250_fifo_en_pack(fifo_en));
}

uint8_t mpu9250_fifo_en_get(mpu9250 *const device, mpu9250_fifo_en *const fifo_en)
{
  return mpu9250_fifo_en_unpack(mpu9250_read(device, MPU9250_FIFO_EN), fifo_en);
}

uint8_t mpu9250_fifo_en_packet_size(const mpu9250_fifo_en *const fifo_en)
{
  uint8_t expected_bytes = 0;
  if (fifo_en->accel) expected_bytes += 6;
  if (fifo_en->temp_fifo_en) expected_bytes += 2;
  if (fifo_en->gyro_xout) expected_bytes += 2;
  if (fifo_en->gyro_yout) expected_bytes += 2;
  if (fifo_en->gyro_zout) expected_bytes += 2;
  if (fifo_en->slv0) expected_bytes += 6;
  if (fifo_en->slv1) expected_bytes += 6;
  if (fifo_en->slv2) expected_bytes += 6;
  return expected_bytes;
}

// FIFO_COUNT

uint16_t mpu9250_fifo_count_get(mpu9250 *const device)
{
  return (
    (mpu9250_read(device, MPU9250_FIFO_COUNTH) << 8) |
    (mpu9250_read(device, MPU9250_FIFO_COUNTL) << 0)
  );
}

// FIFO_R_W

uint8_t mpu9250_fifo_r_w_set(mpu9250 *const device, const uint8_t val)
{
  return mpu9250_write(device, MPU9250_FIFO_R_W, val);
}

uint8_t mpu9250_fifo_r_w_get(mpu9250 *const device)
{
  return mpu9250_read(device, MPU9250_FIFO_R_W);
}

// FIFO Samples

static mpu9250_sample fifo_accel_sample;
static mpu9250_sample fifo_gyro_sample;
static mpu9250_sample fifo_magneto_sample;

uint8_t mpu9250_fifo_sample_count(mpu9250 *const device, const mpu9250_fifo_en *const fifo_en)
{
  return mpu9250_fifo_count_get(device) / mpu9250_fifo_en_packet_size(fifo_en);
}

uint8_t mpu9250_fifo_sample_read(mpu9250 *const device, const mpu9250_fifo_en *const fifo_en, mpu9250_fifo_sample *const fifo_sample)
{
  if (mpu9250_fifo_sample_count(device, fifo_en) < 1) return 0;

  if (fifo_en->accel)
  {
    fifo_accel_sample.x = (mpu9250_fifo_r_w_get(device) << 8) | mpu9250_fifo_r_w_get(device);
    fifo_accel_sample.y = (mpu9250_fifo_r_w_get(device) << 8) | mpu9250_fifo_r_w_get(device);
    fifo_accel_sample.z = (mpu9250_fifo_r_w_get(device) << 8) | mpu9250_fifo_r_w_get(device);
    fifo_sample->accel_sample = &fifo_accel_sample;
  }
  else
  {
    fifo_sample->accel_sample = 0;
  }

  if (fifo_en->temp_fifo_en)
  {
    // Currently ignored
    mpu9250_fifo_r_w_get(device);
    mpu9250_fifo_r_w_get(device);
  }

  uint8_t gyro_hit = 0;
  if (fifo_en->gyro_xout)
  {
    fifo_gyro_sample.x = (int16_t)((mpu9250_fifo_r_w_get(device) << 8) | mpu9250_fifo_r_w_get(device));
    gyro_hit |= 1;
  }

  if (fifo_en->gyro_yout)
  {
    fifo_gyro_sample.y = (int16_t)((mpu9250_fifo_r_w_get(device) << 8) | mpu9250_fifo_r_w_get(device));
    gyro_hit |= 1;
  }

  if (fifo_en->gyro_zout)
  {
    fifo_gyro_sample.z = (int16_t)((mpu9250_fifo_r_w_get(device) << 8) | mpu9250_fifo_r_w_get(device));
    gyro_hit |= 1;
  }

  if (gyro_hit)
  {
    fifo_sample->gyro_sample = &fifo_gyro_sample;
  }
  else
  {
    fifo_sample->gyro_sample = 0;
  }

  fifo_sample->magneto_sample = 0;

  return 1;
}

// Gyro Bias

uint8_t mpu9250_xg_offset_set(mpu9250 *const device, const int16_t xg_offset)
{
  return mpu9250_write(device, MPU9250_XG_OFFSET_H, (xg_offset >> 8) & 0xFF) &&
         mpu9250_write(device, MPU9250_XG_OFFSET_L, (xg_offset >> 0) & 0xFF);
}

int16_t mpu9250_xg_offset_get(mpu9250 *const device)
{
  return (mpu9250_read(device, MPU9250_XG_OFFSET_H) << 8) | mpu9250_read(device, MPU9250_XG_OFFSET_L);
}

uint8_t mpu9250_yg_offset_set(mpu9250 *const device, const int16_t yg_offset)
{
  return mpu9250_write(device, MPU9250_YG_OFFSET_H, (yg_offset >> 8) & 0xFF) &&
         mpu9250_write(device, MPU9250_YG_OFFSET_L, (yg_offset >> 0) & 0xFF);
}

int16_t mpu9250_yg_offset_get(mpu9250 *const device)
{
  return (mpu9250_read(device, MPU9250_YG_OFFSET_H) << 8) | mpu9250_read(device, MPU9250_YG_OFFSET_L);
}

uint8_t mpu9250_zg_offset_set(mpu9250 *const device, const int16_t zg_offset)
{
  return mpu9250_write(device, MPU9250_ZG_OFFSET_H, (zg_offset >> 8) & 0xFF) &&
         mpu9250_write(device, MPU9250_ZG_OFFSET_L, (zg_offset >> 0) & 0xFF);
}

int16_t mpu9250_zg_offset_get(mpu9250 *const device)
{
  return (mpu9250_read(device, MPU9250_ZG_OFFSET_H) << 8) | mpu9250_read(device, MPU9250_ZG_OFFSET_L);
}

// INT_STATUS

uint8_t mpu9250_int_status_pack(const mpu9250_int_status *const int_status)
{
  return (int_status->wom_int << 6) |
    (int_status->fifo_overflow_int << 4) |
    (int_status->fsync_int << 3) |
    (int_status->raw_data_rdy_int << 0);
}
uint8_t mpu9250_int_status_unpack(const uint8_t val, mpu9250_int_status *const int_status)
{
  int_status->wom_int = (val >> 6) & 1;
  int_status->fifo_overflow_int = (val >> 4) & 1;
  int_status->fsync_int = (val >> 3) & 1;
  int_status->raw_data_rdy_int = (val >> 0) & 1;
  return 1;
}
uint8_t mpu9250_int_status_get(mpu9250 *const device, mpu9250_int_status *const int_status)
{
  return mpu9250_int_status_unpack(mpu9250_read(device, MPU9250_INT_STATUS), int_status);
}