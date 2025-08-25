/*
*  Modified by: Tobias Madlberger*, Marc Prantl, Matthias, 2025
*  * Corresponding author's email: tobias.madlberger@gmail.com
*/

#include "wallaby_imu.h"

#include <MPU9250.h>

#include "wallaby.h"

float AccData[3], GyroData[3], MagData[3];

void setupIMU()
{
    if (MPU9250_Init() != 0) return;

    MPU9250_SetAccelRange(ACCEL_RANGE_2G);
    MPU9250_SetGyroRange(GYRO_RANGE_2000DPS);
    MPU9250_SetDLPFBandwidth(DLPF_BANDWIDTH_5HZ);
}

void readIMU()
{
    MPU9250_GetData(AccData, MagData, GyroData);

    pack_float(REG_RW_GYRO_X_0, GyroData[0]);
    pack_float(REG_RW_GYRO_Y_0, GyroData[1]);
    pack_float(REG_RW_GYRO_Z_0, GyroData[2]);

    pack_float(REG_RW_ACCEL_X_0, AccData[0]);
    pack_float(REG_RW_ACCEL_Y_0, AccData[1]);
    pack_float(REG_RW_ACCEL_Z_0, AccData[2]);

    pack_float(REG_RW_MAG_X_0, MagData[0]);
    pack_float(REG_RW_MAG_Y_0, MagData[1]);
    pack_float(REG_RW_MAG_Z_0, MagData[2]);
}
