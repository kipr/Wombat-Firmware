/*
 * MPU925.h
 *
 *  Created on: 23 ��� 2018 �.
 *      Author: Max
 *  Modified by: Tobias Madlberger, Marc Prantl, Matthias, 2025
 *  * Corresponding author's email: tobias.madlberger@gmail.com
 */

#ifndef MPU925_H_
#define MPU925_H_

#include <stdint.h>

typedef enum
{
    GYRO_RANGE_250DPS = 0,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
} GyroRange;

typedef struct
{
    GyroRange range;
    int config; // FS Sel register value 
    float lsb; // LSB value per degree per second
} GyroRangeInfo;

typedef enum AccelRange_
{
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
} AccelRange;

typedef struct
{
    AccelRange range;
    int config; // AFS Sel register value
    float sensitivity; // LSB value per g
} AccelRangeInfo;

typedef enum DLPFBandwidth_
{
    DLPF_BANDWIDTH_184HZ = 0,
    DLPF_BANDWIDTH_92HZ,
    DLPF_BANDWIDTH_41HZ,
    DLPF_BANDWIDTH_20HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;

typedef struct
{
    DLPFBandwidth bandwidth;
    int config; // DLPF register value
} DLPFBandwidthInfo;

typedef enum SampleRateDivider_
{
    LP_ACCEL_ODR_0_24HZ = 0,
    LP_ACCEL_ODR_0_49HZ,
    LP_ACCEL_ODR_0_98HZ,
    LP_ACCEL_ODR_1_95HZ,
    LP_ACCEL_ODR_3_91HZ,
    LP_ACCEL_ODR_7_81HZ,
    LP_ACCEL_ODR_15_63HZ,
    LP_ACCEL_ODR_31_25HZ,
    LP_ACCEL_ODR_62_50HZ,
    LP_ACCEL_ODR_125HZ,
    LP_ACCEL_ODR_250HZ,
    LP_ACCEL_ODR_500HZ
} SampleRateDivider;

typedef enum AK8963Mode_
{
    AK8963_MODE_14BITS_8HZ, // Continuous measurement mode 1
    AK8963_MODE_14BITS_100HZ, // Continuous measurement mode 2
    AK8963_MODE_16BITS_8HZ, // Continuous measurement mode 1
    AK8963_MODE_16BITS_100HZ, // Continuous measurement mode 2
} AK8963Mode;

typedef struct
{
    AK8963Mode mode;
    int config; // AK8963_CNTL1 register value
    float scaleFactor;
} AK8963ModeInfo;

extern const GyroRangeInfo gyroRanges[];
extern const AccelRangeInfo accelRanges[];
extern const DLPFBandwidthInfo dlpfBandwidths[];
extern const AK8963ModeInfo ak8963Modes[];

inline GyroRangeInfo getInfoFromRange(const GyroRange range);
inline AccelRangeInfo getAccelInfoFromRange(const AccelRange range);
inline DLPFBandwidthInfo getDLPFInfoFromRange(const DLPFBandwidth range);
inline AK8963ModeInfo getAK8963InfoFromMode(const AK8963Mode range);

uint8_t MPU9250_Init();
/* read the data, each argiment should point to a array for x, y, and x */
void MPU9250_GetData(float* AccData, float* MagData, float* GyroData);

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd);
/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth);
/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range);
/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range);
/* Sets the magnetometer continuous mode to values other than default */
void MPU9250_SetMagnetometerMode(AK8963Mode mode);

#endif /* MPU925_H_ */
