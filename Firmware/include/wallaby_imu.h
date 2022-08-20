#ifndef WALLABY_IMU_H
#define WALLABY_IMU_H

#include "stm32f4xx.h"

void setupIMU();
void readIMU();

void setGyroSensitivity(uint8_t sensitivity);
void setAccelSensitivity(uint8_t sensitivity);

uint8_t getCachedGyroSensitivity();
uint8_t getCachedAccelSensitivity();
#endif /* WALLABY_IMU_H */