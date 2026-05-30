#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);

uint8_t MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_Kalman_Init(float Q, float R);
void MPU6050_CalibrateGyroZ(void);
void MPU6050_UpdateYaw_Filtered(int16_t GZ);
void MPU6050_UpdateYaw(int16_t GZ);
float MPU6050_GetYaw(void);
int16_t MPU6050_GetGyroZBias(void);

#endif
