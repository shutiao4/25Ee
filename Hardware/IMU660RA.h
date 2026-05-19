#ifndef __IMU660RA_H
#define __IMU660RA_H

#include "stm32f10x.h"

uint8_t IMU660RA_Init(void);
uint8_t IMU660RA_GetID(void);
void IMU660RA_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                      int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/* ---- 卡尔曼滤波器 ---- */

// 初始化卡尔曼滤波器参数
void IMU660RA_Kalman_Init(float Q, float R);

/* ---- 陀螺仪偏航角处理  ---- */

// 初始化并返回状态（封装了 IMU660RA_Init 的调用）
uint8_t IMU660RA_GetInitStatus(void);

// 陀螺仪Z轴零偏校准（取100次平均值），需在 IMU660RA_Init 成功后调用
void IMU660RA_CalibrateGyroZ(void);

// 基于当前 GZ 值更新偏航角（陀螺仪Z轴积分，带卡尔曼滤波，推荐使用）
void IMU660RA_UpdateYaw_Filtered(int16_t GZ);

// 基于当前 GZ 值更新偏航角（陀螺仪Z轴积分，无滤波，原始行为）
void IMU660RA_UpdateYaw(int16_t GZ);

// 获取计算得到的偏航角（度，0~360）
float IMU660RA_GetYaw(void);

// 获取陀螺仪Z轴零偏值
int16_t IMU660RA_GetGyroZBias(void);

#endif
