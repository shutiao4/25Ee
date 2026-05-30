#ifndef	__LINE_H__
#define __LINE_H__

#include "stm32f10x.h"

#define ADC_N 8

void track_zhixian1(void);
void track_curve(int base_speed);
void Line_Kalman_Reset(void);
void Line_Set_Curve_Hold(int8_t direction);
void track_zhixian2(void);
void track_PID1(int pwm,float P);
void track_PID2(int pwm,float P);
void track_PID3(int pwm,float P);
char Check_BlackLine(void);

/* 获取最后已知的线位置偏差（正=偏右，负=偏左），用于判断转弯方向 */
int8_t Get_Line_Bias(void);

#endif
