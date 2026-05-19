#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

// 左右电机最终目标速度
extern int V_R, V_L;
// 陀螺仪零偏校准（实际调用IMU660RA的校准），并初始化直行PID参数
void IMU660ra_Calibrate(void);
// 读取陀螺仪并积分更新当前车体角度
void Car_Update_Angle(void);
// 获取当前累计偏航角
float Car_Get_Angle(void);
// 设置/读取直行阶段要锁定的目标航向角
void Car_Set_Straight_Target(float target);
float Car_Get_Straight_Target(void);
// 把当前朝向锁成后续直行目标，并清空直行PID积分
void Car_Lock_Current_Heading(void);
// 清零累计角度、滤波器和PID积分项
void Car_Reset_Angle(void);
// 直行控制：基于当前角度偏差修正左右轮速度
void Car_Go_Straight(int speed);
// 0.75秒内从0加速到目标速度（带目标航向闭环），需在20ms周期中持续调用
void Car_Go_Straight_To_Target_Ramp(int target_speed, float target_yaw);

/* ====== 新添加的PID控制函数 ====== */

/**
 * @brief  按目标角度直行（基于Yaw闭环）
 * @param  speed     基础速度（含方向，正=前进，负=后退）
 * @param  target_yaw 目标航向角（度，-180~180）
 * @note   内部自动调用 Car_Update_Angle 更新角度
 */
void Car_Go_Straight_To_Target(int speed, float target_yaw);

/**
 * @brief  使用PID闭环控制转弯到指定Yaw角度
 * @param  target_yaw 目标偏航角（度，-180~180）
 * @param  speed      转弯速度 (0~100)
 * @retval 0=正在转弯中, 1=已到达目标角度
 * @note   需要在主循环中周期性调用直到返回1
 */
uint8_t Car_Turn_To_Yaw(float target_yaw, int speed);

#endif
