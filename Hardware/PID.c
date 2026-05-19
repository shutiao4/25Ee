#include "PID.h"
#include "IMU660RA.h"
#include "delay.h"
#include "pwm.h"

// 车辆直行目标角、yaw角度偏移量（支持重置角度）
static float straight_target_angle = 0.0f;
static float yaw_offset = 0.0f;
extern int V_R;               // 右电机速度输出
extern int V_L;               // 左电机速度输出

// 直行控制使用的位置式PID结构体
typedef struct
{
    float target;
    float measure;
    float err;
    float last_err;
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float output;
} PID_TypeDef;

static PID_TypeDef PID_Str;        // 直行控制PID实例

static PID_TypeDef PID_Turn;       // 转弯控制PID实例

// 初始化直行控制PID参数
static void PID_Init(float kp, float ki, float kd)
{
    PID_Str.Kp = kp;
    PID_Str.Ki = ki;
    PID_Str.Kd = kd;
    PID_Str.target = 0;
    PID_Str.measure = 0;
    PID_Str.err = 0;
    PID_Str.last_err = 0;
    PID_Str.integral = 0;
    PID_Str.output = 0;
}

// 初始化转弯控制PID参数
static void PID_Turn_Init(float kp, float ki, float kd)
{
    PID_Turn.Kp = kp;
    PID_Turn.Ki = ki;
    PID_Turn.Kd = kd;
    PID_Turn.target = 0;
    PID_Turn.measure = 0;
    PID_Turn.err = 0;
    PID_Turn.last_err = 0;
    PID_Turn.integral = 0;
    PID_Turn.output = 0;
}
// 采样陀螺仪静态零偏，并重置直行PID
void IMU660ra_Calibrate(void)
{
    // 调用IMU660RA零偏校准（100次采样均值+卡尔曼初始化）
    IMU660RA_CalibrateGyroZ();
    yaw_offset = IMU660RA_GetYaw(); // 校准后当前yaw设为偏移基准

    // 直行PID参数，可根据实际跑车效果继续微调
    PID_Init(1.3f, 0.0f,7.0f);
   // 1.2f, 0.0f,4.0f speed=30-40

    // 转弯PID参数（Kp=每1°误差对应的PWM增量，Kd抑制过冲）
    // 新映射方式：PID输出直接作为PWM幅度，不再乘以固定系数
    // 推荐调试顺序：先调Kp使转弯速度合适，再加Kd消除过冲
    PID_Turn_Init(1.0f, 0.00f, 0.5f);
}
// 位置式PID计算（通用，可指定PID实例）
static float PID_Calc_Generic(PID_TypeDef *pid, float measure, float target)
{
    pid->measure = measure;
    pid->target = target;

    pid->err = pid->target - pid->measure;

    // 积分限幅，避免长时间偏差导致积分饱和
    pid->integral += pid->err;
    if (pid->integral > 200)
        pid->integral = 200;
    if (pid->integral < -200)
        pid->integral = -200;

    // 位置式PID输出
    pid->output = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->last_err);

    pid->last_err = pid->err;
    return pid->output;
}

// 直行PID专用计算（保持原有行为）
static float PID_Calc(float measure, float target)
{
    return PID_Calc_Generic(&PID_Str, measure, target);
}

// 清空PID内部状态，通常在重新开始任务或转弯前调用
static void PID_Clear(void)
{
    PID_Str.err = 0;
    PID_Str.last_err = 0;
    PID_Str.integral = 0;
    PID_Str.output = 0;
}

// 清空转弯PID内部状态
static void PID_Turn_Clear(void)
{
    PID_Turn.err = 0;
    PID_Turn.last_err = 0;
    PID_Turn.integral = 0;
    PID_Turn.output = 0;
}

/**
 * @brief  将角度差归一化到 -180° ~ +180°
 */
static float Angle_Normalize(float diff)
{
    while (diff > 180.0f)
        diff -= 360.0f;
    while (diff < -180.0f)
        diff += 360.0f;
    return diff;
}



// Yaw已在main.c主循环中每20ms更新一次（IMU660RA_GetData + IMU660RA_UpdateYaw_Filtered）
// Car_Update_Angle() 仅作为获取最新Yaw值的占位函数，避免重复积分
// 所有PID函数内部调用此函数时不再重复读取IMU
void Car_Update_Angle(void)
{
    /* 无需操作：Yaw已在main.c的主循环中完成更新 */
}

// 读取当前偏航角（基于IMU660RA，以校准时刻为基准偏移）
float Car_Get_Angle(void)
{
    float diff = IMU660RA_GetYaw() - yaw_offset;

    // 归一化到 -180° ~ +180°
    if (diff > 180.0f)
        diff -= 360.0f;
    if (diff < -180.0f)
        diff += 360.0f;

    return diff;
}

void Car_Set_Straight_Target(float target)
{
    straight_target_angle = target;
}

float Car_Get_Straight_Target(void)
{
    return straight_target_angle;
}

void Car_Lock_Current_Heading(void)
{
    straight_target_angle = Car_Get_Angle();
    PID_Clear();
}

// 重新开始角度计算：以当前IMU yaw为偏移基准
void Car_Reset_Angle(void)
{
    yaw_offset = IMU660RA_GetYaw();
    straight_target_angle = 0.0f;
    PID_Clear();
}

// 直行控制：根据车头偏角修正左右轮PWM，实现跑直线
void Car_Go_Straight(int speed)
{
    float angle;
    float err;
    int correction;

    Car_Update_Angle();
    angle = Car_Get_Angle();

    // 计算归一化的角度偏差（-180° ~ +180°）
    err = Angle_Normalize(straight_target_angle - angle);

    // 起步或微小抖动时不做差速修正，避免左右轮一开始就被拉开
    if (err > -1.0f && err < 1.0f)
    {
        PID_Clear();
        correction = 0;
    }
    else
    {
        // 使用归一化后的误差进行PID计算
        correction = (int)PID_Calc(0, err);
    }
    if (correction > 15)
        correction = 15;
    if (correction < -15)
        correction = -15;

    // 通过差速修正航向
    V_L = speed - correction;
    V_R = speed + correction;

    // 输出限幅，避免PWM超出允许范围
    if (V_L > 100)
        V_L = 100;
    if (V_L < -100)
        V_L = -100;
    if (V_R > 100)
        V_R = 100;
    if (V_R < -100)
        V_R = -100;

    Set_PWM(V_R, V_L);
}

// 0.75秒内从0加速到目标速度（带目标航向闭环），需在20ms周期中持续调用
void Car_Go_Straight_To_Target_Ramp(int target_speed, float target_yaw)
{
    const int ramp_steps = 25; // 0.75秒 / 20ms ≈ 37步
    static int last_target_speed = 0;
    static float last_target_yaw = 0.0f;
    static int current_step = 0;
    static int current_speed = 0;

    // 目标速度或目标角度变化时重置加速过程
    if (target_speed != last_target_speed || target_yaw != last_target_yaw)
    {
        last_target_speed = target_speed;
        last_target_yaw = target_yaw;
        current_step = 0;
        current_speed = 0;
    }

    if (target_speed == 0)
    {
        current_speed = 0;
        Car_Go_Straight_To_Target(0, target_yaw);
        return;
    }

    if (current_step < ramp_steps)
    {
        float ratio = (float)(current_step + 1) / (float)ramp_steps;
        current_speed = (int)(target_speed * ratio + (target_speed >= 0 ? 0.5f : -0.5f));
        current_step++;
    }
    else
    {
        current_speed = target_speed;
    }

    Car_Go_Straight_To_Target(current_speed, target_yaw);
}

/* ===================================================================
 *  新添加的函数：按目标角度直行 + PID闭环转弯到指定Yaw
 * =================================================================== */

/**
 * @brief  按目标Yaw角度直行（闭环控制）
 *
 *         优化说明（2026-05-08）：
 *         - 原来使用简单P控制（p_gain=0.15，correction限幅±3），
 *           导致纠偏力度严重不足和整数截杀死区。
 *         - 现在改为复用 Car_Go_Straight 中的位置式PID（Kp=1.0, Kd=5.0，
 *           correction限幅±4），保证一致的纠偏强度。
 *         - 解决起步时左右轮速度不一致的问题（弱PID无法纠正硬件不对称）。
 */
void Car_Go_Straight_To_Target(int speed, float target_yaw)
{
    float angle;
    float err;
    int correction;

    // 1. 更新当前角度
    Car_Update_Angle();
    angle = Car_Get_Angle();

    // 2. 计算归一化的角度偏差（-180° ~ +180°）
    err = Angle_Normalize(target_yaw - angle);

    // 3. 小死区：偏差<1度时不做修正，避免频繁抖动
    if (err > -1.0f && err < 1.0f)
    {
        PID_Clear();
        correction = 0;
    }
    else
    {
        // 使用归一化后的误差进行PID计算
        // 直接传入归一化误差err，PID输出 = Kp*err + Kd*(err - last_err)
        correction = (int)PID_Calc(0, err);
    }

    // 限幅修正量（与Car_Go_Straight保持一致）
    if (correction > 15)
        correction = 15;
    if (correction < -15)
        correction = -15;

    // 4. 计算左右轮速度（与Car_Go_Straight符号一致）
    //    修正量为正 → 左轮加速、右轮减速 → 向右纠偏
    V_L = speed - correction;
    V_R = speed + correction;

    // 5. 输出限幅
    if (V_L > 100)
        V_L = 100;
    if (V_L < -100)
        V_L = -100;
    if (V_R > 100)
        V_R = 100;
    if (V_R < -100)
        V_R = -100;

    Set_PWM(V_R, V_L);
}

/**
 * @brief  使用PID闭环控制，使车体精确转到指定Yaw角度
 *
 *         工作原理：
 *         - 计算目标Yaw与当前Yaw的误差（归一化到-180~180）
 *         - 通过转弯PID（位置式）计算修正量
 *         - 修正量为正表示需要顺时针转 → 右轮正PWM、左轮负PWM
 *         - 修正量为负表示需要逆时针转 → 右轮负PWM、左轮正PWM
 *         - 到达目标角度后自动停止并返回1
 *
 * @param  target_yaw  目标偏航角（度，-180~180）
 * @param  speed       最大转弯速度 (0~100)，越大转弯越快但可能过冲
 * @retval 0  正在转弯中，需要继续调用
 * @retval 1  已到达目标角度，本次转弯完成
 */
uint8_t Car_Turn_To_Yaw(float target_yaw, int speed)
{
    float angle;
    float err;

    // 1. 更新当前角度
    Car_Update_Angle();
    angle = Car_Get_Angle();

    // 2. 计算归一化误差（-180 ~ +180）
    err = Angle_Normalize(target_yaw - angle);

    // 3. 到达判断：偏差绝对值小于2度认为到位
    if (err > -1.0f && err < 1.0f)
    {
        // 到达目标，停车并清空PID状态
        PID_Turn_Clear();
        Set_PWM(0, 0);
        return 1;
    }

    // 4. 使用转弯PID计算修正量
    //    目标值传0，测量值传err，PID输出 = -Kp*err - Ki*integral - Kd*derr
    //    这样err为正（需顺时针转）时输出为负 → 右轮负、左轮正
    float correction = PID_Calc_Generic(&PID_Turn, err, 0.0f);

    // 5. 将PID输出映射到PWM值
    //    PID输出直接作为PWM幅度，Kp的物理意义=每度误差对应的PWM增量
    //    例如Kp=1.5时，90°误差→PID输出=135→限幅到speed
    float abs_correction = correction;
    if (abs_correction < 0)
        abs_correction = -abs_correction;

    // 根据误差大小动态调整PWM：误差大时全速，误差小时减速
    int pwm_magnitude = (int)(abs_correction);  // 直接使用PID输出值，不再乘以固定系数
    if (pwm_magnitude > speed)
        pwm_magnitude = speed;
    if (pwm_magnitude < 15)   // 最小PWM保证足够力矩启动
        pwm_magnitude = 15;

    // 6. 根据误差方向决定左右轮：err>0 顺时针转
    if (err > 0.0f)
    {
        // 顺时针：右轮反转，左轮正转
        V_R =  -pwm_magnitude;
        V_L = pwm_magnitude;
    }
    else
    {
        // 逆时针：右轮正转，左轮反转
        V_R = pwm_magnitude;
        V_L = -pwm_magnitude;
    }

    // 7. 限幅输出
    if (V_R > 100)  V_R = 100;
    if (V_R < -100) V_R = -100;
    if (V_L > 100)  V_L = 100;
    if (V_L < -100) V_L = -100;

    Set_PWM(V_R, V_L);

    return 0;  // 仍在转弯中
}
