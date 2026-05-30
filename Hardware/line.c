/******************************************************************************
 * line.c - 循迹模块
 *
 * 功能：
 *   1. 8路红外传感器（D1~D8）检测黑线位置
 *   2. 使用增量式PID控制算法计算偏差输出
 *   3. 根据偏差调整左右轮PWM，实现沿黑线行驶
 *   4. Check_BlackLine()提供黑线检测状态
 *   5. Check_Corner()检测90°转弯顶点（所有传感器检测到黑线）
 *   6. Get_Line_Bias()返回最后已知的线位置偏差
 *
 * 硬件连接：
 *   - 红外传感器模块输出接至 MCU GPIO（D1 ~ D8）
 *   - 传感器检测到黑线（反射率低）时输出 1
 *   - 传感器检测到白底（反射率高）时输出 0
 *
 ******************************************************************************/

#include "Line.h"

#include "sensor.h"

/* 外部函数声明 -------------------------------------------------------------- */
void Set_PWM(int V_R, int V_L);

/* 外部变量声明 -------------------------------------------------------------- */
extern int V_R;   /* 右轮PWM值 */
extern int V_L;   /* 左轮PWM值 */

/* PID参数（需根据实际赛道调试）--------------------------------------------- */
static float line_kp = 1.2f;   /* 比例系数 */
static float line_ki = 0.0f;   /* 积分系数 */
static float line_kd = 7.0f;   /* 微分系数 */

/* PID中间变量 -------------------------------------------------------------- */
static float line_integral     = 0.0f;
static float line_last_error   = 0.0f;

/* 最后已知的线偏差 --------------------------------------------------------- */
static float last_line_bias = 0.0f;

/* 直角弯权重保持 -----------------------------------------------------------
 * 检测到直角弯后设为一个固定偏差(±20)，掉线时直接使用此值转向，
 * 直到传感器重新抓到黑线后自动清零恢复正常循迹。
 * 0 = 正常模式，正数=右转保持，负数=左转保持
 */
static int8_t curve_hold_weight = 0;

/* 权重保持计时器 -----------------------------------------------------------
 * 进入弯道设权重时启动计时(25 tick = 0.5s)，
 * 计时结束前即使扫到黑线也不清除权重保持，
 * 确保小车在弯道入口有足够的强制转向时间。
 */
static uint16_t curve_hold_timer = 0;

/* 卡尔曼滤波器结构体 ------------------------------------------------------- */
typedef struct
{
    float x;  /* 状态估计值（滤波后的偏差） */
    float P;  /* 估计误差协方差 */
    float Q;  /* 过程噪声协方差（越大越信任测量，越小越平滑） */
    float R;  /* 测量噪声协方差（越大越平滑，越小响应越快） */
    float K;  /* 卡尔曼增益 */
} Kalman_t;

/* 循迹卡尔曼滤波器实例 ----------------------------------------------------- */
static Kalman_t line_kalman = {
    .x = 0.0f,
    .P = 1.0f,
    .Q = 0.05f,   /* 过程噪声 - 调小使轨迹更平滑 */
    .R = 3.0f,    /* 测量噪声 - 传感器噪声较大时可适当增大 */
    .K = 0.0f
};

/**
 * @brief  一维卡尔曼滤波器
 * @param  kf  卡尔曼滤波器结构体指针
 * @param  z   测量值（原始传感器偏差）
 * @retval 滤波后的估计值
 * @note   标准一维卡尔曼滤波五公式：
 *         预测：P(k|k-1) = P(k-1|k-1) + Q
 *         更新：K = P(k|k-1) / (P(k|k-1) + R)
 *               x(k|k) = x(k|k-1) + K * (z - x(k|k-1))
 *               P(k|k) = (1 - K) * P(k|k-1)
 */
static float Kalman_Filter(Kalman_t *kf, float z)
{
    /* 1. 预测阶段：先验估计协方差 */
    kf->P = kf->P + kf->Q;

    /* 2. 更新阶段：计算卡尔曼增益 */
    kf->K = kf->P / (kf->P + kf->R);

    /* 3. 更新阶段：用测量值修正状态估计 */
    kf->x = kf->x + kf->K * (z - kf->x);

    /* 4. 更新阶段：更新估计误差协方差 */
    kf->P = (1.0f - kf->K) * kf->P;

    return kf->x;
}

/**
 * @brief  循迹直行函数
 * @note   使用8路红外传感器（D1~D8）检测黑线
 *         通过偏差调整左右轮PWM实现沿黑线行驶
 */
void track_zhixian1(void)
{
    int base_speed = 13;         /* 基础速度 */
    int sum        = 0;
    int count      = 0;
    int error      = 0;
    float output   = 0.0f;
    int correction = 0;

    /* ---- 1. 读取8路传感器，计算加权和 ---- */
    if(D1) { sum += 7; count++; }   /* 最左侧 */
    if(D2) { sum += 5; count++; }
    if(D3) { sum += 3; count++; }
    if(D4) { sum += 1; count++; }
    if(D5) { sum += -1; count++; }
    if(D6) { sum += -3; count++; }
    if(D7) { sum += -5; count++; }
    if(D8) { sum += -7; count++; }   /* 最右侧 */

    /* ---- 2. 无传感器检测到黑线：掉线处理 ---- */
    if(count == 0)
    {
        /* 掉线时使用上一帧的滤波值，卡尔曼滤波器会自然衰减到0 */
        Kalman_Filter(&line_kalman, 0.0f);
        error = (int)line_kalman.x;
        goto pid_calc;
    }

    /* ---- 3. 计算平均偏差 ---- */
    error = sum / count;

    /* ---- 4. 卡尔曼滤波平滑偏差 ---- */
    Kalman_Filter(&line_kalman, (float)error);
    last_line_bias = line_kalman.x;

    /* 使用滤波后的值进行PID计算 */
    error = (int)line_kalman.x;

pid_calc:
    /* ---- 5. 增量式PID计算修正量 ---- */
    line_integral += (float)error;
    if(line_integral > 20)   line_integral =  20;
    if(line_integral < -20)  line_integral = -20;

    output = line_kp * (float)error
           + line_ki * line_integral
           + line_kd * ((float)error - line_last_error);

    line_last_error = (float)error;

    correction = (int)output;

    /* 限幅 */
    if(correction > 20) correction = 20;
    if(correction < -20) correction = -20;

    /* ---- 6. 计算左右轮PWM ---- */
    /*    correction为正（线偏右）→ 需要右转纠偏：左轮加速、右轮减速 */
    /*    correction为负（线偏左）→ 需要左转纠偏：右轮加速、左轮减速 */
    V_R = base_speed - correction;
    V_L = base_speed + correction;

    /* 限幅 */
    if(V_R > 100) V_R = 100;
    if(V_R < -100) V_R = -100;
    if(V_L > 100) V_L = 100;
    if(V_L < -100) V_L = -100;

    /* ---- 7. 输出PWM ---- */
    Set_PWM(V_R, V_L);
}

/**
 * @brief  重置循迹卡尔曼滤波器和PID状态
 * @note   在每次开始新一段循迹前调用，清除上一段的积分和估计值
 */
void Line_Kalman_Reset(void)
{
    line_kalman.x = 0.0f;
    line_kalman.P = 1.0f;
    line_kalman.K = 0.0f;
    line_integral     = 0.0f;
    line_last_error   = 0.0f;
    last_line_bias    = 0.0f;
    curve_hold_weight = 0;
    curve_hold_timer  = 0;
}

/**
 * @brief  设置直角弯权重保持值
 * @param  direction  弯道方向：1=右弯，-1=左弯，0=取消保持
 * @note   调用后 track_curve() 在掉线时会将偏差锁定为 ±20，
 *         强制保持 0.5s 后开始检测黑线恢复循迹。
 *         应在进入 STATE4_CURVE 时调用。
 */
void Line_Set_Curve_Hold(int8_t direction)
{
    if (direction > 0)
        curve_hold_weight = 5.5;   /* 右转最大权重 */
    else if (direction < 0)
        curve_hold_weight = -5.5;  /* 左转最大权重 */
    else
        curve_hold_weight = 0;   /* 取消保持 */

    /* 启动 0.5s 强制保持计时（25 tick × 20ms = 500ms）
     * 计时结束前即使扫到黑线也不清除权重保持，
     * 确保小车在弯道入口有足够的强制转向时间 */
    curve_hold_timer = 25;
}

/**
 * @brief  弯道循迹函数（卡尔曼滤波+差速过弯）
 * @param  base_speed  弯道基础速度（建议 6~10，比直行稍低）
 * @note   卡尔曼滤波在弯道中的优势：
 *         1. 平滑传感器切换偏差，避免过弯时PWM突变
 *         2. 掉线时保持卡尔曼估计的转向趋势（不衰减到0）
 *         3. 出弯时滤波器自然过渡，减少回中过冲
 */
void track_curve(int base_speed)
{
    int sum        = 0;
    int count      = 0;
    int error      = 0;
    float output   = 0.0f;
    int correction = 0;

    /* ---- 1. 读取8路传感器，计算加权和 ---- */
    if(D1) { sum += 7; count++; }
    if(D2) { sum += 5; count++; }
    if(D3) { sum += 3; count++; }
    if(D4) { sum += 1; count++; }
    if(D5) { sum += -1; count++; }
    if(D6) { sum += -3; count++; }
    if(D7) { sum += -5; count++; }
    if(D8) { sum += -7; count++; }

    /* ---- 2. 弯道掉线处理（卡尔曼滤波 + 权重保持） ---- */
    if(count == 0)
    {
        /* 直角弯弯心处黑线会超出传感器范围：
         * 将保持权重作为测量值输入卡尔曼滤波器，
         * 滤波器平滑收敛到该值，保持期结束时自然过渡到正常循迹，
         * 避免直接切换权重导致的PID微分突变 */
        if (curve_hold_weight != 0)
        {
            if (curve_hold_timer > 0) curve_hold_timer--;
            Kalman_Filter(&line_kalman, (float)curve_hold_weight);
            last_line_bias = line_kalman.x;
            error = (int)line_kalman.x;
        }
        else
        {
            Kalman_Filter(&line_kalman, line_kalman.x);
            error = (int)line_kalman.x;
        }
        goto pid_calc;
    }

    /* ---- 3. 有黑线 → 卡尔曼滤波保持，计时归零后恢复 ---- */
    if (curve_hold_timer > 0)
    {
        /* 0.5s 强制保持期内，即使扫到黑线也不清除权重：
         * 将 curve_hold_weight 作为测量值送入卡尔曼滤波器，
         * 滤波后偏差平滑趋近 ±20，不会产生跳变；
         * 0.5s 后 timer=0，清除权重，恢复正常传感器计算，
         * 卡尔曼滤波器自然衔接，过渡平滑无突变 */
        curve_hold_timer--;
        Kalman_Filter(&line_kalman, (float)curve_hold_weight);
        last_line_bias = line_kalman.x;
        error = (int)line_kalman.x;
        goto pid_calc;
    }
    curve_hold_weight = 0;

    /* ---- 4. 计算平均偏差 ---- */
    error = sum / count;

    /* ---- 5. 卡尔曼滤波平滑偏差 ----
     *  原始偏差 error 作为测量值 z 输入卡尔曼滤波器，
     *  滤波器结合上一时刻的状态估计 x 给出最优估计，
     *  有效抑制传感器在黑白交界处的抖动
     */
    Kalman_Filter(&line_kalman, (float)error);
    last_line_bias = line_kalman.x;
    error = (int)line_kalman.x;

pid_calc:
    /* ---- 6. 增量式PID计算（积分限幅放宽，允许更大累积） ---- */
    line_integral += (float)error;
    if(line_integral > 30)   line_integral =  30;
    if(line_integral < -30)  line_integral = -30;

    output = line_kp * (float)error
           + line_ki * line_integral
           + line_kd * ((float)error - line_last_error);

    line_last_error = (float)error;
    correction = (int)output;

    /* 弯道修正限幅放宽，允许更大转向 */
    if(correction > 40) correction = 40;
    if(correction < -40) correction = -40;

    /* ---- 7. 计算左右轮PWM（差速过弯） ---- */
    V_R = base_speed - correction;
    V_L = base_speed + correction;

    if(V_R > 100) V_R = 100;
    if(V_R < -100) V_R = -100;
    if(V_L > 100) V_L = 100;
    if(V_L < -100) V_L = -100;

    /* ---- 8. 输出PWM ---- */
    Set_PWM(V_R, V_L);
}

/**
 * @brief  获取最后已知的线位置偏差
 * @retval 偏差值（正=偏右，负=偏左）
 */
int8_t Get_Line_Bias(void)
{
    return (int8_t)(last_line_bias);
}

/**
 * @brief  检测是否到达90°转弯顶点
 * @note   当D1~D8全部检测到黑线（连续3次确认），说明到达交叉点
 * @retval 1=到达顶点，0=未到达
 */
uint8_t Check_Corner(void)
{
    static uint8_t corner_cnt = 0;

    if(D1 && D2 && D3 && D4 && D5 && D6 && D7 && D8)
    {
        if(corner_cnt < 3) corner_cnt++;
    }
    else
    {
        corner_cnt = 0;
    }

    return (corner_cnt >= 3) ? 1 : 0;
}

/**
 * @brief  黑线检测函数
 * @note   任一传感器检测到黑线即返回1
 * @retval 1=检测到黑线，0=未检测到
 */
char Check_BlackLine(void)
{
    if(D1 || D2 || D3 || D4 || D5 || D6 || D7 || D8)
    {
        return 1;
    }
    return 0;
}
