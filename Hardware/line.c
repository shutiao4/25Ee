
/******************************************************************************
 * line.c - 循迹模块
 *
 * 功能：
 *   1. 7路红外传感器（D2~D8）检测黑线位置
 *   2. 使用增量式PID控制算法计算偏差输出
 *   3. 根据偏差调整左右轮PWM，实现沿黑线行驶
 *   4. Check_BlackLine()提供消抖后的黑线检测状态
 *
 * 硬件连接：
 *   - 红外传感器模块输出接至 MCU GPIO（D2 ~ D8）
 *   - 传感器检测到黑线（反射率低）时输出 1
 *   - 传感器检测到白底（反射率高）时输出 0
 *
 * 作者：
 * 日期：
 ******************************************************************************/
#include "Line.h"
#include "sensor.h"

/* 外部函数声明 -------------------------------------------------------------- */
void Set_PWM(int V_R, int V_L);

/* 外部变量声明 -------------------------------------------------------------- */
extern int V_R;   /* 右轮PWM值 */
extern int V_L;   /* 左轮PWM值 */

/* PID参数（需根据实际赛道调试）--------------------------------------------- */
static float line_kp = 6.0f;   /* 比例系数 - 决定对当前偏差的响应强度 */
static float line_ki = 0.0f;   /* 积分系数 - 消除稳态误差（循迹通常设为0） */
static float line_kd =32.0f;   /* 微分系数 - 抑制震荡，提高响应速度 */

/* PID中间变量 -------------------------------------------------------------- */
static float line_integral     = 0.0f;   /* 积分累加值 */
static float line_last_error   = 0.0f;   /* 上一次偏差值（用于微分项计算） */

/**
 * @brief  直行循迹函数（7路传感器 + 增量式PID）
 * @note   使用7个红外传感器（D2~D8）检测黑线位置，传感器从左到右排列：
 *         D2(最左) -> D8(最右)，分别赋予权重 -3 ~ +3。
 *         通过对检测到黑线的传感器权重求和平均得到当前偏差 error，
 *         再经 PID 计算输出，调整左右电机转速实现循迹。
 *
 *         传感器布局与权重对应关系：
 *         D2(-3)  D3(-2)  D4(-1)  D5(0)  D6(+1)  D7(+2)  D8(+3)
 *            左  ←  ←  ←  [车头方向]  →  →  →  右
 *
 *         当没有任何传感器检测到黑线时（count==0），
 *         认为已脱离赛道，让小车后退并返回。
 *
 * @param  无
 * @retval 无
 */
void track_zhixian1(void)
{
    int   base_speed = 35;         /* 基础速度（0~100），实际运行时根据需求调整 */
    int   sum        = 0;          /* 权重累加和 */
    int   count      = 0;          /* 检测到黑线的传感器数量 */
    float error      = 0.0f;       /* 当前偏差（带符号） */
    float output     = 0.0f;       /* PID 控制器输出 */

    /* ---- 1. 读取7路传感器，计算加权和 ---- */
    /*    每个传感器检测到黑线时累加对应权重 */
    if(D2) { sum += -4; count++; }   /* 最左侧 → 偏差负大 */
    if(D3) { sum += -2; count++; }   /* 左偏    → 偏差负 */
    if(D4) { sum += -1; count++; }   /* 微左偏  → 偏差负小 */
    if(D5) { sum +=  0; count++; }   /* 居中    → 偏差0 */
    if(D6) { sum += +1; count++; }   /* 微右偏  → 偏差正小 */
    if(D7) { sum += +2; count++; }   /* 右偏    → 偏差正 */
    if(D8) { sum += +4; count++; }   /* 最右侧  → 偏差正大 */

    /* ---- 2. 无传感器检测到黑线：掉线处理 ---- */
    if(count == 0)
    {
        /* 未检测到黑线，说明可能已脱离赛道，执行后退 */
        Set_PWM(30, 30);
        return;
    }

    /* ---- 3. 计算平均偏差 ---- */
    error = (float)sum / count;

    /* ---- 4. 增量式PID计算 ---- */
    /*    积分项：累加偏差，并做限幅防积分饱和 */
    line_integral += error;
    if(line_integral > 20)   line_integral =  20;
    if(line_integral < -20)  line_integral = -20;

    /*    PID输出 = P * error + I * integral + D * (error - last_error) */
    output = line_kp * error
           + line_ki * line_integral
           + line_kd * (error - line_last_error);

    /*    保存本次偏差供下次微分项使用 */
    line_last_error = error;

    /* ---- 5. 计算左右轮PWM值 ---- */
    /*    PID输出为正 → 右转（右轮减速，左轮加速） */
    /*    PID输出为负 → 左转（左轮减速，右轮加速） */
    V_L = base_speed + (int)output;
    V_R = base_speed - (int)output;

    /* ---- 6. 限幅保护（PWM范围 -100 ~ +100） ---- */
    if(V_L > 100)   V_L =  100;
    if(V_L < -100)  V_L = -100;
    if(V_R > 100)   V_R =  100;
    if(V_R < -100)  V_R = -100;

    /* ---- 7. 输出PWM到电机 ---- */
    Set_PWM(V_L, V_R);
}

/**
 * @brief  黑线检测函数（带消抖滤波）
 * @note   通过软件消抖（连续5次采样）判断是否检测到黑线，
 *         避免传感器信号抖动导致误判。
 *
 *         消抖原理：
 *         - 检测到黑线时（hit==1），black_cnt 递增（上限5）
 *         - 未检测到黑线时（hit==0），black_cnt 递减（下限0）
 *         - black_cnt >= 1 时认为检测到黑线（black_state = 1）
 *         - black_cnt == 0 时认为未检测到（black_state = 0）
 *
 *         这样需要连续1次以上采样确认黑线出现/消失，滤除瞬时干扰。
 *
 * @param  无
 * @retval 1 - 检测到黑线
 *         0 - 未检测到黑线
 */
char Check_BlackLine(void)
{
    static uint8_t black_cnt   = 0;   /* 消抖计数器 */
    static uint8_t black_state = 0;   /* 当前黑线状态（1=检测到，0=未检测到） */
    uint8_t        hit         = 0;   /* 本次采样是否检测到黑线 */

    /* ---- 1. 采样：任一传感器检测到黑线即认为命中 ---- */
    if(D2==1 || D3==1 || D4==1 || D5==1 || D6==1 || D7==1 || D8==1)
    {
        hit = 1;
    }

    /* ---- 2. 消抖计数 ---- */
    if(hit)
    {
        /* 检测到黑线 → 计数器递增（上限5） */
        if(black_cnt < 3)
        {
            black_cnt++;
        }
    }
    else
    {
        /* 未检测到 → 计数器递减（下限0） */
        if(black_cnt > 0)
        {
            black_cnt--;
        }
    }

    /* ---- 3. 根据计数器更新状态 ---- */
    if(black_cnt >= 2)
    {
        black_state = 1;   /* 连续检测到1次以上 → 确认有黑线 */
    }
    else if(black_cnt == 0)
    {
        black_state = 0;   /* 连续未检测到 → 确认无黑线 */
    }

    /* ---- 4. 返回检测结果 ---- */
    return black_state;
}
