#include "IMU660RA.h"
#include "MyI2C.h"
#include "delay.h"

/* ---- 偏航角计算参数 ---- */
#define GYRO_SENSITIVITY  66.4f    //
#define DT                0.02f       // 采样周期 (秒)，20ms

static int16_t gyro_z_bias = 0;      // 陀螺仪Z轴零偏
static float   Yaw = 0.0f;          // 偏航角（度）

uint8_t imu_init_ok = 0;            // IMU660RA初始化状态标志

/* ---- 卡尔曼滤波器参数 ---- */
static float kalman_x = 0.0f;       // 滤波后的角速度值 (dps)
static float kalman_P = 0.0f;       // 误差协方差
static float kalman_Q = 0.02f;      // 过程噪声协方差
static float kalman_R = 0.5f;       // 测量噪声协方差（0.5兼顾平滑与响应，配合ZUPT抑制静态漂移）
static uint8_t kalman_first = 1;    // 首次运行标志

#define IMU660RA_DEV_ADDR      0x69
#define IMU660RA_CHIP_ID       0x00
#define IMU660RA_PWR_CONF      0x7C
#define IMU660RA_PWR_CTRL      0x7D
#define IMU660RA_INIT_CTRL     0x59
#define IMU660RA_INIT_DATA     0x5E
#define IMU660RA_INT_STA       0x21
#define IMU660RA_ACC_ADDRESS   0x0C
#define IMU660RA_GYRO_ADDRESS  0x12
#define IMU660RA_ACC_CONF      0x40
#define IMU660RA_ACC_RANGE     0x41
#define IMU660RA_GYR_CONF      0x42
#define IMU660RA_GYR_RANGE     0x43
#define IMU660RA_ACC_SAMPLE    0x02
#define IMU660RA_GYR_SAMPLE    0x01

extern const unsigned char imu660ra_config_file[8192];

static void IMU660RA_WriteReg(uint8_t reg, uint8_t data)
{
    MyI2C_Start();
    MyI2C_SendByte(IMU660RA_DEV_ADDR << 1);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}
static uint8_t IMU660RA_ReadReg(uint8_t reg)
{
    uint8_t data;

    MyI2C_Start();
    MyI2C_SendByte(IMU660RA_DEV_ADDR << 1);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte((IMU660RA_DEV_ADDR << 1) | 0x01);
    MyI2C_ReceiveAck();
    data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return data;
}
static void IMU660RA_ReadRegs(uint8_t reg, uint8_t *data, uint16_t count)
{
    uint16_t i;

    MyI2C_Start();
    MyI2C_SendByte(IMU660RA_DEV_ADDR << 1);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte((IMU660RA_DEV_ADDR << 1) | 0x01);
    MyI2C_ReceiveAck();
    for(i = 0; i < count; i++)
    {
        data[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck(i < count - 1 ? 0 : 1);
    }
    MyI2C_Stop();
}
static void IMU660RA_WriteRegs(uint8_t reg, const unsigned char *data, uint16_t count)
{
    uint16_t i;

    MyI2C_Start();
    MyI2C_SendByte(IMU660RA_DEV_ADDR << 1);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();
    for(i = 0; i < count; i++)
    {
        MyI2C_SendByte(data[i]);
        MyI2C_ReceiveAck();
    }
    MyI2C_Stop();
}
uint8_t IMU660RA_GetID(void)
{
    return IMU660RA_ReadReg(IMU660RA_CHIP_ID);
}
uint8_t IMU660RA_Init(void)
{
    uint8_t state = 0;
    uint16_t timeout = 0;

    MyI2C_Init();
    Delay_ms(20);

    while(IMU660RA_GetID() != 0x24)
    {
        Delay_ms(1);
        if(++timeout > 255)
        {
            return 1;
        }
    }

    IMU660RA_WriteReg(IMU660RA_PWR_CONF, 0x00);
    Delay_ms(10);
    IMU660RA_WriteReg(IMU660RA_INIT_CTRL, 0x00);
    IMU660RA_WriteRegs(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));
    IMU660RA_WriteReg(IMU660RA_INIT_CTRL, 0x01);
    Delay_ms(20);

    if(IMU660RA_ReadReg(IMU660RA_INT_STA) != 1)
    {
        state = 1;
    }

    IMU660RA_WriteReg(IMU660RA_PWR_CTRL, 0x0E);
    IMU660RA_WriteReg(IMU660RA_ACC_CONF, 0xA7);
    IMU660RA_WriteReg(IMU660RA_GYR_CONF, 0xA9);
    IMU660RA_WriteReg(IMU660RA_ACC_RANGE, IMU660RA_ACC_SAMPLE);
    IMU660RA_WriteReg(IMU660RA_GYR_RANGE, IMU660RA_GYR_SAMPLE);

    imu_init_ok = (state == 0) ? 1 : 0;
    return state;
}
void IMU660RA_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                      int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t acc[6];
    uint8_t gyro[6];

    IMU660RA_ReadRegs(IMU660RA_ACC_ADDRESS, acc, 6);
    IMU660RA_ReadRegs(IMU660RA_GYRO_ADDRESS, gyro, 6);

    *AccX = (int16_t)((uint16_t)acc[1] << 8 | acc[0]);
    *AccY = (int16_t)((uint16_t)acc[3] << 8 | acc[2]);
    *AccZ = (int16_t)((uint16_t)acc[5] << 8 | acc[4]);

    *GyroX = (int16_t)((uint16_t)gyro[1] << 8 | gyro[0]);
    *GyroY = (int16_t)((uint16_t)gyro[3] << 8 | gyro[2]);
    *GyroZ = (int16_t)((uint16_t)gyro[5] << 8 | gyro[4]);
}
/* ========== 卡尔曼滤波器 ========== */
/**
 * @brief  初始化卡尔曼滤波器
 * @param  Q  过程噪声协方差（默认0.02）
 *            Q越大 → 滤波器响应越快（但噪声更多）
 *            Q越小 → 更平滑（但延迟更大）
 * @param  R  测量噪声协方差（默认0.5）
 *            R越大 → 更依赖预测（更平滑）
 *            R越小 → 更依赖测量（响应更快）
 */
void IMU660RA_Kalman_Init(float Q, float R)
{
    kalman_Q = Q;
    kalman_R = R;
    kalman_x = 0.0f;
    kalman_P = 1.0f;       // 初始协方差设大，让滤波器快速收敛
    kalman_first = 1;
}

/**
 * @brief  自适应卡尔曼滤波器更新
 * @param  measurement  当前测量值（已减去零偏的角速度，单位 dps）
 * @retval 滤波后的角速度值
 *
 * 自适应策略：根据测量值大小动态调整过程噪声 Q
 *   - |角速度| < 5°/s：Q = 0.02（正常值，平滑为主）
 *   - |角速度| ≥ 5°/s 且 < 30°/s：Q = 0.1（中等响应）
 *   - |角速度| ≥ 30°/s：Q = 0.5（急转弯，快速跟踪）
 */
static void Kalman_SetAdaptiveQ(float raw_dps)
{
    float abs_dps = raw_dps;
    if(abs_dps < 0.0f) abs_dps = -abs_dps;

    if(abs_dps < 5.0f)
        kalman_Q = 0.02f;       // 静止/慢速 → 平滑优先
    else if(abs_dps < 30.0f)
        kalman_Q = 0.1f;        // 中速转弯 → 平衡
    else
        kalman_Q = 0.5f;        // 急转弯 → 快速跟踪，减小滞后
}

static float Kalman_Update(float measurement)
{
    float K;

    if(kalman_first)
    {
        kalman_x = measurement;     // 第一次直接用测量值初始化
        kalman_first = 0;
        return kalman_x;
    }

    /* 预测：状态不变（假设角速度变化小），协方差增加过程噪声 */
    kalman_P += kalman_Q;

    /* 更新：计算卡尔曼增益 */
    K = kalman_P / (kalman_P + kalman_R);

    /* 用测量值修正估计 */
    kalman_x += K * (measurement - kalman_x);

    /* 更新协方差 */
    kalman_P *= (1.0f - K);

    return kalman_x;
}

/* ========== 陀螺仪偏航角处理 ========== */
void IMU660RA_CalibrateGyroZ(void)
{
    int i;
    int32_t sum_gz = 0;
    int16_t GX, GY, GZ;
    int16_t AX, AY, AZ;

    /* 取100次采样平均值计算零偏 */
    for(i = 0; i < 100; i++)
    {
        IMU660RA_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        sum_gz += GZ;
    }
    gyro_z_bias = (int16_t)(sum_gz / 100);

    /* 校准完成后顺便初始化卡尔曼滤波器 */
    IMU660RA_Kalman_Init(0.02f, 0.5f);
}

/* ---- 零速修正参数 ---- */
/* 阈值0.6°/s：过滤电机振动噪声（约0.3°/s），同时允许低速转弯（>0.6°/s）正常积分 */
/* 连续5次（100ms）判断，避免转弯减速时频繁误触发冻结 */
#define ZUPT_THRESHOLD_DPS    0.6f     // 静止判定阈值 (°/s)
#define ZUPT_COUNT_THRESHOLD  5        // 连续静止次数（5次×20ms=100ms）
static uint8_t zupt_count = 0;         // 当前连续静止计数值
static float  bias_correction = 0.0f;  // 零偏修正累积量（模块级，可被静止/运动切换重置）
static uint8_t was_stationary = 0;     // 上一周期是否静止

/* ---- 急转弯辅助参数 ---- */
/* 急转弯判定阈值：角速度 > 30°/s 视为急转弯 */
#define TURN_HIGH_SPEED_THRESHOLD  30.0f    // 急转弯判定阈值 (°/s)
/* 急转弯后的ZUPT冷却周期数：急转弯后500ms（25个周期）内禁止ZUPT冻结角度 */
#define ZUPT_COOLDOWN_CYCLES      25        // 冷却周期数
static uint8_t turn_cooldown = 0;           // 转弯冷却计数器（>0时禁用ZUPT冻结）
static uint8_t was_turning = 0;             // 上一周期是否处于急转弯状态

/**
 * @brief  更新偏航角（自适应卡尔曼滤波 + 急转弯检测 + ZUPT冷却）
 * @param  GZ  陀螺仪Z轴原始值
 *
 * 工作原理：
 * 1. 减去零偏 → 原始角速度 raw_dps
 * 2. 自适应卡尔曼滤波（根据角速度大小动态调整Q值）
 * 3. 急转弯检测：raw_dps > 30°/s → 标记转弯状态，启动ZUPT冷却
 * 4. ZUPT判定（仅在冷却结束后才允许冻结角度）：
 *    - 连续多次低于阈值 → 判定静止 → 冻结角度 + 自适应零偏修正
 *    - 急转弯后500ms内禁止冻结，确保减速阶段正常积分
 * 5. 偏置修正：静止时用 filtered_dps 以10%速率跟踪零偏漂移
 * 6. 从静止到运动时清零修正累积，避免错误偏置带入运动中
 */
void IMU660RA_UpdateYaw_Filtered(int16_t GZ)
{
    float raw_dps;          // 原始角速度 (dps)
    float filtered_dps;     // 卡尔曼滤波后的角速度 (dps)
    float abs_raw_dps;      // 原始角速度绝对值

    /* 1. 减去零偏，转为角速度（度/秒） */
    raw_dps = (float)(GZ - gyro_z_bias) / GYRO_SENSITIVITY;

    /* 2. 自适应卡尔曼滤波：先根据原始角速度大小调整Q值，再滤波 */
    Kalman_SetAdaptiveQ(raw_dps);
    filtered_dps = Kalman_Update(raw_dps);

    /* 3. 急转弯检测和冷却管理 */
    abs_raw_dps = raw_dps;
    if(abs_raw_dps < 0.0f) abs_raw_dps = -abs_raw_dps;

    if(abs_raw_dps >= TURN_HIGH_SPEED_THRESHOLD)
    {
        /* 当前处于急转弯状态 → 标记并启动冷却 */
        was_turning = 1;
        turn_cooldown = ZUPT_COOLDOWN_CYCLES;  // 启动冷却计时器
    }
    else if(turn_cooldown > 0)
    {
        /* 急转弯结束但仍在冷却期 → 递减冷却计数器 */
        turn_cooldown--;
    }
    else
    {
        /* 冷却结束 → 清除转弯标志 */
        was_turning = 0;
    }

    /* 4. ZUPT判定：仅在冷却结束后（即非急转弯后减速期）才允许冻结角度 */
    if(turn_cooldown == 0)
    {
        /* 冷却结束 → 正常ZUPT逻辑 */
        if(filtered_dps > -ZUPT_THRESHOLD_DPS && filtered_dps < ZUPT_THRESHOLD_DPS)
        {
            /* 角速度很小 → 可能是静止，计数器累加 */
            if(zupt_count < ZUPT_COUNT_THRESHOLD)
                zupt_count++;

            if(zupt_count >= ZUPT_COUNT_THRESHOLD)
            {
                /* === 已判定为静止 === */
                was_stationary = 1;

                /* 用滤波后的角速度做偏置修正（更平滑，避免噪声干扰） */
                bias_correction += filtered_dps * 0.1f;
                if(bias_correction > 0.25f)
                {
                    gyro_z_bias += 1;
                    bias_correction = 0.0f;
                }
                else if(bias_correction < -0.25f)
                {
                    gyro_z_bias -= 1;
                    bias_correction = 0.0f;
                }

                return;  // 冻结角度，不积分
            }
            /* 未达到连续阈值，继续积分（避免短暂停顿误冻结） */
        }
        else
        {
            /* 角速度超过阈值 → 运动状态，清零计数器 */
            zupt_count = 0;

            /* 从静止切换到运动时，重置偏置修正累积量 */
            if(was_stationary)
            {
                was_stationary = 0;
                bias_correction = 0.0f;
            }
        }
    }
    else
    {
        /* 冷却期中：不进行ZUPT判定，但需要管理静止计数器 */
        /* 如果角速度已经降到阈值以下，不累积zupt_count */
        /* 如果角速度依然超过阈值，确保zupt_count清零 */
        if(filtered_dps > -ZUPT_THRESHOLD_DPS && filtered_dps < ZUPT_THRESHOLD_DPS)
        {
            /* 冷却期中角速度很低但不冻结，只是不累加zupt_count */
            /* 这样冷却结束后可以快速进入静止判定 */
            /* 但为了防止急转弯减速过程中的"假静止"累积，不增加zupt_count */
        }
        else
        {
            zupt_count = 0;  // 冷却期中仍有明显角速度 → 确保zupt_count清零
        }

        /* 如果之前在静止状态，冷却期中离开静止 */
        if(was_stationary)
        {
            was_stationary = 0;
            bias_correction = 0.0f;
        }
    }

    /* 5. 未冻结 → 用滤波后的角速度积分 */
    Yaw += filtered_dps * DT;

    /* 6. 归一化到 -180° ~ +180° */
    if(Yaw > 91.0f)   Yaw -= 180.0f;
    if(Yaw <= -91.0f) Yaw += 180.0f;
}

/**
 * @brief  更新偏航角（不使用滤波，直接积分，保留原行为）
 * @param  GZ  陀螺仪Z轴原始值
 */
void IMU660RA_UpdateYaw(int16_t GZ)
{
    Yaw += (float)(GZ - gyro_z_bias) / GYRO_SENSITIVITY * DT;

    if(Yaw > 91.0f)   Yaw -= 180.0f;
    if(Yaw <= -91.0f) Yaw += 180.0f;
}

float IMU660RA_GetYaw(void)
{
    return Yaw;
}

int16_t IMU660RA_GetGyroZBias(void)
{
    return gyro_z_bias;
}