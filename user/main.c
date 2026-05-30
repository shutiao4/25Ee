/**
 * @file    main.c
 * @brief   智能小车正方形循迹主控程序
 *
 * 【功能概述】
 *   本程序控制小车沿着黑色赛道行驶一个正方形路径。整体采用状态机设计，
 *   分四个阶段：循迹直线 → 前冲补偿 → 90°转弯 → 停止。
 *   使用 MPU6050 陀螺仪进行航向角闭环控制，结合 7 路红外传感器进行循迹。
 *
 * 【控制流程（每 20ms 执行一次）】
 *   1. 读取 MPU6050 原始数据（加速度 + 角速度）
 *   2. 通过卡尔曼滤波更新偏航角 Yaw
 *   3. 若已启动，执行状态机：
 *      - STATE4_FOLLOW  : 红外循迹直行，同时检测正方形顶点
 *      - STATE4_CURVE   : 权重保持(±8)弧形过直角弯
 *      - STATE4_STOP    : 完成 4 条边后停车
 *
 * 【硬件资源】
 *   - PA0~PA7 : 8路红外传感器数字输入（D1~D8）
 *   - PA8      : 右轮方向控制
 *   - PA9      : TIM1_CH2 — 左轮PWM
 *   - PA10     : TIM1_CH3 — 右轮PWM
 *   - PB13/PB14: 左轮方向控制
 *   - PB15     : 右轮方向控制
 *   - TIM2     : 20ms 控制节拍定时器
 *
 * @note  按 KEY 按键启动任务，完成 4 条边后自动停止
 */

/* ==================== 头文件包含 ==================== */
#include "stm32f10x.h"      /* STM32F10x 标准外设库 */
#include "delay.h"          /* 软件延时（调试用） */
#include "MPU6050.h"        /* MPU6050 陀螺仪驱动（含卡尔曼滤波、Yaw解算） */
#include "OLED.h"           /* OLED 显示屏驱动（I2C接口） */
#include "PID.h"            /* 直行/转弯 PID 控制器 + 航向角管理 */
#include "timer.h"          /* TIM2 20ms 定时器中断 */
#include "sensor.h"         /* 红外传感器数字端口 D1~D8 定义 */
#include "line.h"           /* 循迹函数 track_zhixian1、黑线检测 */

/* ==================== 外部函数声明（来源于其他模块） ==================== */
extern void        Key_Init(void);          /* 按键 GPIO 初始化 */
extern uint8_t     Key_GetNum(void);        /* 获取按键编号（0=无按键） */
extern void        Set_PWM(int V_R, int V_L); /* 设置左右轮PWM（-100~100） */
extern void        Gpio_Init(void);         /* 电机方向控制 GPIO 初始化 */
extern void        PWM_Init(uint16_t arr, uint16_t psc); /* TIM1 PWM 初始化 */
/* ==================== 全局变量 ==================== */
/**
 * @name 电机控制变量
 * @{
 */
int V_R = 0;            /**< 右轮目标PWM值（-100~100，正=前进，负=后退） */
int V_L = 0;            /**< 左轮目标PWM值（-100~100，正=前进，负=后退） */
/** @} */

/**
 * @name MPU6050 原始数据
 * @{
 */
int16_t AX, AY, AZ;     /**< 三轴加速度计原始值（需除 16384 得 g 值） */
int16_t GX, GY, GZ;     /**< 三轴陀螺仪原始值（需除 131 得 °/s）     */
/** @} */

/**
 * @name 系统状态变量
 * @{
 */
uint16_t car_state = 0x0000;    /**< 车辆运行标志：0x0000=停止，0x1000=运行中 */
uint8_t  Keynum = 0;            /**< 当前按键编号（0=无按键动作） */
static uint8_t oled_disp_counter = 0; /**< OLED刷新计数器，每10个节拍(200ms)刷新一次，
                                           减少I2C通信对20ms控制回路的干扰 */
/** @} */


/* ==================== 正方形循迹状态机 ==================== */
/**
 * @enum    CarState
 * @brief   正方形循迹任务的状态枚举
 *
 *          STATE4_FOLLOW  →  STATE4_CURVE  →  (循环3次)  →  STATE4_STOP
 *          循迹直线            弧形过弯                     停止
 */
typedef enum {
    STATE4_FOLLOW,      /**< [循迹直线] 红外传感器循迹前进，同时检测正方形顶点 */
    STATE4_CURVE,       /**< [弧线过弯] 减速并沿黑线弧线自然过弯，不依赖陀螺仪 */
    STATE4_STOP         /**< [完成停止] 四条边全部走完，停车等待下次按键 */
} CarState;

CarState now_state;     /**< 状态机当前所处状态 */

/**
 * @name 正方形任务专用变量
 * @{
 */
static uint8_t  square_edge_count = 0;  /**< 已完成的边数计数（0~4），达到4表示一圈完成 */
static int8_t   curve_direction = 0;    /**< 当前弯道方向：1=右弯，-1=左弯 */
static uint8_t  curve_exit_cnt = 0;     /**< 出弯确认计数器：连续检测到偏差在中心附近时递增 */
static uint8_t  curve_timeout = 0;      /**< 弯道超时保护：防止卡在弯道中 */
/** @} */

/* ==================== 内部函数声明 ==================== */
static void SystemClock_Config(void);       /* 系统时钟配置 */
static void Key_Scan(void);                 /* 按键扫描与任务启动 */
static void Car_Run_StateMachine(void);     /* 车辆主状态机 */
static void OLED_DisplayYaw(void);          /* OLED显示当前偏航角 */
/* ==================== 系统时钟配置 ==================== */
/**
 * @brief  配置系统时钟
 * @note   调用 SystemInit() 使用 HSI/HSE 默认时钟配置，
 *         系统时钟默认为 72MHz（取决于外部晶振和 PLL 设置）
 */
static void SystemClock_Config(void)
{
    SystemInit();
}

/* ==================== 主函数 ==================== */
/**
 * @brief  程序入口
 *
 * 【初始化流程】
 *   1. 系统时钟、OLED、MPU6050、GPIO、PWM、按键、中断分组
 *   2. 陀螺仪校准（采集零偏 + PID参数初始化）
 *   3. 启动 TIM2（20ms 周期中断）
 *
 * 【主循环（while(1)）】
 *   每 20ms 由 TIM2 中断置位 control_flag，主循环检测到后执行一次控制周期：
 *     a. 读取 MPU6050 六轴原始数据
 *     b. 基于 GZ 角速度更新卡尔曼滤波后的偏航角 Yaw
 *     c. 如果任务已启动，执行正方形循迹状态机
 *     d. 每 200ms 刷新一次 OLED 显示（降低 I2C 干扰）
 *
 * @note  初始状态下 car_state = 0x0000，状态机不运行。
 *        按下任意按键后 car_state = 0x1000，开始循迹。
 */
int main(void)
{
    /* ---- 硬件初始化 ---- */
    SystemClock_Config();               /* 系统时钟配置（72MHz） */
    OLED_Init();                        /* OLED 显示屏初始化（I2C） */
    OLED_Clear();                       /* 清屏 */
    OLED_ShowString(1, 1, "Task:1 Square"); /* 第1行：显示任务名称 */
    OLED_ShowString(2, 1, "Yaw:");      /* 第2行：显示偏航角标签 */

    MPU6050_Init();                     /* MPU6050 初始化（I2C配置+寄存器设置） */
    Gpio_Init();                        /* 电机方向控制 GPIO（PB13/14/15, PA8） */
    PWM_Init(7199, 0);                  /* PWM 初始化（TIM1_CH2=PA9, CH3=PA10） */
    Key_Init();                         /* 按键 GPIO 初始化 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); /* 中断优先级分组：2位抢占+2位子优先级 */

    IMU_Calibrate();                    /* 陀螺仪零偏校准 + PID参数初始化 */
    TIM2_Init();                        /* 启动 TIM2，开启 20ms 周期中断 */

    /* ---- 主控制循环（20ms 节拍） ---- */
    while (1)
    {
        Key_Scan();                     /* 扫描按键（非阻塞，无按键时立即返回） */

        if (control_flag)               /* TIM2 中断每 20ms 置位一次 */
        {
            control_flag = 0;           /* 清除节拍标志 */

            /* ---- 步骤1：读取 IMU 原始数据 ---- */
            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

            /* ---- 步骤2：更新滤波后的航向角 ----
             *  基于 GZ（Z轴角速度）进行积分，
             *  结合卡尔曼滤波和零速修正(ZSC)减少漂移，
             *  最终 Yaw 值通过 MPU6050_GetYaw() 获取
             */
            MPU6050_UpdateYaw_Filtered(GZ);

            /* ---- 步骤3：执行控制状态机 ----
             *  car_state & 0x1000 为真 → 任务已启动
             *  按 KEY 按键后该位被置位，运行状态机
             */
            if (car_state & 0x1000)
            {
                Car_Run_StateMachine();
            }

            /* ---- 步骤4：OLED 显示（每200ms刷新一次） ----
             *  每10个节拍(10×20ms=200ms)刷新一次OLED，
             *  避免频繁I2C通信占用总线，干扰 MPU6050 的数据读取
             */
            if (++oled_disp_counter >= 10)
            {
                oled_disp_counter = 0;
                OLED_DisplayYaw();
            }
        }
    }
}

/**
 * @brief  在 OLED 第2行显示当前偏航角
 * @note   每 200ms 调用一次，显示范围为 0~999°
 *         （-180°~+180° 范围内够用）
 */
static void OLED_DisplayYaw(void)
{
    float yaw = MPU6050_GetYaw();           /* 获取卡尔曼滤波后的偏航角 */
    uint16_t yaw_int = (uint16_t)yaw;       /* 取整数部分（丢弃小数位） */
    OLED_ShowNum(2, 6, yaw_int, 3);         /* 第2行第6列，显示3位数字 */
}


/* =================== 按键扫描与任务启动 ==================== */
/**
 * @brief  按键扫描，按下任意键启动正方形循迹任务
 *
 * 【启动流程】
 *   1. 读取按键值，无按键时直接返回
 *   2. 重置航向角基准（Car_Reset_Angle → 以当前Yaw为零点）
 *   3. 重新校准陀螺仪（IMU_Calibrate → 采集零偏 + 重置PID参数）
 *   4. 锁定当前航向为直行目标（Car_Lock_Current_Heading）
 *   5. car_state 置为 0x1000，状态机开始运行
 *   6. 初始化正方形任务所有状态变量，从 STATE4_FOLLOW 开始
 *   7. 刷新 OLED 显示
 *
 * @note  当前所有按键均启动同一个正方形循迹任务，
 *        如需多任务切换可通过 Keynum 区分不同按键
 */
static void Key_Scan(void)
{
    Keynum = Key_GetNum();              /* 读取按键编号（非阻塞） */
    if (Keynum == 0) {                  /* 无按键按下，直接返回 */
        return;
    }

    /* ---- 航向角系统重置 ---- */
    Car_Reset_Angle();                  /* 重置角度基准：当前方向设为0° */
    IMU_Calibrate();                    /* 重新采集陀螺仪零偏 + 初始化PID参数 */
    Car_Update_Angle();                 /* （占位调用）更新角度状态 */
    Car_Lock_Current_Heading();         /* 锁定当前航向为直行目标 */
    car_state = 0x1000;                 /* 允许状态机运行 */

    /* ---- 正方形循迹任务状态初始化 ---- */
    square_edge_count = 0;              /* 已走边数清零 */
    curve_direction = 0;                /* 弯道方向清零 */
    curve_exit_cnt = 0;                 /* 出弯计数器清零 */
    curve_timeout = 0;                  /* 弯道超时保护清零 */
    now_state = STATE4_FOLLOW;          /* 从循迹直线阶段开始 */

    /* ---- OLED 显示初始化 ---- */
    OLED_Clear();
    OLED_ShowString(1, 1, "Task:1 Square");
    OLED_ShowString(2, 1, "Yaw:");
}
/* ==================== 可调参数宏定义（根据赛道实际调试） ==================== */

/** @defgroup CurveParams 弧形过弯参数
 *  @{
 */
#define CURVE_SPEED         7       /**< 弯道循迹速度 (0~100)，比直行速度低 */
#define CURVE_EXIT_THRESH   2       /**< 出弯判定阈值：偏差绝对值小于此值认为已出弯 */
#define CURVE_EXIT_CONFIRM  5       /**< 出弯确认次数：连续N次偏差在阈值内才确认出弯 */
#define CURVE_TIMEOUT_MS    8000    /**< 弯道超时保护(ms)：超过此时间强制出弯 */
/** @} */

/** @defgroup CornerDetect 顶点检测参数
 *  @{
 */
#define CORNER_DEBOUNCE     2      /**< 顶点确认次数：连续N次检测到侧边黑线才确认顶点，
                                         用于防抖动误触发 */
/** @} */

/**
 * @brief  检测正方形顶点（直角拐弯处）并判断转弯方向
 *
 * 【检测原理】
 *   7路循迹传感器（D1~D8）中，D1/D2 位于车头左侧，D7/D8 位于车头右侧。
 *   当车辆沿直线行驶接近正方形顶点时，赛道会向左或向右拐弯。
 *
 *   - 线向左拐 → 车头左侧传感器(D1,D2)会持续检测到黑线 → 需左转
 *   - 线向右拐 → 车头右侧传感器(D7,D8)会持续检测到黑线 → 需右转
 *
 *   引入 CORNER_DEBOUNCE 消抖计数，连续 N 次确认后才判定为顶点，
 *   避免传感器抖动导致的误触发。
 *
 * @retval  1  右侧检测到黑线 → 右转（线向右拐）
 * @retval -1  左侧检测到黑线 → 左转（线向左拐）
 * @retval  0  未到达顶点
 *
 * @note   ！！！重要！！！：
 *         如果实际跑圈发现转弯方向反了（例如应该左转却右转了），
 *         只需将 return -1 和 return 1 的返回值交换即可。
 */
static int8_t Check_Square_Corner(void)
{
    static uint8_t left_cnt = 0;    /**< 左侧顶点确认计数器（连续检测到左侧黑线的次数） */
    static uint8_t right_cnt = 0;   /**< 右侧顶点确认计数器（连续检测到右侧黑线的次数） */

    /*
     * 判断逻辑：
     *   if(D1 && D2)  → 左前侧两个传感器同时检测到黑线 → 线向左拐
     *   if(D7 && D8)  → 右前侧两个传感器同时检测到黑线 → 线向右拐
     *   else          → 未到顶点，清除所有计数器
     */
    if(D1 && D2 && D3)                    /* 左侧D1和D2同时检测到黑线 → 前方线向左拐 */
    {
        left_cnt++;
        right_cnt = 0;              /* 一旦进入左侧逻辑，右侧计数器清零 */
        if(left_cnt >= CORNER_DEBOUNCE)  /* 连续确认次数达标 → 确认顶点 */
        {
            left_cnt = 0;
            return 1;              /* 返回左转信号 */
        }
    }
    else if(D7 && D8 && D6)               /* 右侧D7和D8同时检测到黑线 → 前方线向右拐 */
    {
        right_cnt++;
        left_cnt = 0;               /* 进入右侧逻辑，左侧计数器清零 */
        if(right_cnt >= CORNER_DEBOUNCE) /* 连续确认次数达标 → 确认顶点 */
        {
            right_cnt = 0;
            return -1;               /* 返回右转信号 */
        }
    }
    else                            /* 左右侧均未持续检测到黑线 → 未到顶点 */
    {
        left_cnt  = 0;
        right_cnt = 0;
    }
    return 0;                       /* 未到达顶点，继续循迹 */
}

/* =============车辆状态机============= */
static void Car_Run_StateMachine(void)
{
    switch (now_state)
    {
        case STATE4_FOLLOW:
            /* ---- 循迹直线 + 卡尔曼滤波 ---- */
            track_zhixian1();

            /* ---- 检测直角弯入口 ---- */
            {
                int8_t corner_dir = Check_Square_Corner();
                if (corner_dir != 0)
                {
                    /* 记录弯道方向 */
                    curve_direction = corner_dir;
                    curve_exit_cnt = 0;
                    curve_timeout = CURVE_TIMEOUT_MS / 20;

                    /* 设置权重保持(±20)：0.5s强制转向后开始检测黑线 */
                    Line_Set_Curve_Hold(curve_direction);

                    /* 不停止，自然过渡到弯道循迹 */
                    now_state = STATE4_CURVE;
                }
            }
            break;

        case STATE4_CURVE:
            /* ---- 弧形过弯：权重保持+循迹 ---- */
            track_curve(CURVE_SPEED);

            /* ---- 检测出弯 ---- */
            {
                int8_t bias = Get_Line_Bias();

                if (curve_timeout > 0) curve_timeout--;

                if (bias > -CURVE_EXIT_THRESH && bias < CURVE_EXIT_THRESH)
                {
                    curve_exit_cnt++;
                    if (curve_exit_cnt >= CURVE_EXIT_CONFIRM || curve_timeout == 0)
                    {
                        square_edge_count++;
                        if (square_edge_count >= 4)
                        {
                            Set_PWM(0, 0);
                            now_state = STATE4_STOP;
                        }
                        else
                        {
                            Line_Kalman_Reset();
                            now_state = STATE4_FOLLOW;
                        }
                    }
                }
                else
                {
                    curve_exit_cnt = 0;
                    if (curve_timeout == 0)
                    {
                        square_edge_count++;
                        if (square_edge_count >= 4)
                        {
                            Set_PWM(0, 0);
                            now_state = STATE4_STOP;
                        }
                        else
                        {
                            Line_Kalman_Reset();
                            now_state = STATE4_FOLLOW;
                        }
                    }
                }
            }
            break;

        case STATE4_STOP:
            Set_PWM(0, 0);
            car_state = 0x0000;
            break;
    }
}
