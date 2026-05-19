/**
 * @file    main.c
 * @brief   主程序：IMU660RA 小车控制——初始化、按键扫描、状态机运行
 * @note    基于 20ms 定时器节拍，在 Car_Run_StateMachine 中实现四个任务
 *          优化版本：移除未定义引用、统一校准接口、精简主循环
 */
 
#include "stm32f10x.h"
#include "delay.h"
#include "IMU660RA.h"
#include "OLED.h"
#include "Buzzer.h"
#include "PID.h"

/* ==================== 外部函数声明（来源于其他模块） ==================== */
extern void        Key_Init(void);
extern uint8_t     Key_GetNum(void);
extern void        Set_PWM(int V_R, int V_L);
extern void        Gpio_Init(void);
extern void        PWM_Init(uint16_t arr, uint16_t psc);
extern uint8_t     Check_BlackLine(void);
extern void        track_zhixian1(void);
extern void        Buzzer_Beep(uint16_t ms);
extern void        Buzzer_Update(uint16_t period_ms);

/* ==================== 全局变量 ==================== */

// 电机PWM输出值，供外部模块链接
int V_R = 0;
int V_L = 0;

// IMU660RA 原始6轴数据
int16_t AX, AY, AZ, GX, GY, GZ;

// 车辆状态：0x1000 = 运行，0x0000 = 停止
uint16_t car_state = 0x0000;

// 任务选择：1~4 分别对应四个任务
uint8_t task_select = 1;

// 按键扫描得到的键值
uint8_t Keynum = 0;

// 20ms 控制节拍标志（在 TIM2 中断中置1）
volatile uint8_t control_flag = 0;

// OLED显示节拍计数器：每10个节拍（200ms）刷新一次，减少I2C对控制回路的干扰
static uint8_t oled_disp_counter = 0;

// 任务4使用的圈数计数器
uint8_t circle_count = 0;

/* ==================== 定时器辅助（用于定时直行等） ==================== */
/*
 * 使用方法（以定时直行1.5秒为例）：
 *
 *   case MY_STATE:
 *       Car_Go_Straight_To_Target(40, 0.0f);
 *       if(!timer_active) Timer_Start(1500);   // 首次进入时启动
 *       if(Timer_Check()) {                     // 时间到
 *           now_state = MY_NEXT_STATE;
 *       }
 *       break;
 *
 * 定时+黑线混合：
 *   if(Timer_Check() || Check_BlackLine()) { ... }
 */
static uint16_t task_timer = 0;       // 定时计数器（单位：20ms）
static uint8_t  timer_active = 0;     // 定时器是否激活

// 启动定时器，ms 按20ms对齐向上取整
static void Timer_Start(uint16_t ms)
{
    task_timer = (ms + 19) / 20;
    timer_active = 1;
}

// 检查定时器是否到期（每次20ms周期调用一次）
// 返回 1 = 时间到（自动停止定时器），0 = 还在计时中
static uint8_t Timer_Check(void)
{
    if(!timer_active) return 0;
    if(task_timer > 0)
    {
        task_timer--;
        if(task_timer == 0)
        {
            timer_active = 0;
            return 1;
        }
    }
    return 0;
}

/* ==================== 状态枚举 ==================== */
typedef enum {
    /* 任务2状态序列 */
    STATE2_STRAIGHT1,   // 第一段直行
    STATE2_TRACKING1,   // 第一段循迹
    STATE2_STRAIGHT2,   // 第二段直行
    STATE2_TRACKING2,   // 第二段循迹
    STATE2_STOP1,       // 停止

    /* 任务3状态序列 */
    STATE3_STRAIGHT1A,  // 第一段直行：以0°目标直行1.5秒
    STATE3_STRAIGHT1B,  // 第一段直行：以20°目标直行直到黑线
    STATE3_TRACKING1,   // 第一段循迹
    STATE3_TRACKING2,   // 第二段循迹
    STATE3_STRAIGHT2A,   // 第二段直行
    STATE3_STRAIGHT2B,   // 第二段直行
    STATE3_STOP1,       // 停止

    /* 任务4状态序列：任务3重复4圈，逐圈展开 */
    // 第1圈
    STATE4_C1_STRAIGHT1A,
    STATE4_C1_STRAIGHT1B,
    STATE4_C1_TRACKING1,
    STATE4_C1_STRAIGHT2A,
    STATE4_C1_STRAIGHT2B,
    STATE4_C1_TRACKING2,
    // 第2圈
    STATE4_C2_STRAIGHT1A,
    STATE4_C2_STRAIGHT1B,
    STATE4_C2_TRACKING1,
    STATE4_C2_STRAIGHT2A,
    STATE4_C2_STRAIGHT2B,
    STATE4_C2_TRACKING2,
    // 第3圈
    STATE4_C3_STRAIGHT1A,
    STATE4_C3_STRAIGHT1B,
    STATE4_C3_TRACKING1,
    STATE4_C3_STRAIGHT2A,
    STATE4_C3_STRAIGHT2B,
    STATE4_C3_TRACKING2,
    // 第4圈
    STATE4_C4_STRAIGHT1A,
    STATE4_C4_STRAIGHT1B,
    STATE4_C4_TRACKING1,
    STATE4_C4_STRAIGHT2A,
    STATE4_C4_STRAIGHT2B,
    STATE4_C4_TRACKING2,
    STATE4_STOP1,
} CarState;

CarState now_state;  // 当前状态机状态

/* ==================== 函数声明 ==================== */
static void SystemClock_Config(void);
static void TIM2_Init(void);
static void Key_Scan(void);
static void Car_Run_StateMachine(void);
static void OLED_DisplayYaw(void);

/* ==================== 系统初始化 ==================== */
/**
 * @brief  配置系统时钟为72MHz
 */
static void SystemClock_Config(void)
{
    SystemInit();
}

/* ==================== 主函数 ==================== */
int main(void)
{
    /* ---- 系统初始化 ---- */
    SystemClock_Config();

    /* OLED先初始化并显示内容，再执行耗时的IMU初始化 */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Task:1");
    OLED_ShowString(2, 1, "Yaw:");

     IMU660RA_Init();
    Gpio_Init();
    PWM_Init(7199, 0);
    Key_Init();
    Buzzer_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* ---- 陀螺仪校准 + PID初始化 ---- */
    IMU660ra_Calibrate();

    /* ---- 启动20ms定时器中断 ---- */
    TIM2_Init();

    /* ---- 主循环 ---- */
    while (1)
    {
        Key_Scan();

        if (control_flag)
        {
            control_flag = 0;
            Buzzer_Update(20);

            // 读取IMU原始数据
            IMU660RA_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

            // 更新偏航角（基于GZ积分 + 卡尔曼滤波 + 零速修正）
            IMU660RA_UpdateYaw_Filtered(GZ);

            // 运行状态机
            if (car_state & 0x1000)
            {
                Car_Run_StateMachine();
            }
            else
            {
                Set_PWM(0, 0);
            }

            // 每10个节拍（200ms）刷新一次OLED，减少I2C对控制回路的干扰
            if (++oled_disp_counter >= 10)
            {
                oled_disp_counter = 0;
                OLED_DisplayYaw();
            }
        }
    }
}

/**
 * @brief  在OLED上显示偏航角（第2行，从第6列开始）
 */
static void OLED_DisplayYaw(void)
{
    float yaw = IMU660RA_GetYaw();
    int16_t yaw_int = (int16_t)yaw;
    OLED_ShowSignedNum(2, 6, yaw_int, 4);
}

/* ============================================================
 *  按键扫描
 * ============================================================ */
static void Key_Scan(void)
{
    Keynum = Key_GetNum();
    if (Keynum == 0) {
        return;
    }

    // 通用初始化操作
    Buzzer_Beep(100);
    Car_Reset_Angle();
    IMU660ra_Calibrate();
    Car_Update_Angle();
    Car_Lock_Current_Heading();
    circle_count = 0;
    timer_active = 0;       // 重置定时器
    car_state = 0x1000;

    switch (Keynum)
    {
    case 1:
        task_select = 1;
        OLED_Clear();
        OLED_ShowString(1, 1, "Task:1");
        OLED_ShowString(2, 1, "Yaw:");
        break;

    case 2:
        task_select = 2;
        now_state = STATE2_STRAIGHT1;
        OLED_Clear();
        OLED_ShowString(1, 1, "Task:2");
        OLED_ShowString(2, 1, "Yaw:");
        break;

    case 3:
        task_select = 3;
        Car_Set_Straight_Target(0.0f);
        now_state = STATE3_STRAIGHT1A;
        OLED_Clear();
        OLED_ShowString(1, 1, "Task:3");
        OLED_ShowString(2, 1, "Yaw:");
        break;

    case 4:
        task_select = 4;
        Car_Set_Straight_Target(0.0f);
        now_state = STATE4_C1_STRAIGHT1A;
        OLED_Clear();
        OLED_ShowString(1, 1, "Task:4");
        OLED_ShowString(2, 1, "Yaw:");
        break;

    default:
        break;
    }
}

/* ============================================================
 *  车辆状态机
 * ============================================================ */
static void Car_Run_StateMachine(void)
{
    switch (task_select)
    {
    /* ---------- 任务1：直行3秒+黑线检测后停车 ---------- */
    case 1:
     //47.6cm/s
        Car_Go_Straight_To_Target(30, 0.0f);
         if(!timer_active) Timer_Start(2100);   // 首次进入时启动
         if(Timer_Check()) {                     // 时间到
                Buzzer_Beep(100);
                Set_PWM(0, 0);
                car_state = 0x0000; 
         }
        break;

    /* ---------- 任务2 ---------- */
    case 2:
        switch (now_state)
        {
        case STATE2_STRAIGHT1:
            Car_Go_Straight_To_Target(40, 0.0f);
            if (Check_BlackLine() == 1) {
                Buzzer_Beep(100);
                now_state = STATE2_TRACKING1;
            }
            break;

        case STATE2_TRACKING1:
           track_zhixian1();
            if (Check_BlackLine() == 0) {
              Buzzer_Beep(100);
                now_state = STATE2_STRAIGHT2;
            }
            break;
      
        case STATE2_STRAIGHT2:
           Car_Go_Straight_To_Target(40, 89.0f);
            if (Check_BlackLine() == 1) {   
                Buzzer_Beep(100);
                now_state = STATE2_TRACKING2;
            }
            break;

        case STATE2_TRACKING2:
         track_zhixian1(); 
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE2_STOP1;
            }
            break;

        case STATE2_STOP1:
            Buzzer_Beep(100);
            Set_PWM(0, 0);
            car_state = 0x0000;
            break;
        }
        break;

    /* ---------- 任务3：先0°直行1.5秒 → 再20°直行直到黑线 → 循迹 → 直行 → 循迹 → 停 ---------- */
    case 3:
        switch (now_state)
        {
        case STATE3_STRAIGHT1A:
            Car_Go_Straight_To_Target(40, -27.0f);   // 先以0°目标直行
            if(!timer_active) Timer_Start(237);    // 启动1.5秒定时
            if(Timer_Check()) {                     // 1.5秒到
                timer_active = 0;
                now_state = STATE3_STRAIGHT1B;
            }
            break;

        case STATE3_STRAIGHT1B:
            Car_Go_Straight_To_Target(40, 0.0f);  // 再以20°目标直行
            if(Check_BlackLine() == 1) {            // 检测到黑线
                Buzzer_Beep(100);
                now_state = STATE3_TRACKING1;
            }
            break;

        case STATE3_TRACKING1:
              track_zhixian1(); 
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE3_STRAIGHT2A;
            }
            break;

        case STATE3_STRAIGHT2A:
             Car_Go_Straight_To_Target(40, 120.0f);   // 先以0°目标直行
            if(!timer_active) Timer_Start(1370);    // 启动1.5秒定时
            if(Timer_Check()) {                     // 1.5秒到
                timer_active = 0;
                now_state = STATE3_STRAIGHT2B;
            }
            break;

        case STATE3_STRAIGHT2B:
            Car_Go_Straight_To_Target(40, 90.0f);
            if(Check_BlackLine() == 1) {
                Buzzer_Beep(100);
                now_state = STATE3_TRACKING2;
            }
            break;

        case STATE3_TRACKING2:
           track_zhixian1(); 
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE3_STOP1;
            }
            break;

        case STATE3_STOP1:
            Set_PWM(0, 0);
            car_state = 0x0000;
            break;
        }
        break;

    /* ---------- 任务4：任务3重复4圈，逐圈展开 ---------- */
    case 4:
        switch (now_state)
        {
        /* ==== 第1圈 ==== */
        case STATE4_C1_STRAIGHT1A:
            Car_Go_Straight_To_Target(30, -22.5f);
            if(!timer_active) Timer_Start(2450);//1.8秒
            if(Timer_Check()) {
                timer_active = 0;
                now_state = STATE4_C1_STRAIGHT1B;
            }
            break;

        case STATE4_C1_STRAIGHT1B:
            Car_Go_Straight_To_Target(30, 0.0f);
            if(Check_BlackLine() == 1) {
                Buzzer_Beep(100);
                now_state = STATE4_C1_TRACKING1;
            }
            break;

        case STATE4_C1_TRACKING1:
            track_zhixian1();
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE4_C1_STRAIGHT2A;
            }
            break;

        case STATE4_C1_STRAIGHT2A:
            Car_Go_Straight_To_Target(30, 112.5f);
            if(!timer_active) Timer_Start(2200);
            if(Timer_Check()) {
                timer_active = 0;
                now_state = STATE4_C1_STRAIGHT2B;
            }
            break;

        case STATE4_C1_STRAIGHT2B:
            Car_Go_Straight_To_Target(30, 90.0f);
            if(Check_BlackLine() == 1) {
                Buzzer_Beep(100);
                now_state = STATE4_C1_TRACKING2;
            }
            break;

        case STATE4_C1_TRACKING2:
            track_zhixian1();
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE4_C2_STRAIGHT1A;
            }
            break;

        /* ==== 第2圈 ==== */
        case STATE4_C2_STRAIGHT1A:
            Car_Go_Straight_To_Target(30, -22.5f);
            if(!timer_active) Timer_Start(2200);
            if(Timer_Check()) {
                timer_active = 0;
                now_state = STATE4_C2_STRAIGHT1B;
            }
            break;

        case STATE4_C2_STRAIGHT1B:
            Car_Go_Straight_To_Target(30, 0.0f);
            if(Check_BlackLine() == 1) {
                Buzzer_Beep(100);
                now_state = STATE4_C2_TRACKING1;
            }
            break;

        case STATE4_C2_TRACKING1:
            track_zhixian1();
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE4_C2_STRAIGHT2A;
            }
            break;

        case STATE4_C2_STRAIGHT2A:
            Car_Go_Straight_To_Target(30, 112.5f);
            if(!timer_active) Timer_Start(2200);
            if(Timer_Check()) {
                timer_active = 0;
                now_state = STATE4_C2_STRAIGHT2B;
            }
            break;

        case STATE4_C2_STRAIGHT2B:
            Car_Go_Straight_To_Target(30, 90.0f);
            if(Check_BlackLine() == 1) {
                Buzzer_Beep(100);
                now_state = STATE4_C2_TRACKING2;
            }
            break;

        case STATE4_C2_TRACKING2:
            track_zhixian1();
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE4_C3_STRAIGHT1A;
            }
            break;

        /* ==== 第3圈 ==== */
        case STATE4_C3_STRAIGHT1A:
            Car_Go_Straight_To_Target(30, -22.5f);
            if(!timer_active) Timer_Start(2200);
            if(Timer_Check()) {
                timer_active = 0;
                now_state = STATE4_C3_STRAIGHT1B;
            }
            break;

        case STATE4_C3_STRAIGHT1B:
            Car_Go_Straight_To_Target(30, 0.0f);
            if(Check_BlackLine() == 1) {
                Buzzer_Beep(100);
                now_state = STATE4_C3_TRACKING1;
            }
            break;

        case STATE4_C3_TRACKING1:
            track_zhixian1();
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE4_C3_STRAIGHT2A;
            }
            break;

        case STATE4_C3_STRAIGHT2A:
            Car_Go_Straight_To_Target(30, 112.5f);
            if(!timer_active) Timer_Start(2200);
            if(Timer_Check()) {
                timer_active = 0;
                now_state = STATE4_C3_STRAIGHT2B;
            }
            break;

        case STATE4_C3_STRAIGHT2B:
            Car_Go_Straight_To_Target(30, 90.0f);
            if(Check_BlackLine() == 1) {
                Buzzer_Beep(100);
                now_state = STATE4_C3_TRACKING2;
            }
            break;

        case STATE4_C3_TRACKING2:
            track_zhixian1();
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE4_C4_STRAIGHT1A;
            }
            break;

        /* ==== 第4圈 ==== */
        case STATE4_C4_STRAIGHT1A:
            Car_Go_Straight_To_Target(30, -22.5f);
            if(!timer_active) Timer_Start(2200);
            if(Timer_Check()) {
                timer_active = 0;
                now_state = STATE4_C4_STRAIGHT1B;
            }
            break;

        case STATE4_C4_STRAIGHT1B:
            Car_Go_Straight_To_Target(30, 0.0f);
            if(Check_BlackLine() == 1) {
                Buzzer_Beep(100);
                now_state = STATE4_C4_TRACKING1;
            }
            break;

        case STATE4_C4_TRACKING1:
            track_zhixian1();
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE4_C4_STRAIGHT2A;
            }
            break;

        case STATE4_C4_STRAIGHT2A:
            Car_Go_Straight_To_Target(30, 112.5f);
            if(!timer_active) Timer_Start(2200);
            if(Timer_Check()) {
                timer_active = 0;
                now_state = STATE4_C4_STRAIGHT2B;
            }
            break;

        case STATE4_C4_STRAIGHT2B:
            Car_Go_Straight_To_Target(30, 90.0f);
            if(Check_BlackLine() == 1) {
                Buzzer_Beep(100);
                now_state = STATE4_C4_TRACKING2;
            }
            break;

        case STATE4_C4_TRACKING2:
            track_zhixian1();
            if (Check_BlackLine() == 0) {
                Buzzer_Beep(100);
                now_state = STATE4_STOP1;
            }
            break;

        case STATE4_STOP1:
            Set_PWM(0, 0);
            car_state = 0x0000;
            break;

        default:
            now_state = STATE4_C1_STRAIGHT1A;
            break;
        }
        break;

    default:
        break;
    }
}

/* ============================================================
 *  TIM2 初始化：20ms中断
 * ============================================================ */
static void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* 定时周期 = (Prescaler + 1)*(Period + 1) / 72MHz = 20ms */
    TIM_TimeBaseStructure.TIM_Period        = 199;
    TIM_TimeBaseStructure.TIM_Prescaler     = 7199;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ============================================================
 *  TIM2 中断服务函数：每20ms置位 control_flag
 * ============================================================ */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        control_flag = 1;
    }
}
