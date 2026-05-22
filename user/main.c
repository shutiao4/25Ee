#include "stm32f10x.h"
#include "delay.h"
#include "IMU660RA.h"
#include "OLED.h"
#include "Buzzer.h"
#include "PID.h"
#include "timer.h"
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
// 任务选择：
uint8_t task_select = 1;
// 按键扫描得到的键值
uint8_t Keynum = 0;
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

/* ==================== 状态枚举 ==================== *///任务序列
typedef enum {
    //任务1
    STATE1_Straight1,
     STATE1_Straight2,
      STATE1_Straight3,
    STATE1_STOP1,
    //任务2
    STATE2_Straight1,
    STATE2_Straight2,
    STATE2_Straight3,
    STATE2_Straight4,
    STATE2_Straight5,
    STATE2_Straight6,
    STATE2_Straight7,
    STATE2_STOP1
}
 CarState;
CarState now_state;  // 当前状态机状态

/* ==================== 函数声明 ==================== */
static void SystemClock_Config(void);
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
        now_state = STATE1_Straight1;
        OLED_Clear();
        OLED_ShowString(1, 1, "Task:1");
        OLED_ShowString(2, 1, "Yaw:");
        break;

    case 2:
        task_select = 2;
        now_state =STATE2_Straight1;
        OLED_Clear();
        OLED_ShowString(1, 1, "Task:2");
        OLED_ShowString(2, 1, "Yaw:");
        break;

    case 3:
        task_select = 3;
        Car_Set_Straight_Target(0.0f);
        // now_state =;
        OLED_Clear();
        OLED_ShowString(1, 1, "Task:3");
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
        //30=47.6cm/s；
        //35=61.0cm/s；
        //40=72.3cm/s
        
     switch (now_state)
        {
        case STATE1_Straight1:
            Car_Go_Straight_To_Target(35, 1.0f);
            if(!timer_active) Timer_Start(3442);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到
                now_state = STATE1_Straight2; 
            }
            break;
        case STATE1_Straight2:
            Car_Go_Straight_To_Target(35, -45.0f);
            if(!timer_active) Timer_Start(500);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到
                now_state = STATE1_Straight3; 
            }
            break;
        case STATE1_Straight3:
            Car_Go_Straight_To_Target(35, 0.0f);
            if(!timer_active) Timer_Start(1300);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到
                now_state = STATE1_STOP1; 
            }
            break;
        case STATE1_STOP1:
            Buzzer_Beep(100);
            Set_PWM(0, 0);
            car_state = 0x0000;
            break;
        }
        break;
    /* ---------- 任务2 ---------- */
    case 2:
        switch (now_state)
        {
         case STATE2_Straight1:
            Car_Go_Straight_To_Target(35, 0.0f);
            if(!timer_active) Timer_Start(1800);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到
                now_state = STATE2_Straight2; 
            }
            break;
        case STATE2_Straight2:
            Car_Go_Straight_To_Target(35, -45.0f);
            if(!timer_active) Timer_Start(719);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到
                now_state = STATE2_Straight3; 
            }
            break;
        case STATE2_Straight3:
            Car_Go_Straight_To_Target(35, 0.0f);
            if(!timer_active) Timer_Start(719);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到
                now_state = STATE2_Straight4; 
            }
            break;
        case STATE2_Straight4:
            Car_Go_Straight_To_Target(35, 45.0f);   
            if(!timer_active) Timer_Start(819);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到 
                now_state = STATE2_Straight5; 
            }
            break;
        case STATE2_Straight5:
            Car_Go_Straight_To_Target(35, 0.0f);   
            if(!timer_active) Timer_Start(819);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到 
                now_state = STATE2_Straight6; 
            }
            break; 
        case STATE2_Straight6:
            Car_Go_Straight_To_Target(35, -45.0f);   
            if(!timer_active) Timer_Start(819);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到 
                now_state = STATE2_Straight7; 
            }
            break;
        case STATE2_Straight7:
            Car_Go_Straight_To_Target(35, 0.0f);   
            if(!timer_active) Timer_Start(1000);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到 
                now_state = STATE2_STOP1; 
            }
            break;
        case STATE2_STOP1:
        
            Set_PWM(0, 0);
            car_state = 0x0000;
            break;  
        }
        break;

    /* ---------- 任务3：先0°直行1.5秒 → 再20°直行直到黑线 → 循迹 → 直行 → 循迹 → 停 ---------- */
    case 3:
         //72.3cm/s
         {
         Car_Go_Straight_To_Target(35, 1.0f);
            if(!timer_active) Timer_Start(2000);   // 首次进入时启动
            if(Timer_Check()) {                  // 时间到
                car_state = 0x0000; 
                 Set_PWM(0, 0);
            }
        }
        break;
     
    }
}
