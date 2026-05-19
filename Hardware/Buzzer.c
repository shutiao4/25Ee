#include "Buzzer.h"

// 剩余鸣叫时间，单位为 ms
static uint16_t buzzer_remain_ms = 0;
// 蜂鸣器工作状态，1 表示正在鸣叫
static uint8_t buzzer_busy = 0;

void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能 GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // 配置 PB5 为推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 默认输出高电平，蜂鸣器关闭
    GPIO_SetBits(GPIOB, GPIO_Pin_5);
}

void Buzzer_Beep(uint16_t duration_ms)
{
    // 传入 0 表示立即停止蜂鸣器
    if (duration_ms == 0)
    {
        Buzzer_Stop();
        return;
    }

    // 记录本次鸣叫时间，并立刻开始发声
    buzzer_remain_ms = duration_ms;
    buzzer_busy = 1;
    GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

void Buzzer_BeepPattern(uint16_t on_time_ms, uint16_t off_time_ms, uint8_t repeat_count)
{
    // 这里先保留统一接口，当前等价于一次单响
    (void)off_time_ms;
    (void)repeat_count;
    Buzzer_Beep(on_time_ms);
}

void Buzzer_Update(uint16_t elapsed_ms)
{
    // 没有鸣叫任务时直接返回
    if (!buzzer_busy)
    {
        return;
    }

    // 时间到后关闭蜂鸣器，否则继续累减剩余时间
    if (elapsed_ms >= buzzer_remain_ms)
    {
        Buzzer_Stop();
    }
    else
    {
        buzzer_remain_ms -= elapsed_ms;
    }
}

void Buzzer_Stop(void)
{
    // 清空状态并拉高引脚，停止发声
    buzzer_busy = 0;
    buzzer_remain_ms = 0;
    GPIO_SetBits(GPIOB, GPIO_Pin_5);
}
