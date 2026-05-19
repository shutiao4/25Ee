#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32f10x.h"

// 初始化蜂鸣器GPIO，默认关闭
void Buzzer_Init(void);
// 单次鸣叫，参数为持续时间(ms)
void Buzzer_Beep(uint16_t duration_ms);
// 预留的多次鸣叫接口，当前实现为一次单响
void Buzzer_BeepPattern(uint16_t on_time_ms, uint16_t off_time_ms, uint8_t repeat_count);
// 周期性更新时间基，通常每20ms调用一次
void Buzzer_Update(uint16_t elapsed_ms);
// 立即关闭蜂鸣器
void Buzzer_Stop(void);

#endif
