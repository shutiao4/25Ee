#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 开启时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);

	/* 初始化 GPIOB PB0、PB1 上拉输入 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 初始化 GPIOA PA11、PA12 上拉输入 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * 函    数：非阻塞式按键获取键码
  * 参    数：无
  * 返 回 值：按下按键的键码值 1/2/3/4，返回 0 代表无按键动作
  * 特    点：无死循环、无阻塞，主循环可以一直跑
  */
uint8_t Key_GetNum(void)
{
	// 静态变量：记录按键状态（防抖、按下/松开）
	static uint8_t key_state = 0;
	static uint16_t key_debounce = 0;
	
	uint8_t Keynum = 0;
	uint8_t key_now = 0;

	// 读取当前按键电平（按下=0）
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)		key_now =3 ;
	else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)	key_now =4 ;
	else if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == 0)	key_now = 1;
	else if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0)	key_now = 2;
	else													key_now = 0;

	// 非阻塞防抖 + 状态机
	switch (key_state)
	{
		case 0:	// 空闲状态
			if (key_now != 0)
			{
				key_state = 1;	// 进入防抖
				key_debounce = 20;	// 20ms防抖
			}
			break;

		case 1:	// 防抖中
			if (key_debounce > 0)
			{
				key_debounce--;
			}
			else
			{
				if (key_now != 0)	// 确认按下
				{
					Keynum = key_now;
					key_state = 2;	// 等待松开
				}
				else
				{
					key_state = 0;	// 抖动，回到空闲
				}
			}
			break;

		case 2:	// 等待松开
			if (key_now == 0)
			{
				key_state = 0;	// 松开，回到空闲
			}
			break;
	}

	return Keynum;
}
