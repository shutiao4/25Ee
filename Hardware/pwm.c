#include "pwm.h"

void PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;                //???????GPIO_InitStructure
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;      //???????TIM_TimeBaseStructure   
	TIM_OCInitTypeDef TIM_OCInitStructure;              //???????TIM_OCInitStructure
	
	// 1. ?????????????????????
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // ???PA??PB??????
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         // ???????????
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;        // IO?????
	
	// ????? PA9 (PWMB)
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9; 
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	// ????? PA10 (PWMA)
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10; 
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	// 3. ?????????????
	TIM_TimeBaseStructure.TIM_Period = arr;                // ?????????
	TIM_TimeBaseStructure.TIM_Prescaler = psc;             // ?????
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;           // ?????
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ???????
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	// 4. PWM??????
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;             // PWM??1
	TIM_OCInitStructure.TIM_Pulse       = 0;                           // ???????0
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;         // ????????
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;      // ??????
	

	TIM_OC2Init(TIM1,&TIM_OCInitStructure);

	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	//???????
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	
		//?????????
	TIM_Cmd(TIM1, ENABLE);
	
	

}
void Set_PWM(int V_R, int V_L)
{
	int PWMA, PWMB;

    // ?? -100~100 ???? PWM 0~7200
    PWMA =-V_L * 72;
    PWMB =-V_R * 72;

	if(PWMA>0){
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
		 GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		 	  //??? AIN1????  AIN2????  
	}else{
		 GPIO_ResetBits(GPIOB, GPIO_Pin_13);	
		GPIO_SetBits(GPIOB, GPIO_Pin_14);	 
		PWMA=-PWMA;	
		     //??? AIN1????  AIN2????  pwm??????
	}
	
	if(PWMB>0)
		{
		GPIO_SetBits(GPIOA, GPIO_Pin_8);    
		GPIO_ResetBits(GPIOB, GPIO_Pin_15); 
		//??? BIN1??????BIN2????
		}
	else
		{
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
	    //??? BIN1??????BIN2????
		PWMB=-PWMB;
		}

  TIM_SetCompare3(TIM1,PWMA);  //A???????pwm  
	TIM_SetCompare2(TIM1,PWMB);	 //B???????pwm	 
}
