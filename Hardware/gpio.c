#include "gpio.h"

void Gpio_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;            //???????GPIO_InitStructure
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOA, ENABLE); // ???PB??????  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	  //PB12 PB13 PB14 PB15
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     	//??????????????????  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //IO?????
	GPIO_Init(GPIOB, &GPIO_InitStructure);          //GBIOB?????  
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
