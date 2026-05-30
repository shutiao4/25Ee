//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其他用途
//////////////////////////////////////////////////////////////////////////////////
#include "sensor.h"

/*************************************
*函数名称：SENSOR_GPIO_Config
*函数功能：GPIO管脚的配置
*参数：
*说明：
*			
**************************************/
void SENSOR_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	//8路数字口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//数字上拉输入引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*************************************
*函数名称：digtal
*函数功能：获取X通道数字值
*参数：
*说明：
*			
**************************************/
unsigned char digtal(unsigned char channel)//1-ADC_N	  获取X通道数字值
{
	u16 value = 0;
	switch(channel) 
	{
		case 1:  
			if(PAin(0) == 1) value = 1;
			else value = 0;  
			break;  
		case 2: 
			if(PAin(1) == 1) value = 1;
			else value = 0;  
			break;  
		case 3: 
			if(PAin(2) == 1) value = 1;
			else value = 0;  
			break;   
		case 4:  
			if(PAin(3) == 1) value = 1;
			else value = 0;  
			break;   
		case 5:
			if(PAin(4) == 1) value = 1;
			else value = 0;  
			break;
		case 6:  
			if(PAin(5) == 1) value = 1;
			else value = 0;  
			break;  
		case 7: 
			if(PAin(6) == 1) value = 1;
			else value = 0;  
			break;  
		case 8: 
			if(PAin(7) == 1) value = 1;
			else value = 0;  
			break;  
	}
	return value; 
}




