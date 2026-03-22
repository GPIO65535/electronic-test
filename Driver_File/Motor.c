#include "Motor.h"



/**
  * 函    数：直流电机初始化
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(AIN1_RCC|AIN2_RCC, ENABLE);	// 开启电机控制引脚时钟
	RCC_APB2PeriphClockCmd(BIN1_RCC|BIN2_RCC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = AIN1_PIN;				// AIN1
	GPIO_Init(AIN1_PORT, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = AIN2_PIN;				// AIN2
	GPIO_Init(AIN2_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BIN1_PIN;				// BIN1
	GPIO_Init(BIN1_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BIN2_PIN;				// BIN2
	GPIO_Init(BIN2_PORT, &GPIO_InitStructure);

	/*初始化底层PWM*/
	PWM_Init();												
}


/**
  * 函    数：直流电机设置PWM
  * 参    数：n 指定左电机还是右电机，范围：1（左电机），2（右电机）
  * 参    数：PWM 要设置的PWM值，范围：-ARR~ARR（负数为反转）
  * 返 回 值：无
  * 注    意：根据实际情况更改电机转动方向
  */
void Motor_SetPWM(uint8_t n, int8_t PWM)
{
	if (n == 1)			//指定左电机
	{
		if (PWM >= 0)							//如果设置正转的PWM
		{
			GPIO_SetBits(AIN1_PORT, AIN1_PIN);		//AIN1置高电平
			GPIO_ResetBits(AIN2_PORT, AIN2_PIN);	//AIN2置低电平
			PWM1_SetCompare(PWM);					//设置PWM占空比
		}
		else									//否则，即设置反转的PWM
		{
			GPIO_ResetBits(AIN1_PORT, AIN1_PIN);	//AIN1置低电平
			GPIO_SetBits(AIN2_PORT, AIN2_PIN);		//AIN2置高电平
			PWM1_SetCompare(-PWM);					//设置PWM占空比
		}
	}
	else if (n == 2)	//指定右电机
	{
		if (PWM >= 0)							//如果设置正转的PWM
		{
			GPIO_ResetBits(BIN1_PORT, BIN1_PIN);	//BIN1置低电平
			GPIO_SetBits(BIN2_PORT, BIN2_PIN);		//BIN2置高电平
			PWM2_SetCompare(PWM);					//设置PWM占空比
		}
		else									//否则，即设置反转的PWM
		{
			GPIO_SetBits(BIN1_PORT, BIN1_PIN);		//BIN1置高电平
			GPIO_ResetBits(BIN2_PORT, BIN2_PIN);	//BIN2置低电平
			PWM2_SetCompare(-PWM);					//设置PWM占空比
		}
	}
}
