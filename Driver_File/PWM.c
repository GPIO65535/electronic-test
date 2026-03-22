#include "PWM.h"

/**
  * 函    数：PWM初始化
  * 参    数：无
  * 返 回 值：无
  */
void PWM_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(PWM1_TIM_RCC, ENABLE);				//开启TIM2时钟（PA1）
	RCC_APB2PeriphClockCmd(PWM_GPIO_RCC | PWM2_TIM_RCC, ENABLE);	//开启GPIOA和TIM1时钟（PA8）

	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = PWM1_PIN;						//PA1
	GPIO_Init(PWM1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PWM2_PIN;						//PA8
	GPIO_Init(PWM2_PORT, &GPIO_InitStructure);

	/*TIM2时基单元初始化（PA1: TIM2_CH2）*/
	TIM_InternalClockConfig(PWM1_TIM);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = PWM_ARR;
	TIM_TimeBaseInitStructure.TIM_Prescaler = PWM_PSC;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(PWM1_TIM, &TIM_TimeBaseInitStructure);

	/*输出比较初始化*/
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	/*PA1: TIM2_CH2*/
	TIM_OC2Init(PWM1_TIM, &TIM_OCInitStructure);

	/*PA8: TIM1_CH1
	  TIM1时基由Timer_Init统一配置，避免与定时中断配置冲突 */
	TIM_OC1Init(PWM2_TIM, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(PWM2_TIM, ENABLE);							//高级定时器主输出使能

	/*TIM使能*/
	TIM_Cmd(PWM1_TIM, ENABLE);
	TIM_Cmd(PWM2_TIM, ENABLE);
}

/**
  * 函    数：PWM1设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~ARR+1
  * 返 回 值：无
	* 注意事项：CCR2和ARR共同决定占空比，此函数仅设置CCR2的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void PWM1_SetCompare(uint16_t Compare)
{
	TIM_SetCompare2(PWM1_TIM, Compare);		//设置TIM2 CCR2（PA1）
}

/**
  * 函    数：PWM2设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~ARR+1
  * 返 回 值：无
	* 注意事项：CCR1和ARR共同决定占空比，此函数仅设置CCR1的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void PWM2_SetCompare(uint16_t Compare)
{
	TIM_SetCompare1(PWM2_TIM, Compare);		//设置TIM1 CCR1（PA8）
}
