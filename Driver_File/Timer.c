#include "Timer.h"


/**
  * 函    数：定时中断初始化
  * 参    数：无
  * 返 回 值：无
  */
void Timer_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(KEY_TIM_RCC, ENABLE);			//开启TIM1的时钟
	
	/*配置时钟源*/
	TIM_InternalClockConfig(KEY_TIM);		//选择TIM1为内部时钟，若不调用此函数，TIM默认也为内部时钟
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = KEY_TIM_ARR;             //ARR=99, PWM基频20kHz
	TIM_TimeBaseInitStructure.TIM_Prescaler = KEY_TIM_PSC;          //PSC=35, 计数时钟2MHz
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = KEY_TIM_RCR;  //RCR=19, 更新中断周期1ms
	TIM_TimeBaseInit(KEY_TIM, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM1的时基单元
	
	/*中断输出配置*/
	TIM_ClearFlag(KEY_TIM, TIM_FLAG_Update);			//清除定时器更新标志位
	
	TIM_ITConfig(KEY_TIM, TIM_IT_Update, ENABLE);		//开启TIM1的更新中断                                 
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;						//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = KEY_TIM_IRQC;          //选择配置NVIC的TIM1_UP线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //指定NVIC线路的抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);                             //将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*TIM使能*/
	TIM_Cmd(KEY_TIM, ENABLE);			//使能TIM1，定时器开始运行
}



/* 定时器中断函数，可以复制到使用它的地方
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
*/
