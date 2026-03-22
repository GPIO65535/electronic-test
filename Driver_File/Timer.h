#ifndef __TIMER_H
#define __TIMER_H

/*头文件引用*/
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
/*参数定义*/
#define KEY_TIM_RCC		RCC_APB2Periph_TIM1
#define KEY_TIM      	TIM1
#define KEY_TIM_IRQC	TIM1_UP_IRQn

/* TIM1共享给PA8(PWM)和1ms中断：
	72MHz / 36 = 2MHz, ARR=100-1 -> 20kHz PWM
	RCR=20-1 -> 每20个更新事件触发一次中断，即1ms */
#define KEY_TIM_PSC		36-1
#define KEY_TIM_ARR		100-1
#define KEY_TIM_RCR		20-1
/*函数声明*/
void Timer_Init(void);

#endif
