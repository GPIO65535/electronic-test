#ifndef __PWM_H
#define __PWM_H

/*头文件调用*/
#include "stm32f10x.h"                  // Device header
/*参数定义*/
#define PWM_ARR			100-1
#define PWM_PSC			36-1

#define PWM_GPIO_RCC	RCC_APB2Periph_GPIOA

/* PWM1: PA1 -> TIM2_CH2 */
#define PWM1_TIM		TIM2
#define PWM1_TIM_RCC	RCC_APB1Periph_TIM2
#define PWM1_PORT		GPIOA
#define PWM1_PIN		GPIO_Pin_1

/* PWM2: PA8 -> TIM1_CH1 */
#define PWM2_TIM		TIM1
#define PWM2_TIM_RCC	RCC_APB2Periph_TIM1
#define PWM2_PORT		GPIOA
#define PWM2_PIN		GPIO_Pin_8
/*函数声明*/
void PWM_Init(void);
void PWM1_SetCompare(uint16_t Compare);
void PWM2_SetCompare(uint16_t Compare);

#endif
