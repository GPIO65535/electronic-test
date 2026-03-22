#ifndef __ENCODER_H
#define __ENCODER_H

/*头文件引用*/
#include "stm32f10x.h"                  // Device header
/*参数定义*/
#define CODER1_TIM				TIM3	// 编码器1 定时器
#define CODER1_TIM_RCC			RCC_APB1Periph_TIM3
#define CODER1_A_CHANNEL		TIM_Channel_1
#define CODER1_B_CHANNEL		TIM_Channel_2
#define CODER1_GPIO_PORT		GPIOA	// 编码器1 GPIO
#define CODER1_GPIO_RCC			RCC_APB2Periph_GPIOA
#define CODER1_A_PIN			GPIO_Pin_6	
#define CODER1_B_PIN			GPIO_Pin_7

#define CODER2_TIM				TIM4	// 编码器2 定时器
#define CODER2_TIM_RCC			RCC_APB1Periph_TIM4
#define CODER2_A_CHANNEL		TIM_Channel_1
#define CODER2_B_CHANNEL		TIM_Channel_2
#define CODER2_GPIO_PORT		GPIOB	// 编码器2 GPIO
#define CODER2_GPIO_RCC			RCC_APB2Periph_GPIOB
#define CODER2_A_PIN			GPIO_Pin_6	
#define CODER2_B_PIN			GPIO_Pin_7
/*函数声明*/
void Encoder_Init(void);
int16_t Encoder_Get(uint8_t n);

#endif
