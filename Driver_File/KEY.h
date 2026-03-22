#ifndef __KEY_H
#define __KEY_H

/*头文件引用*/
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Timer.h"
/*引脚参数定义*/
#define KEY_RCC1	RCC_APB2Periph_GPIOA
#define KEY_RCC2	RCC_APB2Periph_GPIOC
#define KEY1_PORT	GPIOA
#define KEY1_PIN	GPIO_Pin_11
#define KEY2_PORT	GPIOA
#define KEY2_PIN	GPIO_Pin_12
#define KEY3_PORT	GPIOC
#define KEY3_PIN	GPIO_Pin_14
/*函数声明*/
void Key_Init(void);
uint8_t KEY_GET(void);

#endif
