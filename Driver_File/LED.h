#ifndef __LED_H
#define __LED_H

/*头文件调用*/
#include "stm32f10x.h"                  // Device header
/*引脚参数定义*/
#define LED_RCC1			RCC_APB2Periph_GPIOB
#define LED_RCC2			RCC_APB2Periph_GPIOC
#define LED1_PORT			GPIOB
#define LED1_PIN			GPIO_Pin_5
#define LED2_PORT			GPIOB
#define LED2_PIN			GPIO_Pin_8
#define LED3_PORT			GPIOB
#define LED3_PIN			GPIO_Pin_9
#define LED4_PORT			GPIOC
#define LED4_PIN			GPIO_Pin_13

/*函数声明*/
void LED_Init(void);
void LED1_SET(uint8_t state);
void LED2_SET(uint8_t state);
void LED3_SET(uint8_t state);
void LED4_SET(uint8_t state);
void LED5_SET(uint8_t state);

#endif
