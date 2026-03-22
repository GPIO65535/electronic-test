#ifndef __MOTOR_H
#define __MOTOR_H

/*头文件调用*/
#include "stm32f10x.h"                  // Device header
#include "PWM.h"
/*参数定义*/
#define AIN1_RCC  	RCC_APB2Periph_GPIOB	// AIN1
#define AIN1_PORT	GPIOB
#define AIN1_PIN	GPIO_Pin_12

#define AIN2_RCC  	RCC_APB2Periph_GPIOB	// AIN2
#define AIN2_PORT	GPIOB
#define AIN2_PIN	GPIO_Pin_13

#define BIN1_RCC  	RCC_APB2Periph_GPIOB	// BIN1
#define BIN1_PORT	GPIOB
#define BIN1_PIN	GPIO_Pin_14

#define BIN2_RCC  	RCC_APB2Periph_GPIOB	// BIN2
#define BIN2_PORT	GPIOB
#define BIN2_PIN	GPIO_Pin_15
/*函数声明*/
void Motor_Init(void);
void Motor_SetPWM(uint8_t n, int8_t PWM);

#endif
