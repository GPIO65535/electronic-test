#ifndef __GRAYSENSOR_H
#define __GRAYSENSOR_H
#include"delay.h"
#include"stm32f10x.h"
//Òý½Åºê¶¨Òå
#define GRAY_RCC_GPIO_CLK1 RCC_APB2Periph_GPIOA
#define GRAY_RCC_GPIO_CLK2 RCC_APB2Periph_GPIOB
#define GRAY_CH0_PORT GPIOA
#define GRAY_CH0_PIN GPIO_Pin_4
#define GRAY_CH1_PORT GPIOA
#define GRAY_CH1_PIN GPIO_Pin_5
#define GRAY_CH2_PORT GPIOB
#define GRAY_CH2_PIN GPIO_Pin_0
#define GRAY_OUT_PORT GPIOB
#define GRAY_OUT_PIN GPIO_Pin_1
void GraySensor_Init(void);
uint8_t GraySensor_ReadChannel(uint8_t channel);
void GraySensor_ReadAll(uint8_t data[8]);

#endif
