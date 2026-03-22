#ifndef __Wit_H
#define __Wit_H

/*头文件调用*/
#include "stm32f10x.h"
/*参数定义*/
#define WIT_USART				USART2	// 使用的串口参数
#define WIT_USART_RCC			RCC_APB1Periph_USART2
#define WIT_USART_IRQN			USART2_IRQn
#define WIT_GPIO_RCC			RCC_APB2Periph_GPIOA
#define WIT_GPIO_PORT			GPIOA
#define WIT_TX_PIN			GPIO_Pin_2
#define WIT_RX_PIN			GPIO_Pin_3
/*变量声明*/
extern float Roll,Pitch,Yaw;
extern uint8_t WIT_RxFrame[11];
extern volatile uint8_t WIT_RxFrameReady;
/*函数声明*/
void WIT_Init(uint32_t BaudRate);
void jy61p_ReceiveData(uint8_t RxData);

#endif
