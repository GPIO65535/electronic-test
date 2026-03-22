#ifndef __SERIAL_H
#define __SERIAL_H

/*头文件引用*/
#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
/*参数定义*/
#define USER_USART				USART1	// 使用的串口参数
#define USER_USART_RCC			RCC_APB2Periph_USART1
#define USER_USART_IQRN			USART1_IRQn
#define USART_GPIO_RCC			RCC_APB2Periph_GPIOA
#define USART_GPIO_PORT			GPIOA
#define USART_TX_PIN			GPIO_Pin_9
#define USART_RX_PIN			GPIO_Pin_10
/*函数声明*/
void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#endif
