#ifndef __OLED_H
#define __OLED_H

/*引脚定义*/
#define OLED_RCC            RCC_APB2Periph_GPIOB
#define OLED_SCL_PORT		GPIOB
#define OLED_SDA_PORT		GPIOB
#define OLED_SCL_PIN		GPIO_Pin_10
#define OLED_SDA_PIN		GPIO_Pin_11

/*函数声明*/
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowFNum(uint8_t Line, uint8_t Column, float Number, uint8_t Length, uint8_t FLength);

#endif
