#include "LED.h"



/**
  * @breaf	LED模块初始化
  * @param	无
  * @retval	无
*/
void LED_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(LED_RCC1|LED_RCC2, ENABLE);
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;					//定义结构体变量
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//GPIO模式，赋值为推挽输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//GPIO速度，赋值为50MHz
	
	GPIO_InitStructure.GPIO_Pin = LED1_PIN;				//LED1
	GPIO_Init(LED1_PORT, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = LED2_PIN;				//LED2
	GPIO_Init(LED2_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED3_PIN;				//LED3
	GPIO_Init(LED3_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED4_PIN;				//LED4
	GPIO_Init(LED4_PORT, &GPIO_InitStructure);

}



/**
  * @breaf	LED1设置
  * @param	state LED状态 0 -> 熄灭 ; 1 -> 点亮 ; 其他 -> 熄灭
  * @retval	无
*/
void LED1_SET(uint8_t state)
{
	switch (state)
    {
    	case 1: GPIO_ResetBits(LED1_PORT,LED1_PIN);
    		break;
    	case 0: GPIO_SetBits(LED1_PORT,LED1_PIN);
    		break;
    	default: GPIO_SetBits(LED1_PORT,LED1_PIN);
    		break;
    }
}



/**
  * @breaf	LED2设置
  * @param	state LED状态 0 -> 熄灭 ; 1 -> 点亮 ; 其他 -> 熄灭
  * @retval	无
*/
void LED2_SET(uint8_t state)
{
	switch (state)
    {
    	case 1: GPIO_ResetBits(LED2_PORT,LED2_PIN);
    		break;
    	case 0: GPIO_SetBits(LED2_PORT,LED2_PIN);
    		break;
    	default: GPIO_SetBits(LED2_PORT,LED2_PIN);
    		break;
    }
}



/**
  * @breaf	LED3设置
  * @param	state LED状态 0 -> 熄灭 ; 1 -> 点亮 ; 其他 -> 熄灭
  * @retval	无
*/
void LED3_SET(uint8_t state)
{
	switch (state)
    {
    	case 1: GPIO_ResetBits(LED3_PORT,LED3_PIN);
    		break;
    	case 0: GPIO_SetBits(LED3_PORT,LED3_PIN);
    		break;
    	default: GPIO_SetBits(LED3_PORT,LED3_PIN);
    		break;
    }
}



/**
  * @breaf	LED4设置
  * @param	state LED状态 0 -> 熄灭 ; 1 -> 点亮 ; 其他 -> 熄灭
  * @retval	无
*/
void LED4_SET(uint8_t state)
{
	switch (state)
    {
    	case 1: GPIO_ResetBits(LED4_PORT,LED4_PIN);
    		break;
    	case 0: GPIO_SetBits(LED4_PORT,LED4_PIN);
    		break;
    	default: GPIO_SetBits(LED4_PORT,LED4_PIN);
    		break;
    }
}





