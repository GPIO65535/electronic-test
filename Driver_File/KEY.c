#include "Key.h"

/*全局变量，用于存储按键键码*/
uint8_t Key_Num;

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(KEY_RCC1|KEY_RCC2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	// 上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN;			// KEY1
	GPIO_Init(KEY1_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY2_PIN;			// KEY2
	GPIO_Init(KEY2_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY3_PIN;			// KEY3
	GPIO_Init(KEY3_PORT, &GPIO_InitStructure);
}


/**
  * @breaf	获取按键键码值
  * @param	无
  * @retval 0：无按键按下 
			1：按键1按下
			2：按键2按下
			3：按键3按下
*/
uint8_t KEY_GET(void)
{
	uint8_t KEY = 0;
	if(GPIO_ReadInputDataBit(KEY1_PORT,KEY1_PIN) == 0 | GPIO_ReadInputDataBit(KEY2_PORT,KEY2_PIN) == 0 |
	   GPIO_ReadInputDataBit(KEY3_PORT,KEY3_PIN) == 0)
	{
		Delay_ms(10);
		if(GPIO_ReadInputDataBit(KEY1_PORT,KEY1_PIN) == 0 | GPIO_ReadInputDataBit(KEY2_PORT,KEY2_PIN) == 0 |
		   GPIO_ReadInputDataBit(KEY3_PORT,KEY3_PIN) == 0)
		{
			if(GPIO_ReadInputDataBit(KEY1_PORT,KEY1_PIN) == 0)
				KEY = 1;
			else if(GPIO_ReadInputDataBit(KEY2_PORT,KEY2_PIN) == 0)
				KEY = 2;
			else if(GPIO_ReadInputDataBit(KEY3_PORT,KEY3_PIN) == 0)
				KEY = 3;
		}
		while(GPIO_ReadInputDataBit(KEY1_PORT,KEY1_PIN) == 0 | GPIO_ReadInputDataBit(KEY2_PORT,KEY2_PIN) == 0 |
		      GPIO_ReadInputDataBit(KEY3_PORT,KEY3_PIN) == 0);			// 等待按键释放
	}
	return KEY;
}


///**
//  * 函    数：获取全局变量定义的按键键码
//  * 参    数：无
//  * 返 回 值：按键键码
//  */
//uint8_t Key_GetNum(void)
//{
//	uint8_t Temp;			//定义一个临时变量用于中转
//	if (Key_Num)			//如果全局变量的键码不为0
//	{
//		/*这3句的目的是，实现读取键码并读后清零的效果*/
//		Temp = Key_Num;		//先把键码存入临时变量
//		Key_Num = 0;		//键码清零
//		return Temp;		//返回临时变量，return语句执行后，函数直接结束
//	}
//	return 0;				//如果if不成立，键码为0，则默认返回0
//}


///**
//  * 函    数：获取按键状态
//  * 参    数：无
//  * 返 回 值：有按键按下，直接返回键码（非阻塞），没有按键按下，返回0
//  */
//uint8_t Key_GetState(void)
//{
//	if (GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == 0)		//如果按键1按下
//	{
//		return 1;		//直接返回键码1
//	}
//	if (GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN) == 0)		//如果按键2按下
//	{
//		return 2;		//直接返回键码2
//	}
//	if (GPIO_ReadInputDataBit(KEY3_PORT, KEY3_PIN) == 0)		//如果按键3按下
//	{
//		return 3;		//直接返回键码3
//	}
//	return 0;			//没有if成立，表示没有按键按下，默认返回0
//}


///**
//  * 函    数：用于驱动按键模块运行的自定义按键定时中断函数
//  * 参    数：无
//  * 返 回 值：无
//  * 注意事项：此函数必须在主程序中每隔1ms自动执行一次
//  */
//void Key_Tick(void)
//{
//	/*定义静态变量（默认初值为0，函数退出后保留值和存储空间）*/
//	static uint8_t Count;					//用于计次分频
//	static uint8_t CurrState, PrevState;	//保存按键本次状态和上次状态
//	
//	Count ++;			//计次自增
//	if (Count >= 20)	//如果计次20次，则if成立，即if每隔20ms进一次
//	{
//		Count = 0;		//计次清零，便于下次计次
//		
//		/*获取按键的本次状态和上次状态*/
//		PrevState = CurrState;			//获取上次状态
//		CurrState = Key_GetState();		//获取本次状态
//		
//		/*如果本次状态的键码为0，且上次键码不为0，即检测到按键松手瞬间*/
//		if (CurrState == 0 && PrevState != 0)
//		{
//			/*将上次状态的键码复制给全局变量，后续读取此变量，即可得知哪个按键按下了*/
//			Key_Num = PrevState;
//		}
//	}
//}
