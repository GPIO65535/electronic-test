#include "Encoder.h"                  // Device header

/**
  * 函    数：编码器初始化
  * 参    数：无
  * 返 回 值：无
  */
void Encoder_Init(void)
{
	/*配置CODER2_TIM为编码器模式，用于读取左电机的旋转速度*/
	
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(CODER1_TIM_RCC, ENABLE);					//开启CODER1_TIM的时钟
	RCC_APB2PeriphClockCmd(CODER1_GPIO_RCC, ENABLE);				//开启CODER1_GPIO的时钟
	TIM_InternalClockConfig(CODER1_TIM);
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = CODER1_A_PIN | CODER1_B_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CODER1_GPIO_PORT, &GPIO_InitStructure);				//将CODER1_A和CODER_B引脚初始化为上拉输入
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;               //计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;                //预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(CODER1_TIM, &TIM_TimeBaseInitStructure);       //将结构体变量交给TIM_TimeBaseInit，配置CODER1_TIM的时基单元
	
	/*输入捕获初始化*/
	TIM_ICInitTypeDef TIM_ICInitStructure;							//定义结构体变量
	TIM_ICStructInit(&TIM_ICInitStructure);                         //结构体初始化，若结构体没有完整赋值
	                                                                //则最好执行此函数，给结构体所有成员都赋一个默认值
	                                                                //避免结构体初值不确定的问题
	TIM_ICInitStructure.TIM_Channel = CODER1_A_CHANNEL;             //选择配置定时器通道1
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                         //输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(CODER1_TIM, &TIM_ICInitStructure);                   //将结构体变量交给TIM_ICInit，配置CODER1_TIM的输入捕获通道
	TIM_ICInitStructure.TIM_Channel = CODER1_B_CHANNEL;             //选择配置定时器通道2
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                         //输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(CODER1_TIM, &TIM_ICInitStructure);                   //将结构体变量交给TIM_ICInit，配置CODER1_TIM的输入捕获通道
	
	/*编码器接口配置*/
	TIM_EncoderInterfaceConfig(CODER1_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
																	//配置编码器模式以及两个输入通道是否反相
	                                                                //注意此时参数的Rising和Falling已经不代表上升沿和下降沿了，而是代表是否反相
	                                                                //此函数必须在输入捕获初始化之后进行，否则输入捕获的配置会覆盖此函数的部分配置
	/*CODER1_TIM使能*/
	TIM_Cmd(CODER1_TIM, ENABLE);			//使能CODER1_TIM，定时器开始运行
	
	
	
	
	/*配置CODER2_TIM为编码器模式，用于读取右电机的旋转速度*/
	
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(CODER2_TIM_RCC, ENABLE);					//开启CODER2_TIM的时钟
	RCC_APB2PeriphClockCmd(CODER2_GPIO_RCC, ENABLE);				//开启CODER2_GPIO的时钟
	TIM_InternalClockConfig(CODER2_TIM);
	/*GPIO初始化*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = CODER2_A_PIN | CODER2_B_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CODER2_GPIO_PORT, &GPIO_InitStructure);				//将CODER2_A_PIN和CODER2_B_PIN引脚初始化为上拉输入
		
	/*时基单元初始化*/
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;               //计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;                //预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(CODER2_TIM, &TIM_TimeBaseInitStructure);       //将结构体变量交给TIM_TimeBaseInit，配置TIM4的时基单元
	
	/*输入捕获初始化*/
	TIM_ICStructInit(&TIM_ICInitStructure);                         //结构体初始化，若结构体没有完整赋值
	                                                                //则最好执行此函数，给结构体所有成员都赋一个默认值
	                                                                //避免结构体初值不确定的问题
	TIM_ICInitStructure.TIM_Channel = CODER2_A_CHANNEL;             //选择配置定时器通道1
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                         //输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(CODER2_TIM, &TIM_ICInitStructure);                   //将结构体变量交给TIM_ICInit，配置TIM4的输入捕获通道
	TIM_ICInitStructure.TIM_Channel = CODER2_B_CHANNEL;             //选择配置定时器通道2
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                         //输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(CODER2_TIM, &TIM_ICInitStructure);                         //将结构体变量交给TIM_ICInit，配置TIM4的输入捕获通道
	
	/*编码器接口配置*/
	TIM_EncoderInterfaceConfig(CODER2_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
																	//配置编码器模式以及两个输入通道是否反相
	                                                                //注意此时参数的Rising和Falling已经不代表上升沿和下降沿了，而是代表是否反相
	                                                                //此函数必须在输入捕获初始化之后进行，否则输入捕获的配置会覆盖此函数的部分配置
	/*CODER2_TIM使能*/
	TIM_Cmd(CODER2_TIM, ENABLE);			//使能CODER2_TIM，定时器开始运行
}




/**
  * 函    数：获取编码器的增量值
  * 参    数：n 指定左电机还是右电机，范围：1（左电机），2（右电机）
  * 返 回 值：自上此调用此函数后，编码器的增量值
  */
int16_t Encoder_Get(uint8_t n)
{
	/*使用Temp变量作为中继，目的是返回CNT后将其清零*/
	int16_t Temp;
	if (n == 1)					//指定左电机
	{
		Temp = TIM_GetCounter(CODER1_TIM);
		TIM_SetCounter(CODER1_TIM, 0);
		return Temp;			//CODER1_TIM（左电机编码器）CNT增量值
	}
	else if (n == 2)			//指定右电机
	{
		Temp = TIM_GetCounter(CODER2_TIM);
		TIM_SetCounter(CODER2_TIM, 0);
		return Temp;			//返回CODER2_TIM（右电机编码器）CNT增量值
	}
	return 0;		//参数n非法，默认返回0
}
