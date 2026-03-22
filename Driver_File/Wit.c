#include "Wit.h"
#include <string.h>

static uint8_t RxBuffer[11];			/*接收数据数组*/
uint8_t WIT_RxFrame[11];				/*最近一次校验成功的数据帧*/
volatile uint8_t WIT_RxFrameReady = 0;	/*新数据帧就绪标志位*/
static volatile uint8_t RxState = 0;	/*接收状态标志位*/
static uint8_t RxIndex = 0;				/*接受数组索引*/
float Roll,Pitch,Yaw;					/*角度信息，如果只需要整数可以改为整数类型*/

/**
  * 函    数：Wit初始化
  * 参    数：BaudRate 串口波特率
  * 返 回 值：无
  */
void WIT_Init(uint32_t BaudRate)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(WIT_USART_RCC, ENABLE);			//开启WIT_USART的时钟
	RCC_APB2PeriphClockCmd(WIT_GPIO_RCC, ENABLE);			//开启WIT_GPIO_RCC的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = WIT_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(WIT_GPIO_PORT, &GPIO_InitStructure);			//将WIT_TX_PIN引脚初始化为复用推挽输出
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = WIT_RX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(WIT_GPIO_PORT, &GPIO_InitStructure);			//将WIT_RX_PIN引脚初始化为上拉输入
	
	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;											//定义结构体变量
	USART_InitStructure.USART_BaudRate = BaudRate;              					//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要rdwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;					//模式，发送模式和接收模式均选择;
	USART_InitStructure.USART_Parity = USART_Parity_No;     						//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  						//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长，选择8位8b;
	USART_Init(WIT_USART, &USART_InitStructure);               	//将结构体变量交给WIT_Init，配置WIT_USART
	
	/*中断输出配置*/
	USART_ITConfig(WIT_USART, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
	
	/*NVIC中断分组在主程序中完成一次*/
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;						//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = WIT_USART_IRQN;   		//选择配置WIT_USART_IQRN
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         	//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//指定NVIC线路的抢占优先级为1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      	//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);                         	//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*USART使能*/
	USART_Cmd(WIT_USART, ENABLE);								//使能USER_USART，串口开始运行
}


/**
 * @brief       数据包处理函数
 * @param       串口接收的数据RxData
 * @retval      无
 */
void jy61p_ReceiveData(uint8_t RxData)
{
	uint8_t i,sum=0;
	
	if (RxState == 0)	//等待包头
	{
		if (RxData == 0x55)	//收到包头
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 1;
			RxIndex = 1; //进入下一状态
		}
	}
	
	else if (RxState == 1)
	{
		if (RxData == 0x53)	/*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 2;
			RxIndex = 2; //进入下一状态
		}
	}
	
	else if (RxState == 2)	//接收数据
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//接收完成
		{
			for(i=0;i<10;i++)
			{
				sum = sum + RxBuffer[i]; //计算校验和
			}
			if(sum == RxBuffer[10])		//校验成功
			{
				memcpy(WIT_RxFrame, RxBuffer, sizeof(WIT_RxFrame));
				WIT_RxFrameReady = 1;
				/*计算数据，根据数据内容选择对应的计算公式*/
				Roll = ((int16_t)(RxBuffer[2] | (RxBuffer[3] << 8))) / 32768.0f * 180.0f;
				Pitch = ((int16_t)(RxBuffer[4] | (RxBuffer[5] << 8))) / 32768.0f * 180.0f;
				Yaw = ((int16_t)(RxBuffer[6] | (RxBuffer[7] << 8))) / 32768.0f * 180.0f;
			}
			RxState = 0;
			RxIndex = 0; //读取完成，回到最初状态，等待包头
		}
	}
}

 
/**
  * 函    数：USART2中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  *			  根据实际情况修改中断函数
  */
void USART2_IRQHandler(void)
{
    uint8_t RxData;
    if (USART_GetITStatus(WIT_USART, USART_IT_RXNE) != RESET)
    {
        RxData = USART_ReceiveData(WIT_USART);
        jy61p_ReceiveData(RxData);
        USART_ClearITPendingBit(WIT_USART, USART_IT_RXNE);
    }
}
