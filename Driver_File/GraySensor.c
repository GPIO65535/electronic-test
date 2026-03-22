#include"GraySensor.h"
/*
 * @brief  灰度传感器通道选择函数
 * @param  code 通道代码，0-7
 * @retval None
 */
static void GraySensor_SelectCode(uint8_t code)
{
    GPIO_WriteBit(GRAY_CH0_PORT, GRAY_CH0_PIN, (BitAction)(code & 0x01));
    GPIO_WriteBit(GRAY_CH1_PORT, GRAY_CH1_PIN, (BitAction)((code >> 1) & 0x01));
    GPIO_WriteBit(GRAY_CH2_PORT, GRAY_CH2_PIN, (BitAction)((code >> 2) & 0x01));
}
/**
 * @brief  灰度传感器初始化函数
 * @param  None
 * @retval None

 */
void GraySensor_Init(void)
{
    
    RCC_APB2PeriphClockCmd(GRAY_RCC_GPIO_CLK1 | GRAY_RCC_GPIO_CLK2, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin=GRAY_CH0_PIN | GRAY_CH1_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GRAY_CH0_PORT, &GPIO_InitStructure);
    GPIO_InitTypeDef GPIO_Structure;
    GPIO_Structure.GPIO_Pin=GRAY_CH2_PIN;
    GPIO_Structure.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_Structure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GRAY_CH2_PORT, &GPIO_Structure);
    GPIO_InitTypeDef GPIO_OUT_Structure;
    GPIO_OUT_Structure.GPIO_Pin=GRAY_OUT_PIN;
    GPIO_OUT_Structure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_OUT_Structure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GRAY_OUT_PORT, &GPIO_OUT_Structure);
}

/**
 * @brief  读取指定通道灰度值
 * @param  channel 通道号，1~8（1 对应 CH2CH1CH0=000，2 对应 001 ... 8 对应 111）
 * @retval 0 或 1（GRAY_OUT 引脚电平）
 */
uint8_t GraySensor_ReadChannel(uint8_t channel)
{
    uint8_t code;

    if (channel < 1 || channel > 8)
    {
        return 0;
    }

    code = channel - 1;
    GraySensor_SelectCode(code);
    Delay_us(5);

    return (uint8_t)GPIO_ReadInputDataBit(GRAY_OUT_PORT, GRAY_OUT_PIN);
}

/**
 * @brief  依次读取 8 路灰度值
 * @param  data 长度为 8 的数组，data[0]~data[7] 对应通道 1~8
 * @retval None
 */
void GraySensor_ReadAll(uint8_t data[8])
{
    uint8_t i;

    if (data == 0)
    {
        return;
    }

    for (i = 0; i < 8; i++)
    {
        data[i] = GraySensor_ReadChannel(i + 1);
    }
}