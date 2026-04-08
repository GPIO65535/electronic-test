#include"main.h"
volatile uint8_t g_flag_turn = 0;
volatile uint8_t g_flag_speed = 0;
extern float g_lap_dist; 
int main(void)
{
    ALL_Init();
    Key_Init();
    Timer_Init();
    OLED_Init();
    
    // 上电后等待按键进行里程参数调整或直接开始
    // 假设按下 KEY1 切换不同的目标里程
    if(KEY_GET() == 1) {
        g_stop_target_dist = 36000.0f; 
        g_stop_coast_dist = g_stop_target_dist - 3090.0f;
    }
    
    PID_SetBaseSpeedRef(8.0f);
    //OLED_ShowString(1, 1, "V:");
    //OLED_ShowString(1, 1, "E1:");
    //OLED_ShowString(2, 1, "E2:");
    //OLED_ShowString(1,2, "P:");
    while(1)
    {
        //OLED_ShowFNum(1, 3, g_speed_filt, 4, 2);
        //OLED_ShowSignedNum(2, 4, Encoder_Get(2), 5);
        //OLED_ShowSignedNum(1, 4, Pitch, 5);
        //OLED_ShowFNum(2, 1, g_lap_dist, 5, 0); 
        if(g_flag_turn)
        {
            g_flag_turn=0;
            PID_Turn();
        }
        if(g_flag_speed)
        {
            g_flag_speed=0;
            PID_Speed();
        }
        PID_Up();
    }
}