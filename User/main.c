#include"main.h"
volatile uint8_t g_flag_turn = 0;
volatile uint8_t g_flag_speed = 0;

int main(void)
{
     ALL_Init();
     Timer_Init();
     OLED_Init();

     OLED_ShowString(1, 1, "R:");
     OLED_ShowString(2, 1, "P:");
     OLED_ShowString(3, 1, "Y:");

    while(1)
    {
        OLED_ShowFNum(1, 3, Roll, 5, 2);
        OLED_ShowFNum(2, 3, Pitch, 5, 2);
        OLED_ShowFNum(3, 3, Yaw, 5, 2);
  
        // if(g_flag_turn)
        // {
        //     g_flag_turn=0;
        //     PID_Turn();
        // }
        // if(g_flag_speed)
        // {
        //     g_flag_speed=0;
        //     PID_Speed();
        // }
        // PID_Up();
    }
}