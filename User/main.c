#include"main.h"
volatile uint8_t g_flag_turn = 0;
volatile uint8_t g_flag_speed = 0;

int main(void)
{
    // ALL_Init();
    // Timer_Init();
    uint8_t key_num = 0;

    LED_Init();
    Key_Init();
    OLED_Init();

    LED1_SET(0);
    OLED_Clear();
    OLED_ShowString(1,1,"KEY1:WAIT");
    OLED_ShowString(2,1,"LED1:OFF ");

    while(1)
    {
        key_num = KEY_GET();
        if(key_num == 1)
        {
            LED1_SET(1);
            OLED_ShowString(1,1,"KEY1:PRESS");
            OLED_ShowString(2,1,"LED1:ON  ");
        }
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