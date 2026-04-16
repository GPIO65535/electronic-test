#include"main.h"
volatile uint8_t g_flag_turn = 0;
volatile uint8_t g_flag_speed = 0;
extern float g_lap_dist; 
int main(void)
{
    ALL_Init();
    Key_Init();
    Timer_Init();
    //OLED_Init();
    uint8_t key = 0;
    uint8_t key1_double_press = 0;
    float start_speed_ref = 25.0f;
    uint8_t start_delay_done = 0;
    uint32_t start_ms = 0;
    while(!key) {
        if (GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == 0) {
            uint16_t key1_wait_ms = 0;
            while (GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == 0) {
                Delay_ms(10);
            }
            key = 1;
            while (key1_wait_ms < 350) {
                if (GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == 0) {
                    while (GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == 0) {
                        Delay_ms(10);
                    }
                    key1_double_press = 1;
                    break;
                }
                Delay_ms(10);
                key1_wait_ms += 10;
            }
        } else {
            key = KEY_GET();
        }
    }
    // 按key1一次第一问，两次发挥第一问
    if(key == 1) {
        if (key1_double_press) {
            g_run_dir = 0;              
            g_task_mode = 3;            
            g_cross_line_cnt = 0;       
            g_stop_coast_dist = 40000.0f;
            g_stop_target_dist = 41000.0f;
            start_speed_ref = 25.0f;
        } else {
            start_speed_ref = -0.35f;
        }
        //key3:第三问
    } else if(key == 3) {
        g_run_dir = 1;              
        g_stop_target_dist = 33230.0f;
        g_stop_coast_dist = 32000.0f;
        start_speed_ref = 25.0f;
        //key2:第二问
    } else {    
        g_run_dir = 0;
        g_stop_target_dist = 29850.0f;
        g_stop_coast_dist = 27000.0f;
        start_speed_ref = 25.0f;
    }
    PID_ClearSpeedState(); // 启动前清空速度环状态，防止积分初始值异常导致的开机抖动
    PID_SetBaseSpeedRef(0.0f);
    start_ms = PID_Timebase1ms_Get();
    while(1)
    {
        if (!start_delay_done && (PID_Timebase1ms_Get() - start_ms >= 700))
        {
            PID_SetBaseSpeedRef(start_speed_ref);
            start_delay_done = 1;
        }

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