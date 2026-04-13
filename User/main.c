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
    uint8_t key = 0;
    float start_speed_ref = 13.0f;
    uint8_t start_delay_done = 0;
    uint32_t start_ms = 0;
    while(!key) {
        key = KEY_GET();
    }
    // 按下 KEY1 启动顺时针模式：改变里程，并进入方向标志位1
    if(key == 1) {
        g_stop_target_dist = 32500.0f;
        g_stop_coast_dist = 31500.0f;
        g_run_dir = 1; // 1:顺时针
        g_task_mode = 1;
        start_speed_ref = 13.0f;
    } else if(key == 3) {
        g_run_dir = 0;              // 从A逆时针启动
        g_task_mode = 3;            // 标记进入考题模式
        g_cross_line_cnt = 0;          // 计数清零
        g_stop_coast_dist = 40000.0f;
        g_stop_target_dist = 41000.0f;
        start_speed_ref = 13.0f;
    } else {
        // 默认逆时针启动
        g_run_dir = 0;
        g_task_mode = 0;
        g_stop_target_dist = 30100.0f;
        g_stop_coast_dist = 27000.0f;
        start_speed_ref = 13.0f;
    }
    PID_ClearSpeedState(); // 启动前清空速度环状态，防止积分初始值异常导致的开机抖动
    PID_SetBaseSpeedRef(0.0f);
    start_ms = PID_Timebase1ms_Get();
    while(1)
    {
        if (!start_delay_done && (PID_Timebase1ms_Get() - start_ms >= 700))
        {
            PID_ClearSpeedState();
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