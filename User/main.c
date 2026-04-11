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
    while(!key) {
        key = KEY_GET();
    }
    // 按下 KEY1 启动顺时针模式：改变里程，并进入方向标志位1
    if(key == 1) {
        g_stop_target_dist = 33700.0f;
        g_stop_coast_dist = 32000.0f;
        g_run_dir = 1; // 1:顺时针
        g_task_mode = 1;
    } else if(key == 3) {
        // --- 核心考题模式逻辑 ---
        g_run_dir = 0;              // 从A逆时针启动
        g_task_mode = 3;            // 标记进入考题模式
        g_cross_line_cnt = 0;          // 计数清零
        
        // 初始里程设定为极大值，直到第7次全黑（B线）再激活停车系统
        g_stop_coast_dist = 999999.0f;
        g_stop_target_dist = 999999.0f;
    } else {
        // 默认逆时针启动
        g_run_dir = 0;
        g_task_mode = 0;
        g_stop_target_dist = 30500.0f;
        g_stop_coast_dist = 27000.0f;
    }
    PID_ClearSpeedState(); // 启动前清空速度环状态，防止积分初始值异常导致的开机抖动
    PID_SetBaseSpeedRef(10.0f);
    // OLED_ShowString(1, 1, "M:");
    // OLED_ShowString(1, 9, "C:");
    // OLED_ShowString(2, 1, "T:");
    // OLED_ShowString(2, 9, "D:");
    while(1)
    {
        // OLED_ShowNum(1, 3, g_task_mode, 1);
        // OLED_ShowNum(1, 11, g_cross_line_cnt, 1);
        // OLED_ShowNum(2, 3, g_is_turning_180, 1);
        // OLED_ShowNum(2, 11, g_run_dir, 1);
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