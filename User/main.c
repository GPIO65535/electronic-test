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
    // // 必须用 while 循环挂起死等，否则单片机上电瞬间（几毫秒内）就会跳过这句if判断
    uint8_t key = 0;
    while(!key) {
        key = KEY_GET();
    }
    // 按下 KEY1 启动顺时针模式：改变里程，并进入方向标志位1
    if(key == 1) {
        g_stop_target_dist = 33800.0f;
        g_stop_coast_dist = 30000.0f;
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
        g_stop_target_dist = 30900.0f;
        g_stop_coast_dist = 27000.0f;
    }
    PID_ClearSpeedState(); // 启动前清空速度环状态，防止积分初始值异常导致的开机抖动
    
    PID_SetBaseSpeedRef(6.0f);
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
        //OLED_ShowSignedNum(1,1,g_speed_filt,4);
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