$content = Get-Content -Raw "Driver_File/PID.c"
$old_str = @"
void PID_Speed(void)
{
    static uint8_t pid_speed_init = 0;
    static float turn_brake_filt=0.0f;
    float alpha=0.2f;
    float v;
    float v_ref; // 速度环目标速度

    static PID_t PID_speedstruct;
    if(!pid_speed_init)
    {
        // 增大Kp和Ki，之前Ki太小（.00075），导致降速时完全无法抵抗电池重心偏
差，从而被重力拉着倒车。
        PID_speedstruct.kp=0.15f;
        PID_speedstruct.ki=0.15f/200.0f;
        PID_speedstruct.kd=0.0f;
        PID_speedstruct.target=0.0f;
        PID_speedstruct.actual=0.0f;
        PID_speedstruct.error=0.0f;
        PID_speedstruct.last_error=0.0f;
        PID_speedstruct.integral=0.0f;
        PID_speedstruct.derivative=0.0f;
        PID_speedstruct.output=0.0f;
        pid_speed_init = 1;
    }
    v=Get_CarSpeed();
    //速度低通滤波
    g_speed_filt=alpha*v+(1-alpha)*g_speed_filt;        

    // // ----- 定点停车逻辑（里程+寻线双重判断）-----
    // static uint8_t g_stop_state = 0; // 0=盲区跑, 1=已武装(可触发), 3=已停  
    // static uint8_t g_line_lock = 0;
    // #define STOP_ARM_DIST 27500.0f // 【需根据实际测试微调】跑完大半圈（过CD后，未到B前）的累加里程。
    // g_lap_dist += AbsF(v);

    // if (g_gray_cnt < 4) {
    //     g_line_lock = 0; // 离开黑线解心
    // }

    // if (g_stop_state == 0) {
    //     if (g_lap_dist > STOP_ARM_DIST) {
    //         g_stop_state = 1;
    //     }
    // } else if (g_stop_state == 1) {
    //     if (g_gray_cnt >= 7 && !g_line_lock) {
    //         g_line_lock = 1;
    //         g_stop_state = 3; // 压到B线全黑，停车
    //     }
    // }

    // if (g_stop_state == 3) {
    //     v_ref = 0.0f; // 停车目标
    //     // 瞬间赋予极其强大的驻车刹车力
    //     PID_speedstruct.kp = 0.5f;
    //     PID_speedstruct.ki = 0.02f;
    // } else {
    //     // 恢复原有微弱软趴的巡线状态，防止直道抽搐后退    
    //     PID_speedstruct.kp = 0.15f;
    //     PID_speedstruct.ki = 0.15f/200.0f;

    //     // ---【防冲出核心：动态弯道减速】---
    //     // 跑圈时遇到弯道必须动态减速，否则按自然重心直
冲会飞出赛道！
    //     float turn_err = AbsF(Get_Grayerror());
    //     if (turn_err > 1.2f) {  // 偏差较大（外侧灯亮起，进入大弯）
    //         v_ref = g_base_v_ref * 0.4f - 3.0f; // 强行拉低目标速度产生瞬间刹车力抵消惯性
    //     } else {
    //         v_ref = g_base_v_ref; // 直线维持正常
    //     }
    // }
    // ----------------------------------------

    // OLED实时查看里程状态，方便调试（由于你未启动可以在此作为调试依据）
    // 目前里程累加阈值为21000，如果你没到B线就已触发，则该值过小；如果过B线仍不停，则过大。
    PID_speedstruct.target = v_ref;
    PID_speedstruct.actual = g_speed_filt;
    float pid_output=PID_Cal(&PID_speedstruct,DT_SPEED);// dt使用实际控制周期，实际运行时间直立环决定
    // 将速度环输出限幅后给直立环作为目标倾角
    g_target_pitch_from_speed=PID_Limit(pid_output,-5.0f,5.0f);
}
