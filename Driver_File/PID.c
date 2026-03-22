#include"PID.h"
static float g_target_pitch_from_speed=0.0f; // 速度环输出的目标角度，供直立环使用
static float g_speed_filt=0.0f;//速度测量滤波
static float g_turn_output=0.0f;//转速偏差
static float g_up_dt=DT_UP;//直立环动态dt
static volatile uint32_t g_pid_ms_tick=0;//1ms系统节拍

#define DT_UP_MIN       0.003f
#define DT_UP_MAX       0.015f
#define DT_UP_TIMEOUT   0.03f
/**
 * 函数功能：外设初始化
 * 参数：无
 */
void ALL_Init(void)
{
    GraySensor_Init();
    Encoder_Init();
    Motor_Init();
    WIT_Init(115200);
}
/**
 * 函数功能：PID计算
 * 参数：pid - PID结构体指针，dt - 时间间隔
 */
float PID_Cal(PID_t *pid,float dt)
{
    if(dt <= 0)
    {
        return pid->output;
    }

    pid->error=pid->target-pid->actual;
    pid->integral+=pid->error*dt;
    pid->derivative=(pid->error-pid->last_error)/dt;
    pid->output=pid->kp*pid->error+pid->ki*pid->integral+pid->kd*pid->derivative;
    pid->last_error=pid->error;
    return pid->output;
}
/**
 * 函数功能：PID限幅
 * 参数：value - 待限幅的值，min - 最小值，max - 最大值
 * 返回值：限幅后的值
 */
float PID_Limit(float value,float min,float max)
{
    if(value>max)
    {
        value=max;
    }
    else if(value<min)
    {
         value=min;
    }
    return value; 
}

/**
 * 函数功能：1ms节拍计数
 */
void PID_Timebase1ms_Tick(void)
{
    g_pid_ms_tick++;
}

/**
 * 函数功能：获取当前1ms节拍计数
 */
uint32_t PID_Timebase1ms_Get(void)
{
    return g_pid_ms_tick;
}

/**
 * 函数功能：更新直立环实际控制周期
 * 参数：dt - 实际周期，单位秒
 */
void PID_Up_UpdateDt(float dt)
{
    if(dt <= 0.0f)
    {
        return;
    }

    if(dt > DT_UP_TIMEOUT)
    {
        return;
    }

    g_up_dt = PID_Limit(dt, DT_UP_MIN, DT_UP_MAX);
}
/**
 * 函数功能：平衡车直立环
 * 参数：无
 */
void PID_Up(void)
{
    static uint8_t pid_up_init = 0;
    static uint8_t imu_tick_init = 0;
    static PID_t PID_upstruct;
    static uint32_t last_imu_ms = 0;
    float pidup_output;
    float dt_used;
    float left_pwm,right_pwm=0.0f;
    uint32_t now_ms;
    uint32_t frame_dt_ms;

    if(!WIT_RxFrameReady)
    {
        return;
    }
    WIT_RxFrameReady=0;

    now_ms = PID_Timebase1ms_Get();
    if(imu_tick_init)
    {
        frame_dt_ms = now_ms - last_imu_ms;
        PID_Up_UpdateDt((float)frame_dt_ms * 0.001f);
    }
    last_imu_ms = now_ms;
    imu_tick_init = 1;

    if(!pid_up_init)
    {
        PID_upstruct.kp=0.5f;
        PID_upstruct.ki=0.0f;
        PID_upstruct.kd=0.05f;
        PID_upstruct.target=0.0f;
        PID_upstruct.actual=0.0f;
        PID_upstruct.error=0.0f;
        PID_upstruct.last_error=0.0f;
        PID_upstruct.integral=0.0f;
        PID_upstruct.derivative=0.0f;
        PID_upstruct.output=0.0f;
        pid_up_init = 1;
    }

    PID_upstruct.target = g_target_pitch_from_speed;      // 目标直立角，后续可按实车偏差调整
    PID_upstruct.actual = Pitch;

    dt_used = PID_Limit(g_up_dt, DT_UP_MIN, DT_UP_MAX);
    pidup_output=PID_Cal(&PID_upstruct,dt_used); // 使用动态dt减少串口帧抖动影响
    pidup_output=PID_Limit(pidup_output,-99.0f,99.0f);
    left_pwm=PID_Limit(pidup_output-g_turn_output,-99.0f,99.0f);
    right_pwm=PID_Limit(pidup_output+g_turn_output,-99.0f,99.0f);

    Motor_SetPWM(1,(int8_t)left_pwm);
    Motor_SetPWM(2,(int8_t)right_pwm);
}
/**
 * 获取平衡车当前速度，供速度环使用
 */
static float Get_CarSpeed(void)
{
    float v_l=Encoder_Get(1);
    float v_r=Encoder_Get(2);
    return (v_l+v_r)*0.5f;
}
/**
 * 获取灰度传感器误差，供转向环使用
 */
static float Get_Grayerror(void)
{
    static const float weight[8]={-3.5f,-2.5f,-1.5f,-0.5f,0.5f,1.5f,2.5f,3.5f}; // 灰度传感器权重，根据实际情况调整
    static float last_e=0.0f;//上次误差值
    uint8_t gray[8];//灰度传感器原始值
    uint8_t i,cnt=0;
    float sum=0;
    GraySensor_ReadAll(gray);
    for(i=0;i<8;i++)
    {
        #if LINE_IS_LOW  
            if(gray[i]==0)
        #else        
            if(gray[i]==1)
        #endif
        {
            sum+=weight[i];
            cnt++;
        }
    }
    if(cnt==0)
    {
        return last_e; // 如果没有检测到线，返回上次的误差值，保持转向不变
    }
    last_e=sum/cnt; // 计算平均误差
    return last_e;
}
/*
 * 函数功能：平衡车速度环(串级pid,速度环输出目标角给直立环)*/
void PID_Speed(void)
{
    static uint8_t pid_speed_init = 0;
    float alpha=0.2f;
    float v;
    float v_ref=0.0f; // 速度环目标速度，后续可按实车偏差调整
    static PID_t PID_speedstruct;
    if(!pid_speed_init)
    {
        PID_speedstruct.kp=0.5f;
        PID_speedstruct.ki=0.01f;
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
    PID_speedstruct.target=v_ref;
    PID_speedstruct.actual=g_speed_filt;
    float pid_output=PID_Cal(&PID_speedstruct,DT_SPEED);// dt根据实际控制周期设置,实车运行时建议直立环快
    PID_speedstruct.integral=PID_Limit(PID_speedstruct.integral,-100.0f,100.0f); // 积分限幅，防止积分过大
    //将速度环输出的目标角限制在合理范围，供直立环使用
    g_target_pitch_from_speed=PID_Limit(pid_output,-5.0f,5.0f);
}
void PID_Turn(void)
{
    static PID_t PID_turnstruct;
    static uint8_t pid_turn_init=0;
    float e_line,out;
    if(!pid_turn_init)
    {
        PID_turnstruct.kp=0.5f;
        PID_turnstruct.ki=0.0f;
        PID_turnstruct.kd=0.01f;
        PID_turnstruct.target=0.0f;
        PID_turnstruct.actual=0.0f;
        PID_turnstruct.error=0.0f;
        PID_turnstruct.last_error=0.0f;
        PID_turnstruct.integral=0.0f;
        PID_turnstruct.derivative=0.0f;
        PID_turnstruct.output=0.0f;
        pid_turn_init=1;
    }
    e_line=Get_Grayerror();
    PID_turnstruct.target=0.0f;
    PID_turnstruct.actual=e_line;
    out=PID_Cal(&PID_turnstruct,DT_TURN);
    PID_turnstruct.integral=PID_Limit(PID_turnstruct.integral,-100.0f,100.0f); // 积分限幅，防止积分过大
    g_turn_output=PID_Limit(out,-35.0f,35.0f);
}

void TIM1_UP_IRQHandler(void)
{
    static uint8_t cnt_turn = 0;
    static uint8_t cnt_speed = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        // 1ms 时间基准：给直立环动态 dt 使用
        PID_Timebase1ms_Tick();

        // 10ms 转向环节拍
        cnt_turn++;
        if (cnt_turn >= 10)
        {
            cnt_turn = 0;
            g_flag_turn = 1;
        }

        // 20ms 速度环节拍
        cnt_speed++;
        if (cnt_speed >= 20)
        {
            cnt_speed = 0;
            g_flag_speed = 1;
        }

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
