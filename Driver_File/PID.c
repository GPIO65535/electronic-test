#include"PID.h"
static float g_target_pitch_from_speed=0.0f; // 速度环输出的目标角度，供直立环使用
static float g_speed_filt=0.0f;//速度测量滤波
static float g_turn_output=0.0f;//转速偏差
static float g_up_dt=DT_UP;//直立环动态dt
static volatile uint32_t g_pid_ms_tick=0;//1ms系统节拍

#define DT_UP_MIN       0.003f
#define DT_UP_MAX       0.015f
#define DT_UP_TIMEOUT   0.03f
static float AbsF(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float g_base_v_ref=0.0f;

void PID_SetBaseSpeedRef(float v_ref)
{
    g_base_v_ref=v_ref;
}



/**
 * 功能：全局初始化
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
 * 功能：PID计算
 * 参数：pid - PID结构体指针，dt - 时间步长
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
 * 功能：PID限幅
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
 * 功能：1ms计数的计数
 */
void PID_Timebase1ms_Tick(void)
{
    g_pid_ms_tick++;
}

/**
 * 功能：获取当前1ms计数的计数
 */
uint32_t PID_Timebase1ms_Get(void)
{
    return g_pid_ms_tick;
}

/**
 * 功能：更新直立环实际控制周期
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
 * 功能：平衡车直立环
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
        PID_upstruct.kp=6.5*0.6f;
        PID_upstruct.ki=0.0f;
        PID_upstruct.kd=0.3*0.6f;
        PID_upstruct.target=0.0f;
        PID_upstruct.actual=0.0f;
        PID_upstruct.error=0.0f;
        PID_upstruct.last_error=0.0f;
        PID_upstruct.integral=0.0f;
        PID_upstruct.derivative=0.0f;
        PID_upstruct.output=0.0f;
        pid_up_init = 1;
    }

    PID_upstruct.target = g_target_pitch_from_speed;      // 目标直立角，由速度环实时偏置给出
    PID_upstruct.actual = Pitch;

    dt_used = PID_Limit(g_up_dt, DT_UP_MIN, DT_UP_MAX);
    pidup_output=PID_Cal(&PID_upstruct,dt_used); // 使用动态dt，减少单帧延迟影响
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
    return -(v_l+v_r)*0.5f;
}

/*
 * 弯道前瞻强度估计：误差越大、越靠边、有效点越少，越接近入弯
 */
static float Get_TurnPreviewLevel(void)
{
    static const float weight[8]={-3.5f,-2.5f,-1.5f,-0.5f,0.5f,1.5f,2.5f,3.5f};
    uint8_t gray[8];
    uint8_t i,cnt=0;
    float sum=0.0f;
    float e_norm;
    float edge_term;
    float width_term;
    float preview;

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
        return 1.0f;
    }

    e_norm=AbsF(sum/(float)cnt)/3.5f;
    if(e_norm>1.0f)
    {
        e_norm=1.0f;
    }

    edge_term=0.0f;
    #if LINE_IS_LOW
        if((gray[0]==0)||(gray[7]==0))
    #else
        if((gray[0]==1)||(gray[7]==1))
    #endif
    {
        edge_term=1.0f;
    }

    width_term=(8.0f-(float)cnt)/8.0f;

    preview=0.60f*e_norm+0.25f*edge_term+0.15f*width_term;
    return PID_Limit(preview,0.0f,1.0f);
}
static uint8_t g_gray_cnt = 0;

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
        g_gray_cnt = 0;
        return last_e;
    }

    g_gray_cnt = cnt;
    last_e=sum/(float)cnt; // 计算平均误差并更新历史误差
    return last_e;
}

/*
 * 功能：平衡车速度环(外环pid,速度环输出的目标角给直立环)*/
float g_lap_dist = 0.0f; // 里程累加器（外部可见，供OLED读取）

void PID_Speed(void)
{
    static uint8_t pid_speed_init = 0;
    static float turn_brake_filt=0.0f;
    float alpha=0.2f;
    float v;
    static PID_t PID_speedstruct;
    if(!pid_speed_init)
    {
        // 完全恢复为你跑出“完美一圈”时的原本温和参数！！！
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

    // 完美跑完一圈时的代码：砍掉所有花里胡哨的目标速度干预，完全靠重心自然前倾！
    PID_speedstruct.target = 0.0f; 
    PID_speedstruct.actual = g_speed_filt;
    
    // 【全场MVP代码】：完美跑一圈时最关键的积分限幅！防止弯道差速造成积分负向死锁（消除所有倒车情况）
    PID_speedstruct.integral = PID_Limit(PID_speedstruct.integral, -15.0f, 100.0f);
    
    float pid_output=PID_Cal(&PID_speedstruct,DT_SPEED);// dt使用实际控制周期
    
    // 将速度环输出限幅后给直立环作为目标倾角
    g_target_pitch_from_speed=PID_Limit(pid_output,-5.0f,5.0f);
}
void PID_Turn(void)
{
    static PID_t PID_turnstruct;
    static uint8_t pid_turn_init=0;
    float out;
    static float e_line = 0.0f; // 增加低通滤波防止抖动
    if(!pid_turn_init)
    {
        PID_turnstruct.kp=2.2f;   // 因为设置了定速，1.5转不过来，增加到2.2增加转弯力度（且由于Kd降低了，在直线上也不会抽搐）
        PID_turnstruct.ki=0.0f;
        PID_turnstruct.kd=0.08f;  
        PID_turnstruct.target=0.0f;
        PID_turnstruct.actual=0.0f;
        PID_turnstruct.error=0.0f;
        PID_turnstruct.last_error=0.0f;
        PID_turnstruct.integral=0.0f;
        PID_turnstruct.derivative=0.0f;
        PID_turnstruct.output=0.0f;
        pid_turn_init=1;
    }
    
    // 对灰度原始误差做一次低通滤波，使这8个离散灯跳变时的曲线更平滑
    e_line = e_line * 0.3f + Get_Grayerror() * 0.7f; 

    PID_turnstruct.target=0.0f;
    PID_turnstruct.actual=e_line;
    out=PID_Cal(&PID_turnstruct,DT_TURN);
    PID_turnstruct.integral=PID_Limit(PID_turnstruct.integral,-100.0f,100.0f); // 积分限幅，防止积分饱和
    g_turn_output=PID_Limit(out,-25.0f,25.0f);
}

void TIM1_UP_IRQHandler(void)
{
    static uint8_t cnt_turn = 0;
    static uint8_t cnt_speed = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        // 1ms 时基，供直立环动态 dt 使用
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
