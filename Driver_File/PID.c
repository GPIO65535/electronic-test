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
        
        // 【关键】：开机第一帧时，防止因为初始last_error=0导致的微分爆发
        PID_upstruct.last_error=g_target_pitch_from_speed - Pitch; 
        
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
uint8_t g_run_dir = 0; // 0:逆时针(默认) 1:顺时针
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

    if(cnt == 0)
    {
        g_gray_cnt = 0;
        return last_e;
    }

    g_gray_cnt = cnt;
    
    float current_e = sum / (float)cnt; 

    // 检测极左和极右是否同时压线
    uint8_t left_on = 0, right_on = 0;
    #if LINE_IS_LOW  
        if(gray[0]==0 || gray[1]==0) left_on = 1;
        if(gray[6]==0 || gray[7]==0) right_on = 1;
    #else        
        if(gray[0]==1 || gray[1]==1) left_on = 1;
        if(gray[6]==1 || gray[7]==1) right_on = 1;
    #endif
    if (left_on && right_on) {
        current_e = 2.5f; 
        
        if (current_e - last_e > 2.0f) {
            current_e = last_e + 2.0f; 
        } else if (current_e - last_e < -2.0f) {
            current_e = last_e - 2.0f;
        }
    }

    last_e = current_e;
    return last_e;
}

/*
 * 功能：平衡车速度环(外环pid,速度环输出的目标角给直立环)*/
float g_lap_dist = 0.0f; // 里程累加器（外部可见，供OLED读取）
float g_stop_coast_dist = 24000.0f; // 开始减速滑行的里程
float g_stop_target_dist = 28000.0f; // 彻底停车的目标里程

void PID_Speed(void)
{
    static uint8_t pid_speed_init = 0;
    static float turn_brake_filt=0.0f;
    float alpha=0.2f;
    float v;
    static PID_t PID_speedstruct;
    if(!pid_speed_init)
    {
        PID_speedstruct.kp=0.25f;
        PID_speedstruct.ki=0.25f/200.0f;
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

    // ----- 定点停车状态机（纯里程判断） -----
    static uint8_t g_stop_state = 0; // 0=盲跑阶段, 1=减速滑行, 2=彻底刹停
    
// 改为全局变量以支持按键修改

    g_lap_dist += AbsF(v);

    if (g_stop_state == 0) {
        if (g_lap_dist > g_stop_coast_dist) {
            g_stop_state = 1; // 里程达到减速点，开始滑行缓冲
        }
    }
    if (g_stop_state == 1) {
        if (g_lap_dist > g_stop_target_dist) {
            g_stop_state = 2; // 达到最终里程，触发彻底停车
        }
    }

    if(g_stop_state != 2) {
        // 如果速度跌到了负数很大（比如<-2.0，说明它为了“压住”减速动作而过度后仰导致了倒车）
        // 瞬间抛弃速度环积累的“减速积分”，防止其继续倒退
        if(g_speed_filt < -1.5f) {
            PID_speedstruct.integral = PID_Limit(PID_speedstruct.integral, 0.0f, 15.0f); // 彻底剪掉负向积分，不许后仰
        }
    }
    // ==============================================================

    if (g_stop_state == 2) {
        // 1. 停车状态：提供刹车参数，目标速度归零
        PID_speedstruct.kp = 0.4f; 
        PID_speedstruct.ki = 0.04f;
        PID_speedstruct.target = 0.0f; 
        
        // 2. 解开负向死锁封印进行大力刹车；但若速度已经降到几乎为0，就必须立刻收紧所有积分，让它老实安静地保持站立
        if (AbsF(g_speed_filt) < 2.0f) { // 速度接近停止
            PID_speedstruct.integral = PID_Limit(PID_speedstruct.integral, -0.5f, 0.5f);
            PID_speedstruct.kp = 0.15f;  // 进一步软化比例项，避免车轮震动
        } else {
            PID_speedstruct.integral = PID_Limit(PID_speedstruct.integral, -90.0f, 15.0f);
        }
    } else if (g_stop_state == 1) {
        // 【纯里程滑行】：在距离终点还有一段距离时，提前把目标速度降到龟速
        // 利用滑行减弱动能缓冲，到达目标里程瞬间就能稳稳停住，防止冲出去
        PID_speedstruct.kp = 0.15f; 
        PID_speedstruct.ki = 0.15f/200.0f;
        PID_speedstruct.target = 0.5f; // 龟速滑行 
        PID_speedstruct.integral = PID_Limit(PID_speedstruct.integral, -15.0f, 15.0f);
    } else {
        // 1. 跑圈状态：维持微弱平顺参数，目标速度接回基础速度
        turn_brake_filt = turn_brake_filt * 0.8f + Get_TurnPreviewLevel() * 0.2f; // 获取弯道前瞻
        float speed_target_now = g_base_v_ref * (1.0f - 0.5f * turn_brake_filt); // 根据弯道情况减速（最多减速50%）

        PID_speedstruct.kp = 0.25f; 
        PID_speedstruct.ki = 0.25f/200.0f;
        PID_speedstruct.target = speed_target_now; 
        // 2. 维持原来的防倒车死锁
        PID_speedstruct.integral = PID_Limit(PID_speedstruct.integral, -10.0f, 15.0f);
    }
    // ---------------------------------------------

    PID_speedstruct.actual = g_speed_filt;
    
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
        PID_turnstruct.kp=2.4f;   
        PID_turnstruct.ki=0.0f;
        PID_turnstruct.kd=0.1f;  
        PID_turnstruct.target=0.0f;
        PID_turnstruct.actual=0.0f;
        PID_turnstruct.error=0.0f;
        PID_turnstruct.last_error=0.0f;
        PID_turnstruct.integral=0.0f;
        PID_turnstruct.derivative=0.0f;
        PID_turnstruct.output=0.0f;
        pid_turn_init=1;
    }
    
    // 加强低通滤波，极大降低灰度离散跳变造成的“直道左右抽搐”
    e_line = e_line * 0.7f + Get_Grayerror() * 0.3f; 
    float abs_e = AbsF(e_line);
    PID_turnstruct.kp = 1.2f + 0.6f * abs_e; 
    PID_turnstruct.kd = 0.15f + 0.08f * abs_e;

    PID_turnstruct.target=0.0f;
    PID_turnstruct.actual=e_line;
    out=PID_Cal(&PID_turnstruct,DT_TURN);
    PID_turnstruct.integral=PID_Limit(PID_turnstruct.integral,-100.0f,100.0f); // 积分限幅，防止积分饱和
    g_turn_output=PID_Limit(out,-35.0f,35.0f);
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
