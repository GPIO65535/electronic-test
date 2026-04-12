#include "PID.h"

static float g_target_pitch_from_speed = 0.0f;   // 速度环给直立环的目标俯仰角
float g_speed_filt = 0.0f;                       // 车速滤波值
static float g_turn_output = 0.0f;               // 转向输出
static float g_up_dt = DT_UP;                    // 直立环实际周期
static volatile uint32_t g_pid_ms_tick = 0;      // 1ms 系统节拍

#define DT_UP_MIN       0.003f
#define DT_UP_MAX       0.015f
#define DT_UP_TIMEOUT   0.03f

static float g_base_v_ref = 0.0f;
uint8_t g_run_dir = 0;                           // 0: 逆时针 1: 顺时针
volatile uint8_t g_task_mode = 0;
volatile uint8_t g_cross_line_cnt = 0;
volatile uint8_t g_is_turning_180 = 0;

float g_lap_dist = 0.0f;                         // 本圈累计里程
float g_stop_coast_dist = 24000.0f;              // 滑行开始里程
float g_stop_target_dist = 28200.0f;             // 最终停车里程

static float g_target_yaw = 0.0f;                // 掉头目标航向角
static uint8_t g_task3_cross_armed = 0;
static uint8_t g_gray_cnt = 0;
volatile uint8_t g_task3_state_dbg = 0;
static float g_task3_stage_dist = 0.0f;

#define TASK3_MIN_DIST_TO_C   480.0f
#define TASK3_MIN_DIST_TO_D   480.0f

static PID_t PID_upstruct;
static PID_t PID_speedstruct;
static PID_t PID_turnstruct;

typedef enum
{
    TASK3_WAIT_LEAVE_A = 0,
    TASK3_WAIT_D_PASS,
    TASK3_WAIT_C,
    TASK3_TURNING_C,
    TASK3_WAIT_D,
    TASK3_TURNING_D,
    TASK3_WAIT_STOP
} Task3State_t;

static Task3State_t g_task3_state = TASK3_WAIT_LEAVE_A;

static void Task3_SyncDebugState(void)
{
    g_task3_state_dbg = (uint8_t)g_task3_state;
}

static float AbsF(float x)
{
    return (x >= 0.0f) ? x : -x;
}

void PID_SetBaseSpeedRef(float v_ref)
{
    g_base_v_ref = v_ref;
    Task3_SyncDebugState();
}

/**
 * 初始化全部硬件
 */
void ALL_Init(void)
{
    GraySensor_Init();
    Encoder_Init();
    Motor_Init();
    WIT_Init(115200);
    Task3_SyncDebugState();
}

/**
 * 通用 PID 计算
 * pid: PID 结构体
 * dt: 采样周期
 */
float PID_Cal(PID_t *pid, float dt)
{
    if (dt <= 0.0f)
    {
        return pid->output;
    }

    pid->error = pid->target - pid->actual;
    pid->integral += pid->error * dt;
    pid->derivative = (pid->error - pid->last_error) / dt;
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;
    pid->last_error = pid->error;
    return pid->output;
}

/**
 * PID 输出限幅
 */
float PID_Limit(float value, float min, float max)
{
    if (value > max)
    {
        value = max;
    }
    else if (value < min)
    {
        value = min;
    }
    return value;
}

/**
 * 1ms 时间基准累加
 */
void PID_Timebase1ms_Tick(void)
{
    g_pid_ms_tick++;
}

/**
 * 获取 1ms 时间基准
 */
uint32_t PID_Timebase1ms_Get(void)
{
    return g_pid_ms_tick;
}

/**
 * 更新直立环实际周期
 */
void PID_Up_UpdateDt(float dt)
{
    if (dt <= 0.0f)
    {
        return;
    }

    if (dt > DT_UP_TIMEOUT)
    {
        return;
    }

    g_up_dt = PID_Limit(dt, DT_UP_MIN, DT_UP_MAX);
}

/**
 * 直立环
 */
void PID_Up(void)
{
    static uint8_t pid_up_init = 0;
    static uint8_t imu_tick_init = 0;
    static uint32_t last_imu_ms = 0;
    float pidup_output;
    float dt_used;
    float left_pwm;
    float right_pwm = 0.0f;
    uint32_t now_ms;
    uint32_t frame_dt_ms;

    if (!WIT_RxFrameReady)
    {
        return;
    }
    WIT_RxFrameReady = 0;

    now_ms = PID_Timebase1ms_Get();
    if (imu_tick_init)
    {
        frame_dt_ms = now_ms - last_imu_ms;
        PID_Up_UpdateDt((float)frame_dt_ms * 0.001f);
    }
    last_imu_ms = now_ms;
    imu_tick_init = 1;

    if (!pid_up_init)
    {
        PID_upstruct.kp = 10.5f * 0.6f;
        PID_upstruct.ki = 0.0f;
        PID_upstruct.kd = 0.13f * 0.6f;
        PID_upstruct.target = 0.0f;
        PID_upstruct.actual = 0.0f;
        PID_upstruct.error = 0.0f;
        PID_upstruct.last_error = g_target_pitch_from_speed - Pitch;
        PID_upstruct.integral = 0.0f;
        PID_upstruct.derivative = 0.0f;
        PID_upstruct.output = 0.0f;
        pid_up_init = 1;
    }

    PID_upstruct.target = g_target_pitch_from_speed-0.3f;
    PID_upstruct.actual = Pitch;

    dt_used = PID_Limit(g_up_dt, DT_UP_MIN, DT_UP_MAX);
    pidup_output = PID_Cal(&PID_upstruct, dt_used);
    pidup_output = PID_Limit(pidup_output, -99.0f, 99.0f);
    left_pwm = PID_Limit(pidup_output - g_turn_output, -99.0f, 99.0f);
    right_pwm = PID_Limit(pidup_output + g_turn_output, -99.0f, 99.0f);

    Motor_SetPWM(1, (int8_t)left_pwm);
    Motor_SetPWM(2, (int8_t)right_pwm);
}

/**
 * 获取当前车速
 */
static float Get_CarSpeed(void)
{
    float v_l = Encoder_Get(1);
    float v_r = Encoder_Get(2);
    return -(v_l + v_r) * 0.5f;
}

/**
 * 估计当前弯道强度，用于弯道预减速
 */
static float Get_TurnPreviewLevel(void)
{
    static const float weight[8] = {-3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f};
    uint8_t gray[8];
    uint8_t i;
    uint8_t cnt = 0;
    float sum = 0.0f;
    float e_norm;
    float edge_term;
    float width_term;
    float preview;

    GraySensor_ReadAll(gray);
    for (i = 0; i < 8; i++)
    {
#if LINE_IS_LOW
        if (gray[i] == 0)
#else
        if (gray[i] == 1)
#endif
        {
            sum += weight[i];
            cnt++;
        }
    }

    if (cnt == 0)
    {
        return 0.0f;
    }

    e_norm = AbsF(sum / (float)cnt) / 3.5f;
    if (e_norm > 1.0f)
    {
        e_norm = 1.0f;
    }

    edge_term = 0.0f;
#if LINE_IS_LOW
    if ((gray[0] == 0) || (gray[7] == 0))
#else
    if ((gray[0] == 1) || (gray[7] == 1))
#endif
    {
        edge_term = 1.0f;
    }

    width_term = (8.0f - (float)cnt) / 8.0f;
    preview = 0.60f * e_norm + 0.25f * edge_term + 0.15f * width_term;
    return PID_Limit(preview, 0.0f, 1.0f);
}

/**
 * 获取灰度误差，并处理考题模式的标记线状态机
 */
static float Get_Grayerror(void)
{
    static const float weight[8] = {-3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f};
    static float last_e = 0.0f;
    static uint8_t last_cross_state = 0;
    static uint32_t last_cross_ms = 0;

    uint8_t gray[8];
    uint8_t i;
    uint8_t cnt = 0;
    uint8_t left_on = 0;
    uint8_t right_on = 0;
    uint8_t raw_cross_state;
    uint8_t current_cross_state;
    float sum = 0.0f;
    float current_e;

    GraySensor_ReadAll(gray);
    for (i = 0; i < 8; i++)
    {
#if LINE_IS_LOW
        if (gray[i] == 0)
#else
        if (gray[i] == 1)
#endif
        {
            sum += weight[i];
            cnt++;
        }
    }

    if (cnt == 0)
    {
        g_gray_cnt = 0;
        return last_e;
    }

    g_gray_cnt = cnt;
    current_e = sum / (float)cnt;

#if LINE_IS_LOW
    if ((gray[0] == 0) && (gray[1] == 0))
    {
        left_on = 1;
    }
    if ((gray[6] == 0) && (gray[7] == 0))
    {
        right_on = 1;
    }
#else
    if ((gray[0] == 1) && (gray[1] == 1))
    {
        left_on = 1;
    }
    if ((gray[6] == 1) && (gray[7] == 1))
    {
        right_on = 1;
    }
#endif

    if (g_task_mode == 3)
    {
        raw_cross_state = (cnt >= 7 && left_on && right_on) ? 1 : 0;
        current_cross_state = raw_cross_state;
    }
    else
    {
        raw_cross_state = (left_on && right_on) ? 1 : 0;
        current_cross_state = raw_cross_state;
    }

    if ((g_task_mode == 3) && !g_is_turning_180)
    {
        if (!g_task3_cross_armed)
        {
            if (!current_cross_state)
            {
                g_task3_cross_armed = 1;
                if (g_task3_state == TASK3_WAIT_LEAVE_A)
                {
                    g_task3_state = TASK3_WAIT_D_PASS;
                    Task3_SyncDebugState();
                }
            }
        }
        else if (current_cross_state && !last_cross_state)
        {
            if ((PID_Timebase1ms_Get() - last_cross_ms) > 350)
            {
                g_cross_line_cnt++;
                last_cross_ms = PID_Timebase1ms_Get();

                if (g_task3_state == TASK3_WAIT_D_PASS)
                {
                    g_task3_state = TASK3_WAIT_C;
                    g_task3_stage_dist = 0.0f;
                    Task3_SyncDebugState();
                }
                else if (g_task3_state == TASK3_WAIT_C)
                {
                    if (g_task3_stage_dist >= TASK3_MIN_DIST_TO_C)
                    {
                        g_is_turning_180 = 1;
                        PID_speedstruct.integral = 0.0f;
                        PID_speedstruct.output = 0.0f;
                        g_target_yaw = Yaw - 180.0f;
                        if (g_target_yaw <= -180.0f)
                        {
                            g_target_yaw += 360.0f;
                        }
                        g_task3_state = TASK3_TURNING_C;
                        Task3_SyncDebugState();
                    }
                }
                else if (g_task3_state == TASK3_WAIT_D)
                {
                    if (g_task3_stage_dist >= TASK3_MIN_DIST_TO_D)
                    {
                        g_is_turning_180 = 1;
                        PID_speedstruct.integral = 0.0f;
                        PID_speedstruct.output = 0.0f;
                        g_target_yaw = Yaw - 180.0f;
                        if (g_target_yaw <= -180.0f)
                        {
                            g_target_yaw += 360.0f;
                        }
                        g_task3_state = TASK3_TURNING_D;
                        Task3_SyncDebugState();
                    }
                }
                else if (g_task3_state == TASK3_WAIT_STOP && g_cross_line_cnt >= 3)
                {
                    g_lap_dist = 0.0f;
                    g_stop_coast_dist = 190.0f;
                    g_stop_target_dist = 230.0f;
                }
            }
        }
    }

    last_cross_state = current_cross_state;

    if (left_on && right_on)
    {

            current_e = PID_Limit(current_e, -0.75f, 0.75f);
    

        if ((current_e - last_e) > 1.0f)
        {
            current_e = last_e + 1.0f;
        }
        else if ((current_e - last_e) < -1.0f)
        {
            current_e = last_e - 1.0f;
        }
    }

    last_e = current_e;
    return last_e;
}

/**
 * 清零速度环和任务状态
 */
void PID_ClearSpeedState(void)
{
    Encoder_Get(1);
    Encoder_Get(2);

    g_speed_filt = 0.0f;
    g_lap_dist = 0.0f;
    g_cross_line_cnt = 0;
    g_is_turning_180 = 0;
    g_target_yaw = 0.0f;
    g_task3_cross_armed = 0;
    g_task3_stage_dist = 0.0f;
    g_task3_state = TASK3_WAIT_LEAVE_A;
    Task3_SyncDebugState();

    PID_speedstruct.integral = 0.0f;
    PID_speedstruct.error = 0.0f;
    PID_speedstruct.target = 0.0f;
    PID_speedstruct.output = 0.0f;

    PID_upstruct.integral = 0.0f;
    PID_upstruct.error = 0.0f;
    PID_upstruct.target = 0.0f;
    PID_upstruct.output = 0.0f;

    PID_turnstruct.error = 0.0f;
    PID_turnstruct.integral = 0.0f;
    PID_turnstruct.target = 0.0f;
    PID_turnstruct.output = 0.0f;
    PID_turnstruct.derivative = 0.0f;
    PID_turnstruct.last_error = 0.0f - Get_Grayerror();

    g_base_v_ref = 0.0f;
}

/**
 * 速度环
 */
void PID_Speed(void)
{
    static uint8_t pid_speed_init = 0;
    static float turn_brake_filt = 0.0f;
    static uint8_t g_stop_state = 0; // 0: 正常 1: 滑行 2: 刹停
    float alpha = 0.2f;
    float v;
    float preview_now;
    float speed_target_now;
    float pid_output;

    if (!pid_speed_init)
    {
        PID_speedstruct.kp = 0.3f;
        PID_speedstruct.ki = 0.15f / 200.0f;
        PID_speedstruct.kd = 0.0f;
        PID_speedstruct.target = 0.0f;
        PID_speedstruct.actual = 0.0f;
        PID_speedstruct.error = 0.0f;
        PID_speedstruct.last_error = 0.0f;
        PID_speedstruct.integral = 0.0f;
        PID_speedstruct.derivative = 0.0f;
        PID_speedstruct.output = 0.0f;
        pid_speed_init = 1;
    }

    v = Get_CarSpeed();
    g_speed_filt = alpha * v + (1.0f - alpha) * g_speed_filt;

    g_lap_dist += AbsF(v);
    if (g_task_mode == 3)
    {
        g_task3_stage_dist += AbsF(v);
    }

    if (g_stop_state == 0)
    {
        if (g_lap_dist > g_stop_coast_dist)
        {
            g_stop_state = 1;
        }
    }
    if (g_stop_state == 1)
    {
        if (g_lap_dist > g_stop_target_dist)
        {
            g_stop_state = 2;
        }
    }

    preview_now = Get_TurnPreviewLevel();
    if (preview_now > turn_brake_filt)
    {
        turn_brake_filt = turn_brake_filt * 0.6f + preview_now * 0.4f;
    }
    else
    {
        turn_brake_filt = turn_brake_filt * 0.96f + preview_now * 0.04f;
    }

    speed_target_now = g_base_v_ref * (1.0f - 0.45f * turn_brake_filt);
    if ((g_task_mode == 3) && g_is_turning_180)
    {
        speed_target_now = 0.0f;
    }

    if (g_stop_state == 2)
    {
        PID_speedstruct.kp = 0.3f;
        PID_speedstruct.ki = 0.07f;
        PID_speedstruct.target = 0.0f;
        if (AbsF(g_speed_filt) < 2.0f)
        {
            PID_speedstruct.integral = PID_Limit(PID_speedstruct.integral, -0.2f, 0.2f);
            PID_speedstruct.kp = 0.15f;
            if (AbsF(g_speed_filt) < 1.0f)
            {
                PID_speedstruct.integral = 0.0f;
                PID_speedstruct.output = 0.0f;
            }
        }
        else
        {
            PID_speedstruct.integral = PID_Limit(PID_speedstruct.integral, -15.0f, 15.0f);
        }
    }
    else if (g_stop_state == 1)
    {
        PID_speedstruct.kp = 0.3f;
        PID_speedstruct.ki = 0.15f / 200.0f;
        PID_speedstruct.target = speed_target_now * 0.5f;
        PID_speedstruct.integral = PID_Limit(PID_speedstruct.integral, -15.0f, 15.0f);
    }
    else
    {
        PID_speedstruct.kp = 0.3f;
        PID_speedstruct.ki = 0.15f / 200.0f;
        PID_speedstruct.target = speed_target_now;
        PID_speedstruct.integral = PID_Limit(PID_speedstruct.integral, -15.0f, 15.0f);
    }

    if (g_base_v_ref == 0.0f)
    {
        PID_speedstruct.integral = 0.0f;
        PID_speedstruct.output = 0.0f;
        PID_speedstruct.target = 0.0f;
    }

    PID_speedstruct.actual = g_speed_filt;
    pid_output = PID_Cal(&PID_speedstruct, DT_SPEED);
    g_target_pitch_from_speed = PID_Limit(pid_output, -3.5f, 3.5f);
}

/**
 * 转向环
 */
void PID_Turn(void)
{
    static uint8_t pid_turn_init = 0;
    static PID_t PID_yawturnstruct;
    static uint16_t ok_cnt = 0;
    float out;

    if (!pid_turn_init)
    {
        PID_turnstruct.kp = 4.3f;
        PID_turnstruct.ki = 0.0f;
        PID_turnstruct.kd = 0.34f;
        PID_turnstruct.target = 0.0f;
        PID_turnstruct.actual = 0.0f;
        PID_turnstruct.error = 0.0f;
        PID_turnstruct.integral = 0.0f;
        PID_turnstruct.derivative = 0.0f;
        PID_turnstruct.output = 0.0f;

        PID_yawturnstruct.kp = 2.3f;
        PID_yawturnstruct.ki = 0.0f;
        PID_yawturnstruct.kd = 0.25f;
        PID_yawturnstruct.target = 0.0f;
        PID_yawturnstruct.actual = 0.0f;
        PID_yawturnstruct.error = 0.0f;
        PID_yawturnstruct.last_error = 0.0f;
        PID_yawturnstruct.integral = 0.0f;
        PID_yawturnstruct.derivative = 0.0f;
        PID_yawturnstruct.output = 0.0f;

        pid_turn_init = 1;
    }

    if ((g_task_mode == 3) && g_is_turning_180)
    {
        float error = g_target_yaw - Yaw;

        while (error > 180.0f)
        {
            error -= 360.0f;
        }
        while (error < -180.0f)
        {
            error += 360.0f;
        }

        PID_yawturnstruct.target = 0.0f;
        PID_yawturnstruct.actual = error;
        out = PID_Cal(&PID_yawturnstruct, DT_TURN);
        g_turn_output = PID_Limit(out, -60.0f, 60.0f);

        if (AbsF(error) < 5.0f)
        {
            ok_cnt++;
            if (ok_cnt > 20)
            {
                g_is_turning_180 = 0;
                ok_cnt = 0;

                if (g_task3_state == TASK3_TURNING_C)
                {
                    g_run_dir = 1;
                    g_task3_state = TASK3_WAIT_D;
                    g_task3_stage_dist = 0.0f;
                    Task3_SyncDebugState();
                }
                else if (g_task3_state == TASK3_TURNING_D)
                {
                    g_run_dir = 0;
                    g_task3_state = TASK3_WAIT_STOP;
                    g_task3_stage_dist = 0.0f;
                    Task3_SyncDebugState();
                }
            }
        }
        else
        {
            ok_cnt = 0;
        }
    }
    else
    {
        PID_turnstruct.target = 0.0f;
        PID_turnstruct.actual = Get_Grayerror();
        out = PID_Cal(&PID_turnstruct, DT_TURN);
        PID_turnstruct.integral = PID_Limit(PID_turnstruct.integral, -100.0f, 100.0f);
        g_turn_output = PID_Limit(out, -35.0f, 35.0f);
    }
}

void TIM1_UP_IRQHandler(void)
{
    static uint8_t cnt_turn = 0;
    static uint8_t cnt_speed = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        PID_Timebase1ms_Tick();

        cnt_turn++;
        if (cnt_turn >= 10)
        {
            cnt_turn = 0;
            g_flag_turn = 1;
        }

        cnt_speed++;
        if (cnt_speed >= 20)
        {
            cnt_speed = 0;
            g_flag_speed = 1;
        }

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
