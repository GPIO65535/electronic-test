#ifndef __PID_H__
#define __PID_H__

#include "stm32f10x.h"
#include "Encoder.h"
#include "Motor.h"
#include "Wit.h"
#include "GraySensor.h"
#include "Timer.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float target;
    float actual;
    float error;
    float last_error;
    float integral;
    float derivative;
    float output;
} PID_t;

#define LINE_IS_LOW 1   // 1: 巡线为低电平有效
#define DT_UP      0.005f
#define DT_SPEED   0.020f
#define DT_TURN    0.010f

extern volatile uint8_t g_flag_turn;
extern volatile uint8_t g_flag_speed;
extern float g_speed_filt;
extern float g_lap_dist;
extern float g_stop_coast_dist;
extern float g_stop_target_dist;
extern uint8_t g_run_dir;
extern volatile uint8_t g_task_mode;
extern volatile uint8_t g_cross_line_cnt;
extern volatile uint8_t g_is_turning_180;
extern volatile uint8_t g_task3_state_dbg;

void ALL_Init(void);
float PID_Cal(PID_t *pid, float dt);
float PID_Limit(float value, float min, float max);
void PID_Timebase1ms_Tick(void);
uint32_t PID_Timebase1ms_Get(void);
void PID_Up_UpdateDt(float dt);
void PID_Up(void);
void PID_Speed(void);
void PID_Turn(void);
void PID_SetBaseSpeedRef(float v_ref);
void PID_ClearSpeedState(void);

#endif
