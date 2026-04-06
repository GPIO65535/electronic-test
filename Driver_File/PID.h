#ifndef __PID_H__
#define __PID_H__ 
#include"stm32f10x.h"
#include"Encoder.h"
#include"Motor.h"
#include"Wit.h"
#include"GraySensor.h"
#include"Timer.h"
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
}PID_t;



#define LINE_IS_LOW 1 //1:巡线低电平有效 
#define DT_UP      0.005f   // 直立环周期 5ms
#define DT_SPEED   0.020f   // 速度环周期 20ms
#define DT_TURN    0.010f   // 转向环周期 10ms

extern volatile uint8_t g_flag_turn;
extern volatile uint8_t g_flag_speed;
extern float g_speed_filt;

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
#endif

