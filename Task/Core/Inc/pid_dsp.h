#ifndef __PID_DSP_H
#define __PID_DSP_H

#include "main.h"
#include "arm_math.h" // 【关键】引用 DSP 库，Keil 里必须开启 FPU

typedef struct {
    arm_pid_instance_f32 dsp_pid; // CMSIS-DSP 库自带的 PID 结构体
    float out_limit;              // 输出限幅
} Motor_PID_t;

// 初始化函数
void PID_DSP_Init(Motor_PID_t *pid, float kp, float ki, float kd, float limit);

// 计算函数
float PID_DSP_Calc(Motor_PID_t *pid, float ref, float fdb);

#endif