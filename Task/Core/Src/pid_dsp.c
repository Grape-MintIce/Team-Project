#include "pid_dsp.h"

// 初始化 PID 参数
void PID_DSP_Init(Motor_PID_t *pid, float kp, float ki, float kd, float limit) {
    // 1. 设置 DSP 库参数
    pid->dsp_pid.Kp = kp;
    pid->dsp_pid.Ki = ki;
    pid->dsp_pid.Kd = kd;
    
    // 2. 调用 DSP 库自带的初始化 (第二个参数 1 代表复位状态)
    arm_pid_init_f32(&pid->dsp_pid, 1); 
    
    // 3. 设置我们需要用的限幅
    pid->out_limit = limit;
}

// 计算 PID 输出
float PID_DSP_Calc(Motor_PID_t *pid, float ref, float fdb) {
    float error = ref - fdb;
    
    // 【核心】调用 FPU 硬件加速计算 PID
    float out = arm_pid_f32(&pid->dsp_pid, error);
    
    // 输出限幅
    if (out > pid->out_limit) out = pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;
    
    return out;
}