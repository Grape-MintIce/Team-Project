#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"
#include "arm_math.h" // DSP库

typedef struct {
    // 硬件句柄
    TIM_HandleTypeDef *pwm_tim;
    uint32_t pwm_channel;
    TIM_HandleTypeDef *enc_tim;
    GPIO_TypeDef *dir_port1; uint16_t dir_pin1;
    GPIO_TypeDef *dir_port2; uint16_t dir_pin2;
    
    // PID控制数据
    arm_pid_instance_f32 pid;
    float target_speed;
    float current_speed;
} Motor_t;

// 全局变量声明
extern Motor_t motor_L;
extern Motor_t motor_R;

// 函数声明
void Motor_Init(void);
void Motor_Update(void);

#endif