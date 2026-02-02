#include "motor.h"

Motor_t motor_L;
Motor_t motor_R;

// 内部函数：设置PWM和方向
static void Set_PWM(Motor_t *m, float output) {
    // 1. 限幅 (PWM最大值4199)
    if (output > 4199.0f) output = 4199.0f;
    if (output < -4199.0f) output = -4199.0f;

    // 2. 设置方向
    if (output >= 0) {
        HAL_GPIO_WritePin(m->dir_port1, m->dir_pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->dir_port2, m->dir_pin2, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(m->dir_port1, m->dir_pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir_port2, m->dir_pin2, GPIO_PIN_SET);
        output = -output; // 取绝对值
    }

    // 3. 赋值PWM
    __HAL_TIM_SET_COMPARE(m->pwm_tim, m->pwm_channel, (uint32_t)output);
}

void Motor_Init(void) {
    // --- 左轮配置 (PE9/PE2/PE3/TIM2) ---
    motor_L.pwm_tim = &htim1;
    motor_L.pwm_channel = TIM_CHANNEL_1;
    motor_L.enc_tim = &htim2;
    motor_L.dir_port1 = GPIOE; motor_L.dir_pin1 = GPIO_PIN_2;
    motor_L.dir_port2 = GPIOE; motor_L.dir_pin2 = GPIO_PIN_3;

    // PID参数 (Kp=1.5, Ki=0.5, Kd=0)
    motor_L.pid.Kp = 1.5f;
    motor_L.pid.Ki = 0.5f;
    motor_L.pid.Kd = 0.0f;
    arm_pid_init_f32(&motor_L.pid, 1);

    // --- 右轮配置 (PE11/PE4/PE5/TIM3) ---
    motor_R.pwm_tim = &htim1;
    motor_R.pwm_channel = TIM_CHANNEL_2;
    motor_R.enc_tim = &htim3;
    motor_R.dir_port1 = GPIOE; motor_R.dir_pin1 = GPIO_PIN_4;
    motor_R.dir_port2 = GPIOE; motor_R.dir_pin2 = GPIO_PIN_5;
    
    motor_R.pid.Kp = 1.5f;
    motor_R.pid.Ki = 0.5f;
    motor_R.pid.Kd = 0.0f;
    arm_pid_init_f32(&motor_R.pid, 1);

    // --- 启动硬件 ---
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    // 必须开启主输出 (TIM1特有)
    __HAL_TIM_MOE_ENABLE(&htim1);
    
    // 开启编码器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void Motor_Update(void) {
    // 1. 读取速度 (转为short类型自动处理溢出)
    int16_t speed_L = (int16_t)__HAL_TIM_GET_COUNTER(motor_L.enc_tim);
    int16_t speed_R = (int16_t)__HAL_TIM_GET_COUNTER(motor_R.enc_tim);
    
    // 2. 清零计数器 (增量式测量)
    __HAL_TIM_SET_COUNTER(motor_L.enc_tim, 0);
    __HAL_TIM_SET_COUNTER(motor_R.enc_tim, 0);
    
    // 3. 计算PID
    float out_L = arm_pid_f32(&motor_L.pid, motor_L.target_speed - speed_L);
    float out_R = arm_pid_f32(&motor_R.pid, motor_R.target_speed - speed_R);
    
    // 4. 执行输出
    Set_PWM(&motor_L, out_L);
    Set_PWM(&motor_R, out_R);
}