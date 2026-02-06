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
// ============================================================
// 【新增】适配层代码：将新任务的调用转发给你的结构体对象
// ============================================================

// 1. 速度控制接口
void Set_Motor_PWM(int16_t pwm_l, int16_t pwm_r) {
    // 调用你原本写好的静态函数 Set_PWM
    // 注意：这里的 motor_L 和 motor_R 是你文件头部定义的全局变量
    Set_PWM(&motor_L, (float)pwm_l);
    Set_PWM(&motor_R, (float)pwm_r);
}

// 2. 左轮速度读取接口
float Get_Speed_L(void) {
    // 从左轮的定时器句柄中读取计数
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(motor_L.enc_tim);
    __HAL_TIM_SET_COUNTER(motor_L.enc_tim, 0); // 清零

    // 简单转换：假设 1560线/圈，半径 0.0325m，周期 5ms (0.005s)
    // Speed = count / PPR * 周长 / 时间
    return (float)count / 1560.0f * (2.0f * 3.14159f * 0.0325f) / 0.005f;
}

// 3. 右轮速度读取接口
float Get_Speed_R(void) {
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(motor_R.enc_tim);
    __HAL_TIM_SET_COUNTER(motor_R.enc_tim, 0); 
    
    // 右轮如果反装，可能需要加负号： return -(float)count ...
    return (float)count / 1560.0f * (2.0f * 3.14159f * 0.0325f) / 0.005f;
}