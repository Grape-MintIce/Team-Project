#include "arm_kinematics.h"
#include "tim.h"         // 引用 htim4
#include "cmsis_os.h"    // 引用 osDelay
#include "math.h"

#define PI 3.1415926f
#define RAD2DEG(x) ((x) * 180.0f / PI)

// 舵机 PWM 范围 (500-2500 对应 0-180度)
#define SERVO_MIN  500
#define SERVO_MAX  2500

// 记录当前末端位置 (用于插补计算起点)
static float cur_x = 0;
static float cur_y = 120; // 初始前伸距离
static float cur_z = 100; // 初始高度

// ================= 内部函数：设置舵机角度 =================
// id: 1=底座, 2=左臂, 3=右臂, 4=爪子
void Set_Servo_Angle(uint8_t id, float angle) {
    // 角度限幅
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // 映射到 PWM 值
    uint16_t pwm = (uint16_t)(SERVO_MIN + (angle / 180.0f) * (SERVO_MAX - SERVO_MIN));

    // 根据您的接线 (TIM4 CH1~CH4)
    switch(id) {
        case 1: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm); break; // 底座
        case 2: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm); break; // 左舵机 (小臂绝对)
        case 3: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm); break; // 右舵机 (大臂)
        case 4: __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm); break; // 爪子
    }
}

// ================= 核心算法：并联结构逆运动学解算 =================
// 输入：目标坐标 (x, y, z)
// 输出：三个舵机的目标角度 (ang_base, ang_small_abs, ang_big)
// 返回：0=成功, 1=失败(够不着)
uint8_t Arm_Solve_IK(float x, float y, float z, float *ang_base, float *ang_small_abs, float *ang_big) {
    // 1. 计算底座旋转角 (Yaw)
    // atan2 返回弧度，转为角度
    *ang_base = RAD2DEG(atan2f(y, x));
    
    // 2. 将三维坐标投影到二维平面 (r, z_local)
    // r 是目标点在水平面上的投影距离
    float r = sqrtf(x*x + y*y);
    // z_local 是相对于机械臂肩膀关节的高度
    float z_local = z - ARM_BASE_HEIGHT;
    
    // 计算肩膀到目标的直线距离
    float L_target = sqrtf(r*r + z_local*z_local);
    
    // 检查是否超出最大臂长 (预留 2mm 缓冲)
    if (L_target > (ARM_L1_LENGTH + ARM_L2_LENGTH - 2.0f)) {
        return 1; // 够不着，报错
    }

    // 3. 计算大臂角度 (利用余弦定理)
    // cos_alpha = (a^2 + b^2 - c^2) / 2ab
    float numer = ARM_L1_LENGTH*ARM_L1_LENGTH + L_target*L_target - ARM_L2_LENGTH*ARM_L2_LENGTH;
    float denom = 2.0f * ARM_L1_LENGTH * L_target;
    float cos_alpha = numer / denom;
    
    // 防止浮点数误差导致 acos 输入超出范围 [-1, 1]
    if (cos_alpha > 1.0f) cos_alpha = 1.0f;
    else if (cos_alpha < -1.0f) cos_alpha = -1.0f;
    
    float alpha = acosf(cos_alpha); // 大臂与目标连线的夹角 (弧度)
    float phi = atan2f(z_local, r); // 目标连线与水平面的夹角 (弧度)
    
    // 大臂目标角度 (相对于水平面)
    float theta_big_rad = phi + alpha;
    *ang_big = RAD2DEG(theta_big_rad);

    // 4. 计算小臂绝对角度 (关键步骤：并联结构解算)
    // 先算出肘关节 (Elbow) 的坐标
    float elbow_r = ARM_L1_LENGTH * cosf(theta_big_rad);
    float elbow_z = ARM_L1_LENGTH * sinf(theta_big_rad);
    
    // 手腕 (Wrist) 坐标就是 r, z_local
    // 计算小臂向量 (Wrist - Elbow)
    float vec_r = r - elbow_r;
    float vec_z = z_local - elbow_z;
    
    // 计算小臂向量的角度
    float theta_small_rad = atan2f(vec_z, vec_r);
    *ang_small_abs = RAD2DEG(theta_small_rad);

    return 0; // 解算成功
}

// ================= 初始化函数 =================
void Arm_Init(void) {
    // 启动 PWM 输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    // 初始状态：松开爪子，移动到高处待命
    Arm_Claw_Control(1); 
    // 移动到 (x=0, y=120, z=150) 的位置，耗时 1000ms
    Arm_Move_Line(0.0f, 120.0f, 150.0f, 1000); 
}

// ================= 直线插补控制 (带 FreeRTOS 延时) =================
void Arm_Move_Line(float x, float y, float z, uint32_t duration_ms) {
    float start_x = cur_x;
    float start_y = cur_y;
    float start_z = cur_z;
    
    // 分解动作步数 (每 20ms 走一步)
    int steps = duration_ms / 20;
    if (steps < 1) steps = 1;

    float a_base, a_small, a_big;

    for (int i = 1; i <= steps; i++) {
        // 1. 线性插值计算当前时刻的目标坐标
        float t = (float)i / (float)steps;
        float next_x = start_x + (x - start_x) * t;
        float next_y = start_y + (y - start_y) * t;
        float next_z = start_z + (z - start_z) * t;

        // 2. 调用逆运动学解算角度
        if (Arm_Solve_IK(next_x, next_y, next_z, &a_base, &a_small, &a_big) == 0) {
            // 3. 解算成功，驱动舵机
            Set_Servo_Angle(1, a_base);
            Set_Servo_Angle(2, a_small); // 左舵机 (小臂绝对)
            Set_Servo_Angle(3, a_big);   // 右舵机 (大臂)
        }
        
        // 4. 任务延时，让出 CPU 给底盘任务
        osDelay(20); 
    }

    // 更新当前位置记录
    cur_x = x; 
    cur_y = y; 
    cur_z = z;
}

// ================= 爪子控制 =================
void Arm_Claw_Control(uint8_t state) {
    if (state == 1) {
        Set_Servo_Angle(4, 90.0f);  // 松开 (角度根据实际调试修改)
    } else {
        Set_Servo_Angle(4, 160.0f); // 夹紧
    }
    osDelay(300); // 等待动作完成
}