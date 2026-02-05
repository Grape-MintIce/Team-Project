#include "arm_kinematics.h"
#include "math.h"
#include "tim.h" // 确保包含 tim.h 以使用 htim4

#define PI 3.1415926f
#define RAD2DEG(x) ((x) * 180.0f / PI)
#define DEG2RAD(x) ((x) * PI / 180.0f)

// ================= 全局变量：记录当前坐标 =================
// 记录机械臂当前在哪里，用于插补计算
// 初始值设为 Reset 后的状态 (假设 Reset 是伸向前方)
float g_cur_x = 0.0f;
float g_cur_y = 100.0f; 
float g_cur_z = 150.0f; 

// 内部辅助: 限制 PWM 范围防止烧舵机
uint16_t Constrain_PWM(int pwm) {
    if (pwm < 500) return 500;
    if (pwm > 2500) return 2500;
    return (uint16_t)pwm;
}

// ================= 1. 初始化 =================
void Arm_Init(void) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    Arm_Reset();
}

// ================= 2. 爪子控制 =================
void Arm_Claw_Control(uint8_t open) {
    if (open)
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, CLAW_OPEN);
    else
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, CLAW_CLOSE);
}

// ================= 3. 回中位 =================
void Arm_Reset(void) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500); // 底座正前
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1500); // 大臂垂直
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1500); // 小臂水平
    Arm_Claw_Control(1); // 张开
    
    // 更新全局坐标 (大概值，用于防止下次插补跳变)
    g_cur_x = 0.0f;
    g_cur_y = 100.0f; 
    g_cur_z = 150.0f; // 这是一个估算的安全点
}

// ================= 4. 核心逆解 (瞬移版) =================
// 只计算角度并执行，同时更新全局坐标 g_cur
void Arm_Set_Pose_Instant(float x, float y, float z) {
    
    // --- 1. 计算底座旋转 (Yaw) ---
    float base_angle_rad = atan2f(x, y);
    float base_angle_deg = RAD2DEG(base_angle_rad);
    int pwm_base = PWM_BASE_CENTER + (int)(base_angle_deg * 11.11f * PWM_BASE_DIR);

    // --- 2. 坐标修正 ---
    float R_target = sqrtf(x*x + y*y); 
    float Z_target = z - ARM_BASE_H;

    // --- 3. 几何解算 ---
    float Dist = sqrtf(R_target*R_target + Z_target*Z_target);

    // [限位保护]
    if (Dist > (ARM_L1 + ARM_L2 - 2.0f)) { 
        float ratio = (ARM_L1 + ARM_L2 - 2.0f) / Dist;
        R_target *= ratio;
        Z_target *= ratio;
        Dist = ARM_L1 + ARM_L2 - 2.0f;
    }

    float cos_alpha = (ARM_L1*ARM_L1 + Dist*Dist - ARM_L2*ARM_L2) / (2 * ARM_L1 * Dist);
    if(cos_alpha > 1.0f) cos_alpha = 1.0f;
    if(cos_alpha < -1.0f) cos_alpha = -1.0f;
    float alpha = acosf(cos_alpha); 
    float phi = atan2f(Z_target, R_target);

    // --- 4. 求解目标角度 ---
    float theta1_rad = phi + alpha;
    float theta1_deg = RAD2DEG(theta1_rad);

    // 联动结构解算 (Type A)
    float y1 = ARM_L1 * cosf(theta1_rad);
    float z1 = ARM_L1 * sinf(theta1_rad);
    float theta2_rad = atan2f(Z_target - z1, R_target - y1);
    float theta2_deg = RAD2DEG(theta2_rad);

    // --- 5. 角度转 PWM ---
    int pwm_big = PWM_BIG_CENTER + (int)((theta1_deg - 90.0f) * 11.11f * PWM_BIG_DIR);
    int pwm_small = PWM_SMALL_CENTER + (int)((theta2_deg - 0.0f) * 11.11f * PWM_SMALL_DIR);

    // --- 6. 输出 ---
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Constrain_PWM(pwm_base));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Constrain_PWM(pwm_big));   
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, Constrain_PWM(pwm_small)); 
    
    // 【关键】更新当前坐标记忆
    g_cur_x = x;
    g_cur_y = y;
    g_cur_z = z;
}

// ================= 5. 平滑直线插补运动 =================
void Arm_Move_Line(float target_x, float target_y, float target_z, uint16_t duration_ms) {
    
    // 1. 规划步数 (每 20ms 走一步)
    uint16_t step_cnt = duration_ms / 20; 
    if (step_cnt < 1) step_cnt = 1; 

    // 2. 计算每一步的增量
    float dx = (target_x - g_cur_x) / (float)step_cnt;
    float dy = (target_y - g_cur_y) / (float)step_cnt;
    float dz = (target_z - g_cur_z) / (float)step_cnt;

    // 3. 循环执行小碎步
    for (uint16_t i = 0; i < step_cnt; i++) {
        float next_x = g_cur_x + dx;
        float next_y = g_cur_y + dy;
        float next_z = g_cur_z + dz;

        // 执行瞬移 (这一步很小)
        Arm_Set_Pose_Instant(next_x, next_y, next_z);

        // 延时 20ms
        HAL_Delay(20); 
    }
    
    // 4. 确保最后精确到达目标点
    Arm_Set_Pose_Instant(target_x, target_y, target_z);
    HAL_Delay(20);
}