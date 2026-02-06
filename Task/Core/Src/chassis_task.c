#include "chassis_task.h"
#include "cmsis_os.h"
#include "math.h"

// --- 引用其他模块 ---
#include "ins_task.h"     // 读坐标 (g_robot_pose)
#include "path_planner.h" // 算轨迹 (Bezier, S-Curve)
#include "pid_dsp.h"      // 算PID (arm_pid_f32)
#include "motor.h"        // 控电机 (Set_PWM)

// --- 全局变量 ---
volatile uint8_t g_is_moving = 0; // 默认静止

// 导航参数
static Point2D p0, p1, p2, p3; // 贝塞尔控制点
static float g_nav_progress = 0.0f; // 当前进度 0.0~1.0

// 定义 PID 结构体 (左右轮速度环 + 航向环)
static Motor_PID_t pid_spd_L;
static Motor_PID_t pid_spd_R;
static Motor_PID_t pid_yaw;

// ================= 外部接口：开始跑 S 曲线 =================
void Nav_Start_S_Curve(Point2D start, Point2D end) {
    // 1. 设置起点和终点
    p0 = start;
    p3 = end;

    // 2. 自动生成中间控制点 (生成 S 形)
    // 简单策略：在起点前方 0.5m 处设 p1，终点后方 0.5m 处设 p2
    // 这里为了简化，假设车主要沿 Y 轴跑，稍微做一点 X 轴偏移来避障
    // (实际比赛中这几个点可能需要你微调)
    p1.x = start.x + 0.2f; p1.y = start.y + 0.5f; 
    p2.x = end.x - 0.2f;   p2.y = end.y - 0.5f;

    // 3. 重置状态
    g_nav_progress = 0.0f;
    g_is_moving = 1; // 开始动！
}

// ================= 任务主循环 (50Hz) =================
void StartChassisTask(void const * argument) {
    // 1. 初始化 PID (参数需要实测微调)
    // 速度环: P=2.0, I=0.1
    PID_DSP_Init(&pid_spd_L, 2.0f, 0.1f, 0.0f, 1000.0f); 
    PID_DSP_Init(&pid_spd_R, 2.0f, 0.1f, 0.0f, 1000.0f);
    // 航向环: P=5.0 (发现偏了猛回正)
    PID_DSP_Init(&pid_yaw,   5.0f, 0.0f, 0.0f, 200.0f);
	// 1. 开启 PWM 通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    // 2. 【关键】TIM1 是高级定时器，必须开启“主输出使能”(MOE)
    // 没有这句，TIM1 这种高级定时器只会计数，不会给引脚发波！
    __HAL_TIM_MOE_ENABLE(&htim1);
    for(;;) {
        if (g_is_moving) {
            // --- A. 轨迹计算 ---
            // 算出当前应该在哪 (Target Point)
            Point2D target_pt = Bezier_Calc(p0, p1, p2, p3, g_nav_progress);
            
            // 算出当前应该跑多快 (S-Curve)
            float target_v = SCurve_Get_Target_Speed(g_nav_progress, 0.4f); // 最大0.4m/s

            // --- B. 误差计算 (Pure Pursuit 纯追踪简化版) ---
            float dx = target_pt.x - g_robot_pose.x;
            float dy = target_pt.y - g_robot_pose.y;
            
            // 计算期望朝向
            float target_yaw = atan2f(dy, dx);
            float yaw_err = target_yaw - g_robot_pose.yaw;
            
            // 角度归一化 (-PI ~ PI)
            if(yaw_err > 3.14159f) yaw_err -= 6.28318f;
            if(yaw_err < -3.14159f) yaw_err += 6.28318f;

            // --- C. PID 计算 ---
            // 1. 航向环：算出角速度补偿
            float w_fix = PID_DSP_Calc(&pid_yaw, 0.0f, -yaw_err);

            // 2. 速度分配 (差速转弯)
            float v_ref_L = target_v - w_fix;
            float v_ref_R = target_v + w_fix;

            // 3. 速度环：算出 PWM
            // 注意：这里调用 motor.h 里的 Get_Speed
            float pwm_L = PID_DSP_Calc(&pid_spd_L, v_ref_L, Get_Speed_L());
            float pwm_R = PID_DSP_Calc(&pid_spd_R, v_ref_R, Get_Speed_R());

            // --- D. 输出到底层 ---
            // 这里假设你的 motor.c 里有一个 Set_Motor_PWM 函数
            // 如果你用的是你截图里的结构体，这里改成调用你的 Set_PWM(&motor_L, pwm_L)
            Set_Motor_PWM((int16_t)pwm_L, (int16_t)pwm_R);

            // --- E. 进度更新 ---
            float dist_to_target = sqrtf(dx*dx + dy*dy);
            if (dist_to_target < 0.1f) { // 追上目标点了，推进度
                g_nav_progress += 0.01f; // 步长
                if (g_nav_progress >= 1.0f) {
                    g_is_moving = 0; // 跑完了
                    Set_Motor_PWM(0, 0); // 停车
                }
            }
        } else {
            // 空闲状态：停车
            Set_Motor_PWM(0, 0);
        }

        osDelay(20); // 50Hz 控制频率
    }
}