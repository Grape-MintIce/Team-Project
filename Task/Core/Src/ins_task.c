#include "ins_task.h"
#include "mpu6050.h"   // 记得引用你自己写的 mpu6050 库
#include "motor.h"     // 引用 motor 库以读取速度
#include "math.h"
#include "cmsis_os.h"

// 全局坐标变量 (外部可读)
// 初始状态：在 A 区中心 (0.25, 0.25)，车头朝 X 轴 (0弧度)
volatile Robot_Pose_t g_robot_pose = {0.25f, 0.25f, 0.0f};

void StartNavTask(void const * argument) {
    float dt = 0.005f; // 200Hz
    
    // 初始化
    MPU6050_Init();
    HAL_Delay(500); // 等待传感器稳定
    
    // 记录上电时的零偏 (软件归零)
    // 假设上电时一定要把车头摆正，对着 X 轴方向！
    float yaw_startup_bias = MPU6050_Get_Yaw(); 

    for(;;) {
        // --- 1. 获取航向 (强制要求: 必须用 IMU) ---
        MPU6050_Update_Yaw(dt); // 积分更新
        float raw_yaw = MPU6050_Get_Yaw();
        
        // 绝对角度 = (当前读数 - 初始偏差)
        g_robot_pose.yaw = raw_yaw - yaw_startup_bias;

        // --- 2. 获取编码器速度 (m/s) ---
        // 假设 motor.c 里已经补全了这两个函数
        float v_left = Get_Speed_L();  
        float v_right = Get_Speed_R();
        float v_center = (v_left + v_right) / 2.0f;

        // --- 3. 里程计推算 (Dead Reckoning) ---
        // ds = v * dt (这段时间走过的微小距离)
        float ds = v_center * dt;
        
        // 分解到 X, Y 轴
        g_robot_pose.x += ds * cosf(g_robot_pose.yaw);
        g_robot_pose.y += ds * sinf(g_robot_pose.yaw);

        osDelay(5); // 5ms = 200Hz
    }
}