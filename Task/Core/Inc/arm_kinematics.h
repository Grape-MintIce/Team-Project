#ifndef __ARM_KINEMATICS_H
#define __ARM_KINEMATICS_H

#include "main.h"

// ================= 机械臂核心参数 (已填入你测量的数值) =================
#define ARM_L1       78.0f    // 大臂长度 (mm)
#define ARM_L2       80.5f    // 小臂长度 (mm)
#define ARM_BASE_H   98.5f    // 底座高度 (mm)

// ================= 舵机中位与方向定义 =================
// 1. 底座 (Yaw)
#define PWM_BASE_CENTER 1500
#define PWM_BASE_DIR    1      // 1: PWM变大向左

// 2. 大臂 (Right Servo)
#define PWM_BIG_CENTER  1500   
#define PWM_BIG_DIR     1      // 1: PWM变大向后

// 3. 小臂 (Left Servo)
#define PWM_SMALL_CENTER 1500  
#define PWM_SMALL_DIR    -1    // -1: PWM变大压下

// 4. 爪子
#define CLAW_OPEN       1000
#define CLAW_CLOSE      2000

// ================= 函数声明 =================
void Arm_Init(void);
void Arm_Claw_Control(uint8_t open); // 1=开, 0=合
void Arm_Reset(void); // 回归中位

// 【瞬移函数】直接给PWM，速度最快，用于内部计算或快速复位
void Arm_Set_Pose_Instant(float x, float y, float z);

// 【平滑移动函数】画直线，速度可控，用于抓取
// duration_ms: 动作耗时(毫秒)，建议 500~2000
void Arm_Move_Line(float target_x, float target_y, float target_z, uint16_t duration_ms);

#endif