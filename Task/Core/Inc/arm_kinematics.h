#ifndef __ARM_KINEMATICS_H
#define __ARM_KINEMATICS_H

#include "main.h"

// ================= 机械臂真实尺寸 (基于您的测量数据) =================
#define ARM_L1_LENGTH    78.0f    // 大臂长度 (mm)
#define ARM_L2_LENGTH    80.5f    // 小臂长度 (mm)
#define ARM_BASE_HEIGHT  98.5f    // 底座高度 (mm)

// ================= 安全限制 =================
// 理论最大臂长 158.5mm，限制在 155mm 以内防止机械死点
#define ARM_MAX_REACH    155.0f   

// ================= 函数声明 =================
void Arm_Init(void);  // 初始化
void Arm_Move_Line(float x, float y, float z, uint32_t duration_ms); // 直线插补移动
void Arm_Claw_Control(uint8_t state); // 1=松开, 0=夹紧

#endif