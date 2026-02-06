#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

// MPU6050 I2C地址 (AD0接地)
#define MPU6050_ADDR 0xD0

// 函数声明
uint8_t MPU6050_Init(void);        // 初始化传感器
void MPU6050_Update_Yaw(float dt); // 周期性调用，计算角度
float MPU6050_Get_Yaw(void);       // 获取算好的角度 (弧度制)

#endif