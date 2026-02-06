#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "main.h"

// 定义机器人姿态结构体
typedef struct {
    float x;    // X轴坐标 (m)
    float y;    // Y轴坐标 (m)
    float yaw;  // 航向角 (弧度)
} Robot_Pose_t;

// 【关键】声明全局变量，让外部能访问
extern volatile Robot_Pose_t g_robot_pose;

// 任务入口函数声明
void StartNavTask(void const * argument);

#endif