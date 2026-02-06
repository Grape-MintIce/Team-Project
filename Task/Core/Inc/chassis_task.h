#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "path_planner.h" // 需要用到 Point2D 类型

// 状态标志位：1=正在跑, 0=跑完了
extern volatile uint8_t g_is_moving;

// 【核心接口】供 MissionTask 调用，启动一次 S 形移动
void Nav_Start_S_Curve(Point2D start, Point2D end);

// 任务入口函数声明
void StartChassisTask(void const * argument);

#endif