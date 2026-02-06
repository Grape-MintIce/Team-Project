#ifndef __PATH_PLANNER_H
#define __PATH_PLANNER_H

#include "main.h"

// 定义二维点结构体 (用于坐标传参)
typedef struct {
    float x;
    float y;
} Point2D;

// 贝塞尔曲线计算函数
Point2D Bezier_Calc(Point2D p0, Point2D p1, Point2D p2, Point2D p3, float t);

// S-Curve 速度规划函数
float SCurve_Get_Target_Speed(float progress, float max_speed);

#endif