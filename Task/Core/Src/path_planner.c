#include "path_planner.h"
#include "math.h"

// 贝塞尔公式计算
Point2D Bezier_Calc(Point2D p0, Point2D p1, Point2D p2, Point2D p3, float t) {
    Point2D p;
    float u = 1.0f - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;

    // B(t) = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3
    p.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
    p.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
    return p;
}

// 梯形/S曲线速度规划
// progress: 0.0 (起点) ~ 1.0 (终点)
float SCurve_Get_Target_Speed(float progress, float max_speed) {
    // 简单梯形规划：前15%加速，后15%减速，中间匀速
    if (progress < 0.15f) {
        return max_speed * (progress / 0.15f);
    } else if (progress > 0.85f) {
        return max_speed * ((1.0f - progress) / 0.15f);
    } else {
        return max_speed;
    }
}