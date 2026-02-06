#include "mission_task.h"
#include "chassis_task.h"   // 包含 Nav_Start_S_Curve
#include "arm_kinematics.h" // 包含机械臂控制
#include "cmsis_os.h"

// 定义关键坐标点
// 起点 A
Point2D P_Start = {0.25f, 0.25f}; 
// 抓取点 B (停靠点)
Point2D P_PickB = {0.86f, 0.86f};
// 抓取点 C (停靠点)
Point2D P_PickC = {1.36f, 1.86f};
// 放置点 D (停靠点)
Point2D P_DropD = {1.73f, 2.73f};

// 中间控制点 (用于生成 S 形轨迹，避开直线上障碍)
// 假设障碍在 A-C 直线中间，我们在 Y 轴方向偏移一点绕过去
Point2D P_Control_Left  = {0.5f, 1.5f}; // 向左绕
Point2D P_Control_Right = {1.5f, 0.5f}; // 向右绕

void StartMissionTask(void const * argument) {
    osDelay(3000); // 上电等待 IMU 校准和机械臂归位
    Arm_Init();    // 机械臂复位
	osDelay(2000); // 多等2秒，看见机械臂动了之后赶紧撤离！

    // ================= 任务 1: A -> B (S形) =================
    // 假设用简单的 S 形控制点: 起点前方和终点后方
    Nav_Start_S_Curve(P_Start, P_PickB); 
    while(g_is_moving) osDelay(100); // 等待到达

    // ================= 任务 2: 抓取 B =================
    // 强制要求3: 输入坐标逆解
    // 车在(0.86,0.86), 方块在(1.0,1.0), 相对距离约 20cm
    Arm_Move_Line(0, 200, 50, 1000); // 伸出到上方 (x=0, y=200mm, z=50mm)
    Arm_Claw_Control(1);             // 张开
    Arm_Move_Line(0, 200, 15, 1000); // 下降 (z=15mm 方块中心高度)
    Arm_Claw_Control(0);             // 闭合
    osDelay(500);
    Arm_Move_Line(0, 150, 150, 1000); // 提起

    // ================= 任务 3: B -> D (放置) =================
    Nav_Start_S_Curve(P_PickB, P_DropD);
    while(g_is_moving) osDelay(100);

    // 放置动作
    Arm_Move_Line(0, 200, 50, 1000); // 放到指定位置
    Arm_Claw_Control(1);             // 松开
    Arm_Move_Line(0, 150, 150, 1000); // 收回

    // ================= 任务 4: D -> A (返回) =================
    Nav_Start_S_Curve(P_DropD, P_Start);
    while(g_is_moving) osDelay(100);

    // ================= 任务 5: A -> C (去第二个点) =================
    Nav_Start_S_Curve(P_Start, P_PickC);
    while(g_is_moving) osDelay(100);

    // ================= 任务 6: 抓取 C =================
    Arm_Move_Line(0, 200, 50, 1000);
    Arm_Claw_Control(1);
    Arm_Move_Line(0, 200, 15, 1000);
    Arm_Claw_Control(0);
    osDelay(500);
    Arm_Move_Line(0, 150, 150, 1000);

    // ================= 任务 7: C -> D (堆叠) =================
    Nav_Start_S_Curve(P_PickC, P_DropD);
    while(g_is_moving) osDelay(100);

    // ================= 任务 8: 堆叠放置 =================
    // 注意高度! 第一块高3cm，所以第二块要放到 z = 30mm + 15mm = 45mm 的高度
    Arm_Move_Line(0, 200, 48, 1000); // 稍微高一点点(48mm)避免撞击
    Arm_Claw_Control(1);             // 松开
    Arm_Move_Line(0, 150, 150, 1000); // 收回

    // 任务结束，返回 A
    Nav_Start_S_Curve(P_DropD, P_Start);
    
    for(;;) osDelay(1000); // 停机
}