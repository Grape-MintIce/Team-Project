#include "mpu6050.h"
#include "i2c.h"  // 包含 hi2c1 的定义
#include <math.h>

// 引用 CubeMX 生成的 I2C 句柄
extern I2C_HandleTypeDef hi2c1;

// 全局变量：记录当前的总角度 (弧度)
static float g_total_yaw_rad = 0.0f;
static float g_gyro_z_offset = 0.0f; // 零漂误差

// 写寄存器辅助函数
static void MPU_Write(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, 1, &data, 1, 100);
}

// 读寄存器辅助函数
static void MPU_Read(uint8_t reg, uint8_t* data, uint8_t len) {
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, 1, data, len, 100);
}

// ================= 初始化函数 =================
uint8_t MPU6050_Init(void) {
    uint8_t check;
    uint8_t data;

    // 1. 检查设备 ID (WHO_AM_I)
    MPU_Read(0x75, &check, 1);
    if (check != 0x68) {
        return 1; // 初始化失败 (线没接好?)
    }

    // 2. 唤醒传感器 (PWR_MGMT_1 写 0)
    MPU_Write(0x6B, 0x00);

    // 3. 配置陀螺仪量程 (GYRO_CONFIG)
    // 设置为 ±2000 dps (对应寄存器位 0x18)
    // 灵敏度 = 16.4 LSB/(度/秒)
    MPU_Write(0x1B, 0x18);
    
    // 4. 简单的上电校准 (计算零漂)
    // 此时车必须静止！
    float sum = 0;
    int16_t raw_z;
    uint8_t buf[2];
    for(int i=0; i<200; i++) {
        MPU_Read(0x47, buf, 2); // 读取 GYRO_ZOUT_H 和 L
        raw_z = (int16_t)(buf[0] << 8 | buf[1]);
        sum += raw_z;
        HAL_Delay(2);
    }
    g_gyro_z_offset = sum / 200.0f; // 记录下静止时的平均误差

    return 0; // 成功
}

// ================= 核心：更新角度 =================
// dt: 距离上一次调用的时间 (秒)，例如 0.005
void MPU6050_Update_Yaw(float dt) {
    uint8_t buf[2];
    int16_t raw_z;
    float gyro_z_dps; 
    float gyro_z_rad; 

    // 1. 读取 Z 轴原始数据
    MPU_Read(0x47, buf, 2);
    raw_z = (int16_t)(buf[0] << 8 | buf[1]);

    // 2. 减去零漂
    float clean_z = (float)raw_z - g_gyro_z_offset;
    
    // 3. 简单的“死区”过滤 (去除极小的静止抖动)
    if (clean_z > -5.0f && clean_z < 5.0f) clean_z = 0.0f;

    // 4. 转换物理单位
    // 量程 ±2000dps -> 灵敏度 16.4
    gyro_z_dps = clean_z / 16.4f; 
    
    // 5. 转换为弧度 (1度 = 0.01745 弧度)
    gyro_z_rad = gyro_z_dps * 0.0174533f;

    // 6. 积分累加 (角度 = 角速度 × 时间)
    g_total_yaw_rad += gyro_z_rad * dt;
}

// ================= 获取当前角度接口 =================
float MPU6050_Get_Yaw(void) {
    return g_total_yaw_rad;
}