/**
 * @file    MahonyAHRS.c
 * @brief   标准Mahony互补滤波算法
 * 用于融合陀螺仪和加速度计数据解算姿态
 */
#include "MahonyAHRS.h"
#include <math.h>

// 算法参数
#define sampleFreq  1000.0f          // 采样频率 (Hz)
#define twoKpDef    (2.0f * 0.5f)   // 2 * Kp (比例增益)
#define twoKiDef    (2.0f * 0.001f) // 2 * Ki (积分增益)

// 变量定义
static float twoKp = twoKpDef;
static float twoKi = twoKiDef;
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // 积分误差项

/**
 * @brief   倒数平方根算法 (Quake III 经典算法)
 */
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief   Mahony AHRS 更新函数 (6轴: 陀螺仪 + 加速度计)
 * @param   gx, gy, gz: 陀螺仪数据 (单位: rad/s)
 * @param   ax, ay, az: 加速度计数据 (单位: 任意，算法内会归一化，推荐 m/s^2)
 * @param   dt: 运行周期 (s)
 */
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;  //  归一化系数（向量模的倒数）
    float halfvx, halfvy, halfvz;  // 估计的重力向量的一半（机体坐标系）
    float halfex, halfey, halfez;  // 误差向量的一半（测量值与估计值的叉积）
    float qa, qb, qc;  // 四元数临时变量

    // 1. 如果加速度计数据无效(例如处于失重状态)，则只进行陀螺仪积分
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 2. 加速度计数据归一化
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 3. 估计重力方向 (通过当前四元数推算)
        // 注意：这里计算的是重力向量的一半，为了减少乘法运算
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 4. 计算误差 (测量重力 vs 估计重力 的 叉积)
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 5. 误差积分 (消除稳态误差，即消除陀螺仪零漂)
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 6. 误差比例反馈 (修正陀螺仪)
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 7. 四元数微分方程积分 (一阶龙格库塔法)
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 8. 四元数归一化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/**
 * @brief   获取四元数
 */
void Mahony_GetQuaternion(float *q) {
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

/**
 * @brief   四元数转欧拉角 (Z-Y-X 顺序)
 * @param   pitch, roll, yaw: 输出角度 (单位: 弧度)
 */
void Mahony_GetEulerAngle(float *pitch, float *roll, float *yaw) {
    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    *pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}