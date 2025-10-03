#ifndef MATH_LIB
#define MATH_LIB
#include <math.h>
#include <stdint.h>
/**
 * @file    math_lib.h
 * @brief   自定义数学库
 * @author  SYSU电控组
 * @date    2025-09-24
 * @version 1.0
 * 
 * @note    提供高效的数学运算宏定义和常量
 */

// ===================== 数学常量 =====================
#define M_PI           3.14159265358979323846f   ///< π
#define M_PI_2         1.57079632679489661923f   ///< π/2
#define M_PI_4         0.78539816339744830962f  ///< π/4
#define M_2PI          6.28318530717958647692f  ///< 2π

#define MATH_DEG2RAD   (M_PI / 180.0f)       ///< 度转弧度系数
#define MATH_RAD2DEG   (180.0f / M_PI)        ///< 弧度转度系数

// ===================== 单位转换 =====================
#define DEG_TO_RAD(deg) ((deg) * MATH_DEG2RAD)   ///< 度转弧度
#define RAD_TO_DEG(rad) ((rad) * MATH_RAD2DEG)   ///< 弧度转度
#define RPM_TO_RAD(rpm) ((rpm) * MATH_2PI / 60.0f) ///< RPM转弧度/秒
#define RAD_TO_RPM(rad) ((rad) * 60.0f / MATH_2PI) ///< 弧度/秒转RPM

// ===================== 数学函数宏 =====================
/// 绝对值
#define MATH_ABS(x) ((x) < 0 ? -(x) : (x))

/// 限制值在[min, max]范围内
#define MATH_LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/// 平方
#define MATH_SQ(x) ((x) * (x))

/// 立方
#define MATH_CUBE(x) ((x) * (x) * (x))

/// 快速平方根倒数近似 (Quake III算法)
#define MATH_INV_SQRT(x) ({ \
    float _x = (x); \
    float _xhalf = 0.5f * _x; \
    int32_t _i = *(int32_t*)&_x; \
    _i = 0x5f3759df - (_i >> 1); \
    _x = *(float*)&_i; \
    _x = _x * (1.5f - _xhalf * _x * _x); \
    _x; \
})

/// 快速平方根近似
#define MATH_SQRT(x) (1.0f / MATH_INV_SQRT(x))


// ===================== 常用函数 =====================
static inline float wrap_angle_360(float angle)
{
    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }
    while (angle < 0.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static inline float wrap_angle_180(float angle)
{
    angle = wrap_angle_360(angle + 180.0f) - 180.0f;
    return angle;
}

static inline float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}




#endif // MATH_LIB