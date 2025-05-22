/**
 * @file alg_slope.h
 * @author yssickjgd (1345578933@qq.com)
 * @author davi
 * @brief 斜坡函数, 用于速度规划等
 * @date 2025.5.5
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

#ifndef __SLOPE_H
#define __SLOPE_H

/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 规划优先类型, 分为目标值优先和真实值优先
 * 目标值优先, 即硬规划
 * 真实值优先, 即当前真实值夹在当前规划值和目标值之间, 当前规划值转为当前真实值
 *
 */
typedef enum {
    Slope_First_REAL = 0,
    Slope_First_TARGET
} Enum_Slope_First;

/**
 * @brief Reusable, 斜坡函数本体
 *
 */
typedef struct {
    // 输出值
    float Out;

    // 规划优先类型
    Enum_Slope_First Slope_First;
    // 当前规划值
    float Now_Planning;
    // 当前真实值
    float Now_Real;

    // 绝对值增量, 一次计算周期改变值
    float Increase_Value;
    // 绝对值减量, 一次计算周期改变值
    float Decrease_Value;
    // 目标值
    float Target;
} Slope;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化斜坡函数
 *
 * @param slope 斜坡函数结构体指针
 * @param __Increase_Value 绝对值增量
 * @param __Decrease_Value 绝对值减量
 * @param __Slope_First 规划优先类型
 */
void Slope_set(Slope *slope, float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First);

/**
 * @brief 获取输出值
 *
 * @param slope 斜坡函数结构体指针
 * @return 输出值
 */
float Slope_Get_Out(Slope *slope);

/**
 * @brief 设定当前规划值
 *
 * @param slope 斜坡函数结构体指针
 * @param __Now_Real 当前规划值
 */
void Slope_Set_Now_Real(Slope *slope, float __Now_Real);

/**
 * @brief 设定绝对值增量, 一次计算周期改变值
 *
 * @param slope 斜坡函数结构体指针
 * @param __Increase_Value 绝对值增量, 一次计算周期改变值
 */
void Slope_Set_Increase_Value(Slope *slope, float __Increase_Value);

/**
 * @brief 设定绝对值减量, 一次计算周期改变值
 *
 * @param slope 斜坡函数结构体指针
 * @param __Decrease_Value 绝对值减量, 一次计算周期改变值
 */
void Slope_Set_Decrease_Value(Slope *slope, float __Decrease_Value);

/**
 * @brief 设定目标值
 *
 * @param slope 斜坡函数结构体指针
 * @param __Target 目标值
 */
void Slope_Set_Target(Slope *slope, float __Target);

/**
 * @brief 计算周期回调函数
 *
 * @param slope 斜坡函数结构体指针
 */
void Slope_Calculate_PeriodElapsedCallback(Slope *slope);


float Math_Abs(float x);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/