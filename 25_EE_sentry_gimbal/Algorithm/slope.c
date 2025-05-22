#include "slope.h"



// 求绝对值的函数
float Math_Abs(float x)
{
    return ((x > 0) ? x : -x);
}




/**
 * @brief 初始化斜坡函数
 *
 * @param slope 斜坡函数结构体指针
 * @param __Increase_Value 绝对值增量
 * @param __Decrease_Value 绝对值减量
 * @param __Slope_First 规划优先类型
 */
void Slope_set(Slope *slope, float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First) {
    slope->Out = 0.0f;
    slope->Slope_First = __Slope_First;
    slope->Now_Planning = 0.0f;
    slope->Now_Real = 0.0f;
    slope->Increase_Value = __Increase_Value;
    slope->Decrease_Value = __Decrease_Value;
    slope->Target = 0.0f;
}

/**
 * @brief 获取输出值
 *
 * @param slope 斜坡函数结构体指针
 * @return 输出值
 */
float Slope_Get_Out(Slope *slope) {
    return slope->Out;
}

/**
 * @brief 设定当前规划值
 *
 * @param slope 斜坡函数结构体指针
 * @param __Now_Real 当前规划值
 */
void Slope_Set_Now_Real(Slope *slope, float __Now_Real) {
    slope->Now_Real = __Now_Real;
}

/**
 * @brief 设定绝对值增量, 一次计算周期改变值
 *
 * @param slope 斜坡函数结构体指针
 * @param __Increase_Value 绝对值增量, 一次计算周期改变值
 */
void Slope_Set_Increase_Value(Slope *slope, float __Increase_Value) {
    slope->Increase_Value = __Increase_Value;
}

/**
 * @brief 设定绝对值减量, 一次计算周期改变值
 *
 * @param slope 斜坡函数结构体指针
 * @param __Decrease_Value 绝对值减量, 一次计算周期改变值
 */
void Slope_Set_Decrease_Value(Slope *slope, float __Decrease_Value) {
    slope->Decrease_Value = __Decrease_Value;
}

/**
 * @brief 设定目标值
 *
 * @param slope 斜坡函数结构体指针
 * @param __Target 目标值
 */
void Slope_Set_Target(Slope *slope, float __Target) {
    slope->Target = __Target;
}

/**
 * @brief 斜坡函数调整值, 计算周期取决于调用者
 *
 * @param slope 斜坡函数结构体指针
 */
void Slope_Calculate_PeriodElapsedCallback(Slope *slope) {
    // 规划为当前真实值优先的额外逻辑
    if (slope->Slope_First == Slope_First_REAL) {
        if ((slope->Target >= slope->Now_Real && slope->Now_Real >= slope->Now_Planning) ||
            (slope->Target <= slope->Now_Real && slope->Now_Real <= slope->Now_Planning)) {
            slope->Out = slope->Now_Real;
        }
    }

    if (slope->Now_Planning > 0.0f) {
        if (slope->Target > slope->Now_Planning) {
            // 正值加速
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out += slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        } else if (slope->Target < slope->Now_Planning) {
            // 正值减速
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Decrease_Value) {
                slope->Out -= slope->Decrease_Value;
            } else {
                slope->Out = slope->Target;
            }
        }
    } else if (slope->Now_Planning < 0.0f) {
        if (slope->Target < slope->Now_Planning) {
            // 负值加速
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out -= slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        } else if (slope->Target > slope->Now_Planning) {
            // 负值减速
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Decrease_Value) {
                slope->Out += slope->Decrease_Value;
            } else {
                slope->Out = slope->Target;
            }
        }
    } else {
        if (slope->Target > slope->Now_Planning) {
            // 0值正加速
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out += slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        } else if (slope->Target < slope->Now_Planning) {
            // 0值负加速
            if (Math_Abs(slope->Now_Planning - slope->Target) > slope->Increase_Value) {
                slope->Out -= slope->Increase_Value;
            } else {
                slope->Out = slope->Target;
            }
        }
    }

    // 善后工作
    slope->Now_Planning = slope->Out;
} 
