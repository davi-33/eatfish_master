/**
 * @file gimbal.h
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#define __GIMBAL_H__
#ifdef __GIMBAL_H__

#include "pid.h"
#include "stdint.h"
#include "dm_driver.h"
// yaw是5，pitch是6（新东西）
// yaw是6，pitch是5（老东西）
// 云台电机数据
//#define PITCH_MOTOR CAN_1_6
//#define YAW_MOTOR CAN_1_5
#define PITCH_MOTOR CAN_1_8
#define YAW_MOTOR CAN_1_7
#define PI 3.1415926f

extern pid_t yaw_absolute_speed_pid;
extern pid_t yaw_absolute_pid;

enum gimbal_status_e
{
    LOCATION = 0,
    SPEED,
    ABSOLUTE,
    zero_force,
};

struct gimbal_status
{
    // 设定云台控制模式
    enum gimbal_status_e pitch_status;
    enum gimbal_status_e yaw_status;
		enum gimbal_status_e big_yaw_status;

    struct
    {
        float set, now, last, absoulte_offset, location_offset;
        float stable;
    } pitch;
    float pitch_speed;
    float set_pitch_speed;

    struct
    {
        float set, now, last, absoulte_offset, location_offset;
        float stable;
    } yaw;
    float yaw_speed;
    float set_yaw_speed;
		
		struct
    {
        float set, now, last, absoulte_offset, location_offset;
        float stable;
    } big_yaw;
    float big_yaw_speed;
    float set_big_yaw_speed;
    float big_yaw_current;
};

extern struct gimbal_status gimbal;

// 外部调用
void gimbal_init(void);    // 初始化云台
void gimbal_pid_cal(void); // 云台PID计算

void gimbal_set_offset(float ab_pitch, float ab_yaw, float lo_pitch, float lo_yaw); // 初始化零点
void gimbal_set_pitch(float pitch, float up_angle, float down_angle);               // 设定picth角度,限幅用

float slope_calculation(float IMU_pitch, float LOCATION_pitch); // 计算地盘与地面的角度

void gimbal_clear_cnt(void);
void gimbal_set_speed(float pitch, float yaw); // 设定速度
float feedforward_contorl(float target);       // 前馈


//big_yaw
void big_yaw_init();
void big_yaw_cal();
void set_big_yaw_pos_vel(float pos, float vel,motor_t *motor);
void big_yaw_update();
void relative_angle_big_yaw_send();
void float_to_bytes(float f, uint8_t *bytes) ;
float iir_low_pass_filter(float input, float alpha);
float big_yaw_feedforward(float in);//大yaw前馈

// end of file

#endif
