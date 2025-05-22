/**
 * @file gimbal.c
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "gimbal.h"
#include "dm_driver.h"
#include "chassis_move.h"
#include "Global_status.h"

#include "IMU_updata.h"
#include "CAN_receive&send.h"

#include "Stm32_time.h"
#include "stm32f4xx_hal.h"

#include "math.h"
#include "slope.h"
struct gimbal_status gimbal;
/* 大yaw陀螺仪pid*/

pid_t imu_big_yaw_control_speed;
pid_t imu_big_yaw_control_tor;
Slope slope_big_yaw;

/*前馈参数*/
float KF = 50.0f;
// 目标值
float Target = 0.0f;
float Pre_Target = 0.0f;
// 实际值
float Now = 0.0f;
float forward_out;
/*停在坡上参数*/
float slope;
/*飞坡固定pitch的相关参数*/
float pitch_fly_angle;

float relative_angle_big_yaw;
float imu_bigyaw_dif;
float dm4310_vel_set;
float Time_delay_trs;
float Torque_coefficient = 1.0f;

// 定义一个静态变量来保存上一次的输出值
static float prev_output = 0.0;
float filtered_tor;

float planning_big_yaw_speed = 0.0;

// 云台初始化
void gimbal_init()
{
	/*大yaw陀螺仪pid*/
	
//	pid_set(&imu_big_yaw_control_speed,2.5f, 0.0, 10.0f, 5.0f, 1.0f);
//  pid_set(&imu_big_yaw_control_speed,1.9f, 0.0, 0.0f, 3.0f, 1.0f);
//  pid_set(&imu_big_yaw_control_tor, 1.4f, 0.0, 0.0f, 3.0f, 1.0f);
  pid_set(&imu_big_yaw_control_speed,1.8f, 0.0f, 0.0f, 3.0f, 0.1f);
  pid_set(&imu_big_yaw_control_tor, 0.74f, 0.0f, 0.0f, 3.0f, 0.1f);
	Slope_set(&slope_big_yaw,1.0,2.0,1);
	/*云台相关参数初始化*/

	/* 云台模式*/
	
}

// 零点设置，在main.c中被调用
void gimbal_set_offset(float ab_pitch, float ab_yaw, float lo_pitch, float lo_yaw)
{
	gimbal.pitch.absoulte_offset = ab_pitch;
	gimbal.yaw.absoulte_offset = ab_yaw;
	gimbal.pitch.location_offset = lo_pitch;
	gimbal.yaw.location_offset = lo_yaw;
}

float slope_calculation(float IMU_pitch, float LOCATION_pitch)
{
	slope = rad2degree(IMU_pitch) - LOCATION_pitch - 48.5;
	return slope;
}

/*****************************big行为控制********************************/
void big_yaw_init()
{
	dm4310_init();											//ID和模式初始化

//	dm4310_enable(&hcan2,&dm4310);
	
//	dm4310_ctrl_send(&hcan2,&dm4310);
	
//	dm4310.cmd.kp_set = 5 ;
//	dm4310.cmd.kd_set = 0 ;
//	dm4310.cmd.pos_set = 0 ;
//	dm4310.cmd.tor_set = 0 ;
	
//	dm4310.ctrl.pos_set = IMU_data.AHRS.yaw; //in use
//	dm4310.ctrl.vel_set = 1.0 ;								//in use
	
	dm4310.ctrl.tor_set = 0.0 ;
  /******************************/
//  dm4310.ctrl.kp_set = 1.0f;
//  dm4310.ctrl.kd_set = 1.0f;
	/******************************/
	
//	dm4310_set(&dm4310);
}
void big_yaw_cal()
{

	//MIT模式
/*************************************************************************************************************/
//双环
//  iir_low_pass_filter(chassis.speed_RC.big_yaw,0.1);
	gimbal.big_yaw_speed = cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0];
//	gimbal.set_big_yaw_speed =  pid_cal(&imu_big_yaw_control_speed,IMU_data.AHRS.yaw_rad_cnt-90.0/180.0*PI,chassis.speed_RC.big_yaw);
  gimbal.set_big_yaw_speed =  pid_cal(&imu_big_yaw_control_speed,IMU_data.AHRS.yaw_rad_cnt,chassis.speed_RC.big_yaw);//chassis.speed_RC.big_yaw
	
	planning_big_yaw_speed = Slope_Cal(&slope_big_yaw,gimbal.big_yaw_speed,gimbal.set_big_yaw_speed);
	
  gimbal.big_yaw_current = pid_cal(&imu_big_yaw_control_tor,gimbal.big_yaw_speed,planning_big_yaw_speed);
	dm4310.ctrl.tor_set = Torque_coefficient * gimbal.big_yaw_current;
/*************************************************************************************************************/
////  //输出给tor_set不合理，tor_set是前馈力矩
//  gimbal.big_yaw_speed = cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0];
//  dm4310.ctrl.kp_set = 1.0f;
//  dm4310.ctrl.kd_set = 1.0f;
//  dm4310.ctrl.tor_set = 0.0f;
//  dm4310.ctrl.pos_set = pid_cal(&imu_big_yaw_control_speed,IMU_data.AHRS.yaw_rad_cnt,chassis.speed_RC.big_yaw);
//  dm4310.ctrl.vel_set = pid_cal(&imu_big_yaw_control_tor,gimbal.big_yaw_speed,dm4310.ctrl.pos_set);
  
/*************************************************************************************************************/
////单环
//	gimbal.big_yaw_speed = cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0];
//	dm4310.ctrl.tor_set = pid_cal(&imu_big_yaw_control_tor,gimbal.big_yaw_speed,chassis.speed_RC.big_yaw);

	/************************************************************/
//////	set_big_yaw_pos_vel(chassis.speed_RC.big_yaw,dm4310.ctrl.vel_set,&dm4310);//in use
//	if(Get_sys_time_ms() - Time_delay_trs >2)
//	{
		dm4310_ctrl_send(&hcan2,&dm4310);		
//		Time_delay_trs = Get_sys_time_ms();
//	}
//	dm4310_ctrl_send(&hcan2,&dm4310);		//4310发送控制数据

// /* 1 to 4 */
// 	gimbal.big_yaw_speed = cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0];
// 	gimbal.set_big_yaw_speed =  pid_cal(&imu_big_yaw_control_speed,IMU_data.AHRS.yaw_rad_cnt-90.0/180.0*PI,chassis.speed_RC.big_yaw);//chassis.speed_RC.big_yaw
// 	gimbal.big_yaw_current = pid_cal(&imu_big_yaw_control_tor,gimbal.big_yaw_speed,gimbal.set_big_yaw_speed);
// 	set_motor()
}

void big_yaw_update()
{
	dm4310.ctrl.pos_set = IMU_data.AHRS.yaw_rad_cnt;
}

void set_big_yaw_pos_vel(float pos, float vel,motor_t *motor)
{
	dm4310.ctrl.pos_set = pos ;
	dm4310.ctrl.vel_set = vel ;
}

void relative_angle_big_yaw_send()
{
//	relative_angle_big_yaw = dm4310.para.pos/PI*180 + 2.0 ;
  relative_angle_big_yaw = dm4310.para.pos/PI*180.0f + 50.0f  ;
	
	uint8_t can_send_data [8];
	static CAN_TxHeaderTypeDef tx_message;
	uint32_t send_mail_box;

	tx_message.StdId = 0x05;
//  tx_message.StdId = 0x303;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	
	float_to_bytes(relative_angle_big_yaw, &can_send_data[0]);  // 将x的字节存入can_data[0]~can_data[3]
	float chassis_imu_data_yaw = IMU_data.AHRS.yaw;
	float_to_bytes(chassis_imu_data_yaw, &can_send_data[4]);
		
	HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
}

// 将float按字节拆分
void float_to_bytes(float f, uint8_t *bytes) 
	{
    uint32_t *p = (uint32_t *)&f;  // 将float指针强制转换为uint32_t指针
    uint32_t temp = *p;            // 获取float的二进制表示
    bytes[0] = (temp >> 0) & 0xFF; // 提取最低字节
    bytes[1] = (temp >> 8) & 0xFF; // 提取第二个字节
    bytes[2] = (temp >> 16) & 0xFF; // 提取第三个字节
    bytes[3] = (temp >> 24) & 0xFF; // 提取最高字节
  }
/*****************************big行为控制********************************/
// 一阶 IIR 低通滤波函数
float iir_low_pass_filter(float input, float alpha) 
{
    float output;
    // 根据差分方程计算当前输出
    output = alpha * input + (1 - alpha) * prev_output;
    // 更新上一次的输出值
    prev_output = output;
    return output;
}
//大yaw前馈
float last_in = 0.0;
float T = 0.001;
float big_yaw_feedforward(float in)
{
  float out;
//  out = (in - last_in)/T + in;
  out = (in - last_in)/T ;
  last_in = in;
  return out;
}
/*********************以下函数没有被用到，保留以防以后用到***********************/

void gimbal_clear_cnt(void)
{
	clear_motor_cnt(YAW_MOTOR);
}
// 对数函数
float lnx(float a, float b, float c, float d, float x)
{
	return a * logf(b * fabs(x) + c) + d;
}

// 设定速度
void gimbal_set_speed(float pitch, float yaw)
{
	gimbal.yaw_status = gimbal.pitch_status = SPEED; // 以速度模式控制

	gimbal.set_pitch_speed = pitch;
	gimbal.set_yaw_speed = yaw;
}

// 未成形的前馈
float feedforward_contorl(float target)
{
	float f_out;
	f_out = (target - Pre_Target) * KF;
	Pre_Target = Target;
	return f_out;
}
