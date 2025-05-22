
/**
 * @file chassis_move.c
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */

 #include "stdlib.h"
 #include "stdio.h"
 #include "stdint.h"
 
 #include "CAN_receive&send.h"
 #include "IMU_updata.h"
 #include "cap_ctl.h"
 
 #include "chassis_move.h"
 #include "gimbal.h"
 #include "Global_status.h"
 
 #include "Stm32_time.h"
 
 #include "math.h"
 #include "RampFunc.h" // 引入斜坡函数。
 #include "slope.h"
 // wheel conf
 #define WHEEL_RADIUS 0.26282f // m
 #define Rotation_radius  0.37169f  //m
 #define PI 3.1415926f
 // car conf
 #define ROLLER_DISTANCE 100 // mm  轴距
 #define WHEELS_DISTANCE 100 // mm  轮距
 // mm/s
 #define FR 0
 #define FL 1
 #define BL 2
 #define BR 3
 //零点定义
 #define TURN_FR_ANGLE 7560   //7560
 #define TURN_FL_ANGLE 4097   //
 #define TURN_BL_ANGLE 5435   //122.18
 #define TURN_BR_ANGLE 6181		//6181
 //判断用于检查
 float judgement = 1 ;
 float encoder_difference;
 float shortest_encoder;
 float chassis_power_limit;
 
 //计算转向时用到的角度
 static fp32 deta[4]={45.0f,45.0f,45.0f,45.0f};
 //轮子向前的零点
 fp32 X_AXIS_ECD[4]= {7560,4097,5435,6181};  //7560,4009,2766,6181
 //静止时候保护变速器的零点
 static fp32 still[4] = {7560,4097,5435,6181};
 //求出的角度
 fp32 set_angle[4] = {0,0,0,0};
 //角度转化成的编码器值
 fp32 set_mangle[4] = {0,0,0,0};
 fp32 last_set_mangle[4] = {0,0,0,0};
 //上一次的角度
 fp32 last_angle[4] = {0,0,0,0};
 //标志位
 int fllg[4] = {0,0,0,0};
 static int flag_course=1;
 //延时时候所用的计数
 int time = 0;
 //电机转向设置
 float dirt[4]={-1.0f,1.0f,1.0f,-1.0f};
 
 struct chassis_status chassis;
 struct cap cap_chassis; // 电容组
 
 float wheel_rpm[4]; // 底盘速度数组
 /*软件功率限制*/
 float now_p = 0.0f;
// float b = 0.015f;
 float b = 0.001f;
 float Plimit = 1.0f; // 等比系数
 uint16_t Engerny_buffer;
 /*计算底盘速度*/
 float chassis_angle;
 int32_t max_curr = 0;
 float curr_a = 0.0f;
 /*变速小陀螺相关变量*/
 int refresh_interval = 30; // 更新次数，运行十次更新一次
 int smaller_than_2_count = 0;
 float valve = 0.0f;
 /*底盘模式*/
 extern float spin;
 float spin_forward_sen = -4.0f;
 /*停在坡上*/
 extern float slope;
 
 // 行进电机速度环PID
 pid_t motor_speed_3508[4];
 // 转向电机双环PID
 pid_t motor_location_6020[4];
 pid_t motor_speed_6020[4];
 //--目标速度
 float target_velocity[4]={0,0,0,0};
 // 底盘跟随PID
 pid_t chassis_follow;
 //斜坡规划器规划速度
 float Planning_velocity[4]={0,0,0,0};
 
 //斜坡规划器
 Slope chassis_slope[4];
 
 float tmp_delta_angle[4];
 float test_angle[4];
 float flag_lock;
 
 // 初始化底盘
 void chassis_move_init()
 {
	 chassis.speed.max_x = 8.0f; // m/s
	 chassis.speed.max_y = 8.0f; // m/s
	 chassis.speed.max_r = 5.0f; //
 
	 chassis.acc.max_x = 2.5f; // 1m/^2
	 chassis.acc.max_y = 2.5f; // m/^2
	 chassis.acc.max_r = 2.5f; //
 
	 /***********************************行进电机单速度环控制*******************************************/
	 pid_set(&motor_speed_3508[FR], 8000.0f, 0, 250, MAX_CURRENT, 0);// 8000.0f, 0, 250, MAX_CURRENT, 0
	 pid_set(&motor_speed_3508[FL], 8000.0f, 0, 250, MAX_CURRENT, 0);
	 pid_set(&motor_speed_3508[BL], 8000.0f, 0, 250, MAX_CURRENT, 0);
	 pid_set(&motor_speed_3508[BR], 8000.0f, 0, 250, MAX_CURRENT, 0);
	 /***********************************转向电机双环控制************************************************/
	 pid_set(&motor_location_6020[FR], 25.0f, 0.0f, 0.0f  ,9000.0f, 0.0f);
	 pid_set(&motor_location_6020[FL], 25.0f, 0.0f, 0.0f  ,9000.0f, 0.0f);
	 pid_set(&motor_location_6020[BL], 25.0f, 0.0f, 0.0f  ,9000.0f, 0.0f);
	 pid_set(&motor_location_6020[BR], 25.0f, 0.0f, 0.0f  ,9000.0f, 0.0f);
 
	 pid_set(&motor_speed_6020[FR], 55.0f, 0.0f, 0.0f, 10000.0f, 0.0f);//55.0f, 0.0f, 0.0f, 10000.0f, 0.0f
	 pid_set(&motor_speed_6020[FL], 55.0f, 0.0f, 0.0f, 10000.0f, 0.0f);
	 pid_set(&motor_speed_6020[BL], 55.0f, 0.0f, 0.0f, 10000.0f, 0.0f);
	 pid_set(&motor_speed_6020[BR], 55.0f, 0.0f, 0.0f, 10000.0f, 0.0f);
	 
	 Slope_set(&chassis_slope[0],2.1f,3.2f,1);
	 Slope_set(&chassis_slope[1],2.1f,3.2f,1);
	 Slope_set(&chassis_slope[2],2.1f,3.2f,1);
	 Slope_set(&chassis_slope[3],2.1f,3.2f,1);
 
	 chassis.turn_FR.now = 0;
	 chassis.turn_FR.set = TURN_FR_ANGLE;
	 chassis.turn_FR.offset = 0;
	 chassis.turn_FR.stable = 0;
	 chassis.turn_FR.set_turn_FR_speed = 0;
	 
	 chassis.turn_FL.now = 0;
	 chassis.turn_FL.set = TURN_FL_ANGLE;
	 chassis.turn_FL.offset = 0;
	 chassis.turn_FL.stable = 0;
	 chassis.turn_FL.set_turn_FL_speed = 0;
	 
	 chassis.turn_BL.now = 0;
	 chassis.turn_BL.set = TURN_BL_ANGLE;
	 chassis.turn_BL.offset = 0;
	 chassis.turn_BL.stable = 0;
	 chassis.turn_BL.set_turn_BL_speed = 0;
	 
	 chassis.turn_BR.now = 0;
	 chassis.turn_BR.set = TURN_BR_ANGLE;
	 chassis.turn_BR.offset = 0;
	 chassis.turn_BR.stable = 0;
	 chassis.turn_BR.set_turn_BR_speed = 0;
	 
	 //旋转速度分解的角度deta
	   for(int i=0;i<4;i++)
     {
       deta[i]=deta[i]*PI/180.0f;//角度转弧度
     }
			  
	 // srand(2); // 初始化一个随机数种子，为了之后变速小陀螺使用
 }
 //航向角度解算
 void Chassic_course_solving(float x,float y,float w)
 {
	 static fp32 set_angle_last[4]={TURN_FR_ANGLE,TURN_FL_ANGLE,TURN_BL_ANGLE,TURN_BR_ANGLE};
	 //线速度
	  w=w*Rotation_radius;
	  int16_t angle_temp[4];
 
	 //旋转运动
	 if(x==0&&y==0&&w==0)
	 {
				 judgement = 1 ;

         set_angle[0] = 0.0f;
				 set_angle[1] = 0.0f;
				 set_angle[2] = 0.0f;
				 set_angle[3] = 0.0f;
 
	 }
//	 set_angle[0] = 45.0f;
//				 set_angle[1] = -45.0f;
//				 set_angle[2] = 45.0f;
//				 set_angle[3] = -45.0f;
	 else
	 {
	 /********************************use*********************************************************/
		 set_angle[0] =	 atan2((y+w*0.707107f),(x+w*0.707107f))*180.0f/PI;
     set_angle[1] =	 atan2((y+w*0.707107f),(x-w*0.707107f))*180.0f/PI;
     set_angle[2] =	 atan2((y-w*0.707107f),(x-w*0.707107f))*180.0f/PI;
     set_angle[3] =   atan2((y-w*0.707107f),(x+w*0.707107f))*180.0f/PI;
	 
	 }
		 /**********************use*********************/
   
		 set_mangle[0] =	(X_AXIS_ECD[0]+set_angle[0]*8192.0/360.0)/8192*360;
		 set_mangle[1] =	(X_AXIS_ECD[1]+set_angle[1]*8192.0/360.0)/8192*360;
		 set_mangle[2] =	(X_AXIS_ECD[2]+set_angle[2]*8192.0/360.0)/8192*360;
		 set_mangle[3] =	(X_AXIS_ECD[3]+set_angle[3]*8192.0/360.0)/8192*360;
   
     last_angle[0] = set_angle[0];
		 last_angle[1] = set_angle[1];
		 last_angle[2] = set_angle[2];
		 last_angle[3] = set_angle[3];
		 /*******************************************/	
 
   //角度赋值
   //--在这里设置的需要的角度对应在编码器上的位置
	 chassis.turn_FR.set = (int32_t)set_mangle[0];
	 chassis.turn_FL.set = (int32_t)set_mangle[1];
	 chassis.turn_BL.set = (int32_t)set_mangle[2];
	 chassis.turn_BR.set = (int32_t)set_mangle[3];
 
	 set_angle_last[0]=set_mangle[0];
	 set_angle_last[1]=set_mangle[1];
	 set_angle_last[2]=set_mangle[2];
	 set_angle_last[3]=set_mangle[3];
 }	

 //航向电机数据更新
 void Chassis_course_updata()
 {
 //--用原始数据计算出一些数据
	 decode_as_6020(chassis_turn_FR);
	 decode_as_6020(chassis_turn_FL);
	 decode_as_6020(chassis_turn_BL);
	 decode_as_6020(chassis_turn_BR);
 //--编码器赋值
   chassis.turn_FR.now = get_motor_data(chassis_turn_FR).angle;
   chassis.turn_FL.now = get_motor_data(chassis_turn_FL).angle;
   chassis.turn_BL.now = get_motor_data(chassis_turn_BL).angle;
   chassis.turn_BR.now = get_motor_data(chassis_turn_BR).angle;
 
 }
 //航向PID计算
 void Chassis_course_pid_cal()
 {
   //就近转位
   steer_transfer_nearby();
 
   
   /******************************************************************/
   //back_up
   //FR
	 chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set);
   //FL
	 chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set);
	 //BL
		 chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set);
	 //BR
	 chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set);
   /******************************************************************/
 //--更新下电机数据
	 set_motor(pid_cal(&motor_speed_6020[FR], get_motor_data(chassis_turn_FR).speed_rpm, chassis.turn_FR.turn_FR_speed), chassis_turn_FR);
	 set_motor(pid_cal(&motor_speed_6020[FL], get_motor_data(chassis_turn_FL).speed_rpm, chassis.turn_FL.turn_FL_speed), chassis_turn_FL);
	 set_motor(pid_cal(&motor_speed_6020[BL], get_motor_data(chassis_turn_BL).speed_rpm, chassis.turn_BL.turn_BL_speed), chassis_turn_BL);
	 set_motor(pid_cal(&motor_speed_6020[BR], get_motor_data(chassis_turn_BR).speed_rpm, chassis.turn_BR.turn_BR_speed), chassis_turn_BR);
// 	set_motor(0, chassis_turn_FR);
// 	set_motor(0, chassis_turn_FL);
// 	set_motor(0, chassis_turn_BL);
// 	set_motor(0, chassis_turn_BR);
 }
 //计算行进电机速度及电流
 void Chassis_velocity_calc(float vx, float vy, float vw)
 {
   //
   //计算和速度
	 vw=vw*Rotation_radius;
	 float  V[4]={0,0,0,0};
	 //分别计算四个轮子的速度大小
	 V[3]=sqrt((vx+vw*sinf(deta[3]))*(vx+vw*sinf(deta[3]))+(vy-vw*cosf(deta[3]))*(vy-vw*cosf(deta[3])));
	 V[2]=sqrt((vx-vw*sinf(deta[2]))*(vx-vw*sinf(deta[2]))+(vy-vw*cosf(deta[2]))*(vy-vw*cosf(deta[2])));
	 V[1]=sqrt((-vx-vw*sinf(deta[1]))*(-vx-vw*sinf(deta[1]))+(-vy+vw*cosf(deta[1]))*(-vy+vw*cosf(deta[1])));
	 V[0]=sqrt((vx+vw*sinf(deta[0]))*(vx+vw*sinf(deta[0]))+(vy+vw*cosf(deta[0]))*(vy+vw*cosf(deta[0])));
 
	 //最大速度限制
	 val_limit(&vx, chassis.speed.max_x);
	 val_limit(&vy, chassis.speed.max_y);
	 val_limit(&vw, chassis.speed.max_r);
 //	// 计算加速度
 
 //	}
	//--电机数据更新
	 decode_as_3508(chassis_move_FR);
	 decode_as_3508(chassis_move_FL);
	 decode_as_3508(chassis_move_BL);
	 decode_as_3508(chassis_move_BR);
	//--确定目标速度的正负
	target_velocity[FR] =     V[0]*dirt[0];
	target_velocity[FL] =     V[1]*dirt[2];
	target_velocity[BL] =     V[2]*dirt[1];
	target_velocity[BR] =     V[3]*dirt[3];
	
 //直接规划
	Planning_velocity[FR] = Slope_Cal(&chassis_slope[0],get_motor_data(chassis_move_FR).round_speed * WHEEL_RADIUS * PI,target_velocity[FR]);
	Planning_velocity[FL] = Slope_Cal(&chassis_slope[1],get_motor_data(chassis_move_FL).round_speed * WHEEL_RADIUS * PI,target_velocity[FL]);
	Planning_velocity[BL] = Slope_Cal(&chassis_slope[2],get_motor_data(chassis_move_BL).round_speed * WHEEL_RADIUS * PI,target_velocity[BL]);
	Planning_velocity[BR] = Slope_Cal(&chassis_slope[3],get_motor_data(chassis_move_BR).round_speed * WHEEL_RADIUS * PI,target_velocity[BR]);

 
	 //计算马达电流
	 chassis.wheel_current[FR] = pid_cal(&motor_speed_3508[FR], get_motor_data(chassis_move_FR).round_speed * WHEEL_RADIUS * PI, Planning_velocity[FR]);//Planning_velocity[FR]
	 chassis.wheel_current[FL] = pid_cal(&motor_speed_3508[FL], get_motor_data(chassis_move_FL).round_speed * WHEEL_RADIUS * PI, Planning_velocity[FL]);//Planning_velocity[FL]
	 chassis.wheel_current[BL] = pid_cal(&motor_speed_3508[BL], get_motor_data(chassis_move_BL).round_speed * WHEEL_RADIUS * PI, Planning_velocity[BL]);//Planning_velocity[BL]
	 chassis.wheel_current[BR] = pid_cal(&motor_speed_3508[BR], get_motor_data(chassis_move_BR).round_speed * WHEEL_RADIUS * PI, Planning_velocity[BR]);//Planning_velocity[BR]
 
	/* 有功率控制	3508 */ 
    Plimit = power_limit(chassis.wheel_current);
   /* 无功率控制	3508 */
//   Plimit = 1.0f;

   //发送马达电流 （应在freeRTOS发送）
  	 set_motor((Plimit*chassis.wheel_current[FR]), chassis_move_FR);
  	 set_motor((Plimit*chassis.wheel_current[FL]), chassis_move_FL);
     set_motor((Plimit*chassis.wheel_current[BL]), chassis_move_BL);
     set_motor((Plimit*chassis.wheel_current[BR]), chassis_move_BR);
   
	//  set_motor(chassis.wheel_current[FR], chassis_move_FR);
	//  set_motor(chassis.wheel_current[FL], chassis_move_FL);
	//  set_motor(chassis.wheel_current[BL], chassis_move_BL);
	//  set_motor(chassis.wheel_current[BR], chassis_move_BR);
// 		set_motor(0, chassis_move_FR);
// 		set_motor(0, chassis_move_FL);
// 		set_motor(0, chassis_move_BL);
// 		set_motor(0, chassis_move_BR);
 }
 /*******************************就近转位***********************************/
 void steer_transfer_nearby()
 {

		 tmp_delta_angle[0] = obtain_modulus_normalization(chassis.turn_FR.set - chassis.turn_FR.now, 360.0f);
		 tmp_delta_angle[1] = obtain_modulus_normalization(chassis.turn_FL.set - chassis.turn_FL.now, 360.0f);
		 tmp_delta_angle[2] = obtain_modulus_normalization(chassis.turn_BL.set - chassis.turn_BL.now, 360.0f);
		 tmp_delta_angle[3] = obtain_modulus_normalization(chassis.turn_BR.set - chassis.turn_BR.now, 360.0f);
 
		 // 根据转动角度范围决定是否需要就近转位 FR
		 if (-90.0f <= tmp_delta_angle[0] && tmp_delta_angle[0] <= 90.0f)
		 {
			 // ±PI / 2之间无需反向就近转位
			 chassis.turn_FR.set = tmp_delta_angle[0] + chassis.turn_FR.now;
					   dirt[0] = -1.0f;
		 }
		 else
		 {
			 // 需要反转扣圈情况
			 chassis.turn_FR.set = obtain_modulus_normalization(tmp_delta_angle[0] + 180.0f, 360.0f) + chassis.turn_FR.now;
			 dirt[0] = 1.0f;
		 }
		 // 根据转动角度范围决定是否需要就近转位 FL
		 if (-90.0f <= tmp_delta_angle[1] && tmp_delta_angle[1] <= 90.0f)
		 {
			 // ±PI / 2之间无需反向就近转位
			 chassis.turn_FL.set = tmp_delta_angle[1] + chassis.turn_FL.now;
					   dirt[1] = 1.0f;
		 }
		 else
		 {
			 // 需要反转扣圈情况
			 chassis.turn_FL.set = obtain_modulus_normalization(tmp_delta_angle[1] + 180.0f, 360.0f) + chassis.turn_FL.now;
			 dirt[1] = -1.0f;
		 }
		 // 根据转动角度范围决定是否需要就近转位 BL
		 if (-90.0f <= tmp_delta_angle[2] && tmp_delta_angle[2] <= 90.0f)
		 {
			 // ±PI / 2之间无需反向就近转位
			 chassis.turn_BL.set = tmp_delta_angle[2] + chassis.turn_BL.now;
			 dirt[2] = 1.0f;
		 }
		 else
		 {
			 // 需要反转扣圈情况
			 chassis.turn_BL.set = obtain_modulus_normalization(tmp_delta_angle[2] + 180.0f, 360.0f) + chassis.turn_BL.now;
			 dirt[2] = -1.0f;
		 }
		 // 根据转动角度范围决定是否需要就近转位 BR
		 if (-90.0f <= tmp_delta_angle[3] && tmp_delta_angle[3] <= 90.0f)
		 {
			 // ±PI / 2之间无需反向就近转位
			 chassis.turn_BR.set = tmp_delta_angle[3] + chassis.turn_BR.now;
						 dirt[3] = -1.0f;
 
		 }
		 else
		 {
			 // 需要反转扣圈情况
			 chassis.turn_BR.set = obtain_modulus_normalization(tmp_delta_angle[3] + 180.0f, 360.0f) + chassis.turn_BR.now;
			 dirt[3] = 1.0f;
		 }
	 
 }
 /*******************************就近转位***********************************/
 
 // 软件功率限制
float power_limit(int16_t current[4])
{
		
	
	float max_p; // = REFEREE_DATA.Chassis_Power_Limit - 2.0f; // 2w余量

	if (cap.remain_vol <= 5)
		max_p = chassis_power_limit - 2.0f; // 2w余量
	else if (cap.remain_vol > 5)
	{
	  max_p = chassis_power_limit +cap.remain_vol * 12.5 ;// 超电最大功率 = 超电电压 * 14A 线圈最大电流 
	}
	
	now_p = 0;	
	
	const float a = 1.23e-07;	 // k1
  const float k2 = 6.65300036e-07; // k2
	const float constant = 4.081f;
	const float toque_coefficient = (20.0f/16384.0f)*(0.3f)*(187.0f/3591.0f)/9.55f;
	for (int i = 0; i < 4; i++)
	{
		// 估算功率
		// 西交利物浦：https://github.com/MaxwellDemonLin/Motor-modeling-and-power-control/blob/master/chassis_power_control.c#L89
		now_p += fabs(current[i] * toque_coefficient * get_motor_data(i).speed_rpm +
					  k2 * get_motor_data(i).speed_rpm * get_motor_data(i).speed_rpm +
					  a * current[i] * current[i] + constant) / 0.85f;
	}

	float percentage = max_p / now_p;
//  if(percentage<0)
//  {
//    percentage = -percentage;
//  }
	if(cap.remain_vol == 0)
	return 0.8f;
	if (percentage > 1.0f)
		return 1.0f;
	return percentage - 0.05;
}
 
	 void chassis_receive_from_gimbal_1(uint8_t data[8])
 {
 
		 float speed_x = bytes_to_float(&data[0]);  
	 float speed_y = bytes_to_float(&data[4]);  
		 chassis.speed_RC.x = -speed_x;
		 chassis.speed_RC.y = speed_y;
 }
 void chassis_receive_from_gimbal_2(uint8_t data[8])
 {
 
		 float speed_w = bytes_to_float(&data[0]);  
		 float speed_yaw = bytes_to_float(&data[4]);  
		 chassis.speed_RC.r = speed_w;
//		 chassis.speed_RC.big_yaw = speed_yaw*PI/180.f;		//单位 rad
   chassis.speed_RC.big_yaw = speed_yaw;		//单位 rad
 }
 void receive_REFEREE_DATA(uint8_t data[8])
 {
   chassis_power_limit = bytes_to_float(&data[0]); 
 }
 void chassis_receive_from_gimbal_3(uint8_t data[8])
 {
		 flag_lock = bytes_to_float(&data[0]);  
 }
 
 // 限制值
 inline void val_limit(float *val, float MAX)
 {
	 if (fabs(*val) > MAX)
	 {
		 if (*val > 0)
			 *val = MAX;
		 else
			 *val = -MAX;
	 }
 }
 // 限制变化量
 inline void change_limit(float last, float *now, float limit)
 {
	 float change = *now - last;
	 if (fabs(change) > limit)
	 {
		 if (change > 0)
			 *now = last + limit;
		 else
			 *now = last - limit;
	 }
 }
 
 //妙妙小工具
 //求取模归化  转动角度控制在-PI----PI
 float obtain_modulus_normalization(float x,float modulus)
 {
   float tmp;
   tmp = fmod(x + modulus / 2.0f, modulus);
   if (tmp < 0.0f)
   {
	   tmp += modulus;
   }
   return (tmp - modulus / 2.0f);
 }

 //float Angle_Limit(float angle, float max)
 //{
 //    angle = fmodf(angle, max);
 //    // 处理负角度
 //    if(angle < 0)
 //        angle += max;
 //    
 //    return angle;
 //}
 // 将字节数组转换为float
 float bytes_to_float(uint8_t *bytes) 
	 {
	 uint32_t temp = 0;
	 temp |= (bytes[0] << 0);  // 最低字节
	 temp |= (bytes[1] << 8);  // 第二个字节
	 temp |= (bytes[2] << 16); // 第三个字节
	 temp |= (bytes[3] << 24); // 最高字节
	 return *(float *)&temp;   // 将uint32_t指针强制转换为float指针
	 }
	 	 
 /*********************以下函数没有被用到，保留以防以后用到***********************/
 // 变速小陀螺 待完善
 float random_anti_vision_r_s(float min, float max)
 {
	 static int run_count = 100;
	 run_count++;
	 if (run_count > refresh_interval && smaller_than_2_count < 2)
	 {
		 valve = generate_random_float(min, max); // 下限和上限
 
		 run_count = 0;
		 if (valve < 2.0f)
		 {
			 smaller_than_2_count++;
		 }
		 return valve;
	 }
	 else if (run_count > refresh_interval && smaller_than_2_count >= 2)
	 {
		 smaller_than_2_count = 0;
		 return 3.5f;
	 }
	 else
		 return valve;
 }
 
 float generate_random_float(float min, float max)
 {
	 return min + ((float)rand() / RAND_MAX) * (max - min);
 }
 
 // 以指数的形式去试试上面那个缓启动，输出acc=ae^bv-a+c,减a为了让c为初始加速度,传入速度为自变量
 float fly_speed_up_ex(float a, float b, float c, float v)
 {
	 return a * exp(b * v) - a + c;
 }
 // 刚开始时默认加速度，然后加速越来越快
 // 传入值，每次运行加速度增加值，速度上限值
 float fly_speed_up(float *set, float acceleration_increase_rate, float limit)
 {
	 static float initial_acceleration = 0.2; // 初始加速度设为0.2//每次运行+加速度;
	 if (set == 0)							 // 也就是从头开始了，那就重置加速度
	 {
		 initial_acceleration = 0.2;
	 }
	 if (fabs(*set) < limit)
	 {
		 *set += initial_acceleration;
		 initial_acceleration += acceleration_increase_rate;
	 }
	 else
	 {
		 *set = limit;
	 }
 }
 
 int RampInc_float(int16_t *buffer, float now, float ramp)
 {
	 if (*buffer > 0)
	 {
		 if (*buffer > ramp)
		 {
			 now += ramp;
			 *buffer -= ramp;
		 }
		 else
		 {
			 now += *buffer;
			 *buffer = 0;
		 }
	 }
	 else
	 {
		 if (*buffer < -ramp)
		 {
			 now += -ramp;
			 *buffer -= -ramp;
		 }
		 else
		 {
			 now += *buffer;
			 *buffer = 0;
		 }
	 }
 
	 return now;
 }
  