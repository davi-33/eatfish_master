
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
#include "referee_handle_pack.h"
#include "Global_status.h"

#include "Stm32_time.h"

#include "math.h"
#include "RampFunc.h" // ����б�º�����

// wheel conf
#define WHEEL_RADIUS 0.26282f // m
#define Rotation_radius  0.37169f  //m
#define PI 3.1415926f
// car conf
#define ROLLER_DISTANCE 100 // mm  ���
#define WHEELS_DISTANCE 100 // mm  �־�
// mm/s
#define FR 0
#define FL 1
#define BL 2
#define BR 3
//��㶨��
#define TURN_FR_ANGLE 7560   //7560
#define TURN_FL_ANGLE 4009   //
#define TURN_BL_ANGLE 2766   //122.18
#define TURN_BR_ANGLE 6181		//6181
//�ж����ڼ��
float judgement = 1 ;
float encoder_difference;
float shortest_encoder;

//����ת��ʱ�õ��ĽǶ�
static fp32 deta[4]={45.0f,45.0f,45.0f,45.0f};
//������ǰ�����
fp32 X_AXIS_ECD[4]= {7560,4009,2766,6181};  //7560
//��ֹʱ�򱣻������������
static fp32 still[4] = {7560,4009,2766,6181};
//����ĽǶ�
fp32 set_angle[4] = {0,0,0,0};
//�Ƕ�ת���ɵı�����ֵ
fp32 set_mangle[4] = {0,0,0,0};
fp32 last_set_mangle[4] = {0,0,0,0};
//��һ�εĽǶ�
fp32 last_angle[4] = {0,0,0,0};
//��־λ
static int flag_course=1;
//��ʱʱ�����õļ���
int time = 0;
//���ת������
static int dirt[4]={-1,1,1,-1};
//static int dirt[4]={1,1,1,1};

struct chassis_status chassis;
struct cap cap_chassis; // ������

float wheel_rpm[4]; // �����ٶ�����
/*������������*/
float now_p = 0.0f;
float b = 0.015f;
float Plimit = 1.0f; // �ȱ�ϵ��
uint16_t Engerny_buffer;
/*��������ٶ�*/
float chassis_angle;
int32_t max_curr = 0;
float curr_a = 0.0f;
/*����С������ر���*/
int refresh_interval = 30; // ���´���������ʮ�θ���һ��
int smaller_than_2_count = 0;
float valve = 0.0f;
/*����ģʽ*/
float relative_angle = 0;
extern float spin;
float spin_forward_sen = -4.0f;
/*ͣ������*/
extern float slope;
/*���岻���ı���*/
float Power;
float vis_rela_angle;

// �н�����ٶȻ�PID
pid_t motor_speed_3508[4];
// ת����˫��PID
pid_t motor_location_6020[4];
pid_t motor_speed_6020[4];
//--Ŀ���ٶ�
float target_velocity[4]={0,0,0,0};
// ���̸���PID
pid_t chassis_follow;

float tmp_delta_angle[4];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// ��ʼ������
void chassis_move_init()
{
	chassis.speed.max_x = 8.0f; // m/s
	chassis.speed.max_y = 8.0f; // m/s
	chassis.speed.max_r = 5.0f; //

	chassis.acc.max_x = 2.5f; // 1m/^2
	chassis.acc.max_y = 2.5f; // m/^2
	chassis.acc.max_r = 2.5f; //

	/***********************************�н�������ٶȻ�����*******************************************/
	pid_set(&motor_speed_3508[FR], 1000, 0, 0.01, MAX_CURRENT, 0);
	pid_set(&motor_speed_3508[FL], 1000, 0, 0.01, MAX_CURRENT, 0);
	pid_set(&motor_speed_3508[BL], 1000, 0, 0.01, MAX_CURRENT, 0);
	pid_set(&motor_speed_3508[BR], 1000, 0, 0.01, MAX_CURRENT, 0);
	/***********************************ת����˫������************************************************/
	pid_set(&motor_location_6020[FR], 20.0, 0, 0.0  ,9000, 0);
	pid_set(&motor_location_6020[FL], 20.0, 0, 0.0  ,9000, 0);
	pid_set(&motor_location_6020[BL], 20.0, 0, 0.0  ,9000, 0);
	pid_set(&motor_location_6020[BR], 20.0, 0, 0.0  ,9000, 0);

	pid_set(&motor_speed_6020[FR], 40, 0, 0, 10000, 0);
	pid_set(&motor_speed_6020[FL], 40, 0, 0, 10000, 0);
	pid_set(&motor_speed_6020[BL], 40, 0, 0, 10000, 0);
	pid_set(&motor_speed_6020[BR], 40, 0, 0, 10000, 0);

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
	
	//��ת�ٶȷֽ�ĽǶ�deta
	  for(int i=0;i<4;i++)
	     deta[i]=deta[i]*PI/180.0f;//�Ƕ�ת����
			 
	// srand(2); // ��ʼ��һ����������ӣ�Ϊ��֮�����С����ʹ��
}
//����ǶȽ���
void Chassic_course_solving(float x,float y,float w)
{
	static fp32 set_angle_last[4]={TURN_FR_ANGLE,TURN_FL_ANGLE,TURN_BL_ANGLE,TURN_BR_ANGLE};
	//���ٶ�
	 w=w*Rotation_radius;
	 int16_t angle_temp[4];

	//��ת�˶�
	if(x==0&&y==0&&w==0)
	{
		time++;								
//		if(x==0&&y==0&&w==0)
//		{
//			if(time>50)
//			{
				judgement = 1 ;
////				set_mangle[0]= still[0];
////				set_mangle[1]= still[1];
////				set_mangle[2]= still[2];
////				set_mangle[3]= still[3];
        /******************use******************/
				set_angle[0] = 0.0;
				set_angle[1] = 0.0;
				set_angle[2] = 0.0;
				set_angle[3] = 0.0;
        
//        //RC������,���������·���ж�,���λ��
//        set_mangle[0] = X_AXIS_ECD[0]/8192.0*360.0;
//        set_mangle[1] = X_AXIS_ECD[1]/8192.0*360.0;
//        set_mangle[2] = X_AXIS_ECD[2]/8192.0*360.0;
//        set_mangle[3] = X_AXIS_ECD[3]/8192.0*360.0;
        /************************************/
		
////////				set_mangle[0]= last_set_mangle[0];
////////				set_mangle[1]= last_set_mangle[1];
////////				set_mangle[2]= last_set_mangle[2];
////////				set_mangle[3]= last_set_mangle[3];

//////			  last_angle[0] = 45.0f;
//////				last_angle[1] = -45.0f;
//////				last_angle[2] = 45.0f;
//////				last_angle[3] = -45.0f;
//			}
//			else
//			{
//				set_mangle[0]=X_AXIS_ECD[0]+last_angle[0]*8192/360.0;
//				set_mangle[1]=X_AXIS_ECD[1]+last_angle[1]*8192/360.0;
//				set_mangle[2]=X_AXIS_ECD[2]+last_angle[2]*8192/360.0;
//				set_mangle[3]=X_AXIS_ECD[3]+last_angle[3]*8192/360.0;
        /*************************use*******************************/
				set_angle[0] = last_angle[0];
				set_angle[1] = last_angle[1];
				set_angle[2] = last_angle[2];
				set_angle[3] = last_angle[3];
        
//        //��ʱ500ǰ
//        set_mangle[0] = last_angle[0];
//        set_mangle[1] = last_angle[1];
//        set_mangle[2] = last_angle[2];
//        set_mangle[3] = last_angle[3];
        /********************************************************/
//			}
//		}

	}
	else
	{
		judgement = 2 ;
    /********************************use*********************************************************/
		set_angle[3] =   atan2((y-w*0.707107f),(x+w*0.707107f))*180.0f/PI;
		set_angle[2] =	 atan2((y-w*0.707107f),(x-w*0.707107f))*180.0f/PI;
		set_angle[1] =	 atan2((y+w*0.707107f),(x-w*0.707107f))*180.0f/PI;
		set_angle[0] =	 atan2((y+w*0.707107f),(x+w*0.707107f))*180.0f/PI;
    
//    //���·��
//    set_mangle[0] =	(X_AXIS_ECD[0]+set_angle[0]*8192/360.0)/8192*360;
//		set_mangle[1] =	(X_AXIS_ECD[1]+set_angle[1]*8192/360.0)/8192*360;
//		set_mangle[2] =	(X_AXIS_ECD[2]+set_angle[2]*8192/360.0)/8192*360;
//		set_mangle[3] =	(X_AXIS_ECD[3]+set_angle[3]*8192/360.0)/8192*360;
//    
//    //�ڴ˴��������·���ж�
//    //FR
//    if(set_mangle[0]-get_motor_data(chassis_turn_FR).ecd/8192.0*360.0>90.0)
//    {
//      dirt[0] = 1;
//      set_mangle[0] -= 180.0;
//    }
//    else if(set_mangle[0]-get_motor_data(chassis_turn_FR).ecd/8192.0*360.0<-90.0)
//    {
//      dirt[0] = 1;
//      set_mangle[0] += 180.0;
//    }
//    else
//    {
//      dirt[0] = -1;
//      set_mangle[0] = set_mangle[0];
//    }
//    //FL
//    if(set_mangle[1]-get_motor_data(chassis_turn_FL).ecd/8192.0*360.0>90.0)
//    {
//      dirt[1] = 1;
//      set_mangle[1] -= 180.0;
//    }
//    else if(set_mangle[1]-get_motor_data(chassis_turn_FL).ecd/8192.0*360.0<-90.0)
//    {
//      dirt[1] = 1;
//      set_mangle[1] += 180.0;
//    }
//    else
//    {
//      dirt[1] = -1;
//      set_mangle[1] = set_mangle[1];
//    }
//    //BL
//    if(set_mangle[2]-get_motor_data(chassis_turn_BL).ecd/8192.0*360.0>90.0)
//    {
//      dirt[2] = 1;
//      set_mangle[2] -= 180.0;
//    }
//    else if(set_mangle[2]-get_motor_data(chassis_turn_BL).ecd/8192.0*360.0<-90.0)
//    {
//      dirt[2] = 1;
//      set_mangle[2] += 180.0;
//    }
//    else
//    {
//      dirt[2] = -1;
//      set_mangle[2] = set_mangle[2];
//    }
//    //BR
//    if(set_mangle[3]-get_motor_data(chassis_turn_BR).ecd/8192.0*360.0>90.0)
//    {
//      dirt[3] = 1;
//      set_mangle[3] -= 180.0;
//    }
//    else if(set_mangle[3]-get_motor_data(chassis_turn_BR).ecd/8192.0*360.0<-90.0)
//    {
//      dirt[3] = 1;
//      set_mangle[3] += 180.0;
//    }
//    else
//    {
//      dirt[3] = -1;
//      set_mangle[3] = set_mangle[3];
//    }
//    last_angle[0] = set_mangle[0];
//		last_angle[1] = set_mangle[1];
//		last_angle[2] = set_mangle[2];
//		last_angle[3] = set_mangle[3];
    /*******************************************************************************************/
		/*******************************************/
		//ӳ��->������
		//--�õ��������ϵ�λ��
////		set_mangle[0] =	X_AXIS_ECD[0]+set_angle[0]*8192/360.0;
////		set_mangle[1] =	X_AXIS_ECD[1]+set_angle[1]*8192/360.0;
////		set_mangle[2] =	X_AXIS_ECD[2]+set_angle[2]*8192/360.0;
////		set_mangle[3] =	X_AXIS_ECD[3]+set_angle[3]*8192/360.0;
		
	}
		/**********************use*********************/
  
		set_mangle[0] =	(X_AXIS_ECD[0]+set_angle[0]*8192/360.0)/8192*360;
		set_mangle[1] =	(X_AXIS_ECD[1]+set_angle[1]*8192/360.0)/8192*360;
		set_mangle[2] =	(X_AXIS_ECD[2]+set_angle[2]*8192/360.0)/8192*360;
		set_mangle[3] =	(X_AXIS_ECD[3]+set_angle[3]*8192/360.0)/8192*360;
  
    last_angle[0] = set_angle[0];
		last_angle[1] = set_angle[1];
		last_angle[2] = set_angle[2];
		last_angle[3] = set_angle[3];
		/*******************************************/	
	
////	//���Ž����
//		if( fabs( Find_min_Angle((fp32)get_motor_data(chassis_turn_FR).ecd/8192*360 , set_mangle[0] )) >= 90 )
//		{	
//			dirt[0] = 1;
//			set_mangle[0] = Angle_Limit( set_mangle[0] - 180, 360 );
//		}
//		else
//		dirt[0] = -1;
////		
////		if( fabs( Find_min_Angle((fp32)get_motor_data(chassis_turn_FL).ecd , set_mangle[1] )) >= 2048 )
////		{	
////			dirt[1] = 1;
////			set_mangle[1] = Angle_Limit( set_mangle[1] - 4096, 8192 );
////		}
////		else
////		dirt[1] = -1;
////		
////		if( fabs( Find_min_Angle((fp32)get_motor_data(chassis_turn_BL).ecd , set_mangle[2] )) >= 2048 )
////		{	
////			dirt[2] = 1;
////			set_mangle[2] = Angle_Limit( set_mangle[2] - 4096, 8192 );
////		}
////		else
////		dirt[2] = -1;
////		
////		if( fabs( Find_min_Angle((fp32)get_motor_data(chassis_turn_BR).ecd , set_mangle[3] )) >= 2048 )
////		{	
////			dirt[3] = -1;
////			set_mangle[3] = Angle_Limit( set_mangle[3] - 4096, 8192 );
////		}
////		else
////		dirt[3] = 1;
//		//���Ž���� test
//      if( fabs( Find_min_Angle((fp32)get_motor_data(chassis_turn_FR).ecd/8192*360 , set_mangle[0] )) >= 90 )
//		{	
//			dirt[0] = 1;
//			set_mangle[0] = Angle_Limit( set_mangle[0] - 180, 360 );
//		}
//		else
//		dirt[0] = -1;
//		if( fabs( (fp32)get_motor_data(chassis_turn_FR).last_ecd/8192*360 - set_mangle[0] ) > 90.0 )
//		{	
//			dirt[0] = 1;
//			set_mangle[0] = Angle_Limit( set_mangle[0] - 180.0, 360.0 );
//		}
//		else
//		dirt[0] = -1;
//		
//		if( fabs( (fp32)get_motor_data(chassis_turn_FL).ecd/8192*360 - set_mangle[1] ) > 90.0 )
//		{	
//			dirt[1] = 1;
//			set_mangle[1] = Angle_Limit( set_mangle[1] - 180.0, 360.0 );
//		}
//		else
//		dirt[1] = -1;
//		
//		if( fabs( (fp32)get_motor_data(chassis_turn_BL).ecd/8192*360 - set_mangle[2] ) > 90.0 )
//		{	
//			dirt[2] = 1;
//			set_mangle[2] = Angle_Limit( set_mangle[2] - 180.0, 360.0 );
//		}
//		else
//		dirt[2] = -1;
//		
//		if( fabs( (fp32)get_motor_data(chassis_turn_BR).ecd/8192*360 - set_mangle[3] ) > 90.0 )
//		{	
//			dirt[3] = -1;
//			set_mangle[3] = Angle_Limit( set_mangle[3] - 180.0, 360.0 );
//		}
//		else
//		dirt[3] = 1;
////////		//���Ž����
////////		//if( fabs( (fp32)get_motor_data(chassis_turn_FR).ecd - set_mangle[0] ) >= 2048 )
//////////		if(  set_mangle[0]-motor_data[15].ecd    >= 2048 )
//////////		{dirt[0]=1;
//////////		set_mangle[0]-=4096;
//////////		}
//////////			else if(  set_mangle[0]-motor_data[15].ecd <=- 2048 )
//////////		{	
//////////			dirt[0] = 1;
//////////					set_mangle[0]+=4096;

//////////		}
//////////		else
//////////		dirt[0] = -1;
//////////		if( fabs( get_motor_data(chassis_turn_FR).ecd - set_mangle[0] ) >= 2048 )
//////////		{	
//////////			dirt[1] = 1;
//////////			set_mangle[0] = Angle_Limit( set_mangle[0] - 4096, 8192 );
//////////		}
//////////		else
//////////		dirt[1] = -1;
//////////		
//////////		if( fabs( (fp32)get_motor_data(chassis_turn_FL).ecd - set_mangle[1] ) >= 2048 )
//////////		{	
//////////			dirt[1] = -1;
//////////			set_mangle[1] = Angle_Limit( set_mangle[1] - 4096, 8192 );
//////////		}
//////////		else
//////////		dirt[1] = 1;
//////////		
//////////		if( fabs( (fp32)get_motor_data(chassis_turn_BL).ecd - set_mangle[2] ) >= 2048 )
//////////		{	
//////////			dirt[2] = -1;
//////////			set_mangle[2] = Angle_Limit( set_mangle[2] - 4096, 8192 );
//////////		}
//////////		else
//////////		dirt[2] = 1;
//////////		
//////////		if( fabs( (fp32)get_motor_data(chassis_turn_BR).ecd - set_mangle[3] ) >= 2048 )
//////////		{	
//////////			dirt[3] = 1;
//////////			set_mangle[3] = Angle_Limit( set_mangle[3] - 4096, 8192 );
//////////		}
//////////		else
//////////		dirt[3] = -1;
//////////		
	
  //�Ƕȸ�ֵ
  //--���������õ���Ҫ�ĽǶȶ�Ӧ�ڱ������ϵ�λ��
	chassis.turn_FR.set = (int32_t)set_mangle[0];
	chassis.turn_FL.set = (int32_t)set_mangle[1];
	chassis.turn_BL.set = (int32_t)set_mangle[2];
	chassis.turn_BR.set = (int32_t)set_mangle[3];

	set_angle_last[0]=set_mangle[0];
	set_angle_last[1]=set_mangle[1];
	set_angle_last[2]=set_mangle[2];
	set_angle_last[3]=set_mangle[3];
}	
//���������ݸ���
void Chassis_course_updata()
{
//--��ԭʼ���ݼ����һЩ����
	decode_as_6020(chassis_turn_FR);
	decode_as_6020(chassis_turn_FL);
	decode_as_6020(chassis_turn_BL);
	decode_as_6020(chassis_turn_BR);
  chassis.turn_FR.now = get_motor_data(chassis_turn_FR).angle;
  chassis.turn_FL.now = get_motor_data(chassis_turn_FL).angle;
  chassis.turn_BL.now = get_motor_data(chassis_turn_BL).angle;
  chassis.turn_BR.now = get_motor_data(chassis_turn_BR).angle;
//	chassis.turn_FR.now = get_motor_data(chassis_turn_FR).ecd/8192.0*360.0;
//	chassis.turn_FL.now = get_motor_data(chassis_turn_FL).ecd/8192.0*360.0;
//	chassis.turn_BL.now = get_motor_data(chassis_turn_BL).ecd/8192.0*360.0;
//	chassis.turn_BR.now = get_motor_data(chassis_turn_BR).ecd/8192.0*360.0;
//--ת�ٸ�ֵ
	chassis.turn_FR.turn_FR_speed = get_motor_data(chassis_turn_FR).speed_rpm*6;
	chassis.turn_FL.turn_FL_speed = get_motor_data(chassis_turn_FL).speed_rpm*6;
	chassis.turn_BL.turn_BL_speed = get_motor_data(chassis_turn_BL).speed_rpm*6;
	chassis.turn_BR.turn_BR_speed = get_motor_data(chassis_turn_BR).speed_rpm*6;

}
//����PID����
void Chassis_course_pid_cal()
{
  //�ͽ�תλ
  steer_transfer_nearby();
  /********************************use*****************************/
//	//FR
//	if(chassis.turn_FR.set-chassis.turn_FR.now>=180.0)
//	{
//    chassis.turn_FR.set -= 360.0;
//    if(chassis.turn_FR.set-chassis.turn_FR.now>90.0)
//    {
//      dirt[0] = 1;
//      chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set-180.0);
//    }
//    else if(chassis.turn_FR.set-chassis.turn_FR.now<-90.0)
//    {
//      dirt[0] = 1;
//      chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set+180.0);
//    }
//    else
//    {
//      dirt[0] = -1;
//      chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set);
//    }
////	chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set-360.0);
//	}
//	else if(chassis.turn_FR.set-chassis.turn_FR.now<=-180.0)
//	{
//    chassis.turn_FR.set += 360.0;
//    if(chassis.turn_FR.set-chassis.turn_FR.now>90.0)
//    {
//      dirt[0] = 1;
//      chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set-180.0);
//    }
//    else if(chassis.turn_FR.set-chassis.turn_FR.now<-90.0)
//    {
//      dirt[0] = 1;
//      chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set+180.0);
//    }
//    else
//    {
//      dirt[0] = -1;
//      chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set);
//    }
////	chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set+360.0);
//	}
//	else
//	{
//    if(chassis.turn_FR.set-chassis.turn_FR.now>90.0)
//    {
//      dirt[0] = 1;
//      chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set-180.0);
//    }
//    else if(chassis.turn_FR.set-chassis.turn_FR.now<-90.0)
//    {
//      dirt[0] = 1;
//      chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set+180.0);
//    }
//    else
//    {
//      dirt[0] = -1;
//      chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set);
//    }
//	}
//  //FL
//  if(chassis.turn_FL.set-chassis.turn_FL.now>=180.0)
//	{
//    chassis.turn_FL.set -= 360.0;
//    if(chassis.turn_FL.set-chassis.turn_FL.now>90.0)
//    {
//      dirt[1] = 1;
//      chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set-180.0);
//    }
//    else if(chassis.turn_FL.set-chassis.turn_FL.now<-90.0)
//    {
//      dirt[1] = 1;
//      chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set+180.0);
//    }
//    else
//    {
//      dirt[1] = -1;
//      chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set);
//    }
//	}
//	else if(chassis.turn_FL.set-chassis.turn_FL.now<=-180.0)
//	{
//    chassis.turn_FL.set += 360.0;
//    if(chassis.turn_FL.set-chassis.turn_FL.now>90.0)
//    {
//      dirt[1] = 1;
//      chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set-180.0);
//    }
//    else if(chassis.turn_FL.set-chassis.turn_FL.now<-90.0)
//    {
//      dirt[1] = 1;
//      chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set+180.0);
//    }
//    else
//    {
//      dirt[1] = -1;
//      chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set);
//    }
//	}
//	else
//	{
//    if(chassis.turn_FL.set-chassis.turn_FL.now>90.0)
//    {
//      dirt[1] = 1;
//      chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set-180.0);
//    }
//    else if(chassis.turn_FL.set-chassis.turn_FL.now<-90.0)
//    {
//      dirt[1] = 1;
//      chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set+180.0);
//    }
//    else
//    {
//      dirt[1] = -1;
//      chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set);
//    }
//	}
//	//BL
//	if(chassis.turn_BL.set-chassis.turn_BL.now>=180.0)
//	{
//    chassis.turn_BL.set -= 360.0;
//    if(chassis.turn_BL.set-chassis.turn_BL.now>90.0)
//    {
//      dirt[2] = 1;
//      chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set-180.0);
//    }
//    else if(chassis.turn_BL.set-chassis.turn_BL.now<-90.0)
//    {
//      dirt[2] = 1;
//      chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set+180.0);
//    }
//    else
//    {
//      dirt[2] = -1;
//      chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set);
//    }
//	}
//	else if(chassis.turn_BL.set-chassis.turn_BL.now<=-180.0)
//	{
//    chassis.turn_BL.set += 360.0;
//    if(chassis.turn_BL.set-chassis.turn_BL.now>90.0)
//    {
//      dirt[2] = 1;
//      chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set-180.0);
//    }
//    else if(chassis.turn_BL.set-chassis.turn_BL.now<-90.0)
//    {
//      dirt[2] = 1;
//      chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set+180.0);
//    }
//    else
//    {
//      dirt[2] = -1;
//      chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set);
//    }
//	}
//	else
//	{
//    if(chassis.turn_BL.set-chassis.turn_BL.now>90.0)
//    {
//      dirt[2] = 1;
//      chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set-180.0);
//    }
//    else if(chassis.turn_BL.set-chassis.turn_BL.now<-90.0)
//    {
//      dirt[2] = 1;
//      chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set+180.0);
//    }
//    else
//    {
//      dirt[2] = -1;
//      chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set);
//    }
//	}
//	//BR
//	if(chassis.turn_BR.set-chassis.turn_BR.now>=180.0)
//	{
//    chassis.turn_BR.set -= 360.0;
//    if(chassis.turn_BR.set-chassis.turn_BR.now>90.0)
//    {
//      dirt[3] = -1;
//      chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set-180.0);
//    }
//    else if(chassis.turn_BR.set-chassis.turn_BR.now<-90.0)
//    {
//      dirt[3] = -1;
//      chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set+180.0);
//    }
//    else
//    {
//      dirt[3] = 1;
//      chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set);
//    }
//	}
//	else if(chassis.turn_BR.set-chassis.turn_BR.now<=-180.0)
//	{
//    chassis.turn_BR.set += 360.0;
//    if(chassis.turn_BR.set-chassis.turn_BR.now>90.0)
//    {
//      dirt[3] = -1;
//      chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set-180.0);
//    }
//    else if(chassis.turn_BR.set-chassis.turn_BR.now<-90.0)
//    {
//      dirt[3] = -1;
//      chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set+180.0);
//    }
//    else
//    {
//      dirt[3] = 1;
//      chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set);
//    }
//	}
//	else
//	{
//    if(chassis.turn_BR.set-chassis.turn_BR.now>90.0)
//    {
//      dirt[3] = -1;
//      chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set-180.0);
//    }
//    else if(chassis.turn_BR.set-chassis.turn_BR.now<-90.0)
//    {
//      dirt[3] = -1;
//      chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set+180.0);
//    }
//    else
//    {
//      dirt[3] = 1;
//      chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set);
//    }
//	}
  
  /******************************************************************/
  //back_up
//FR
	if(chassis.turn_FR.set-chassis.turn_FR.now>=180.0)
	{
	chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set-360.0);
	}
	else if(chassis.turn_FR.set-chassis.turn_FR.now<=-180.0)
	{
	chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set+360.0);
	}
	else
	{
	chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020[FR], chassis.turn_FR.now, chassis.turn_FR.set);
	}
  //FL
	if(chassis.turn_FL.set-chassis.turn_FL.now>=180.0)
	{
	chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set-360.0);
	}
	else if(chassis.turn_FL.set-chassis.turn_FL.now<=-180.0)
	{
	chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set+360.0);
	}
	else
	{
	chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020[FL], chassis.turn_FL.now, chassis.turn_FL.set);
	}
	//BL
	if(chassis.turn_BL.set-chassis.turn_BL.now>=180.0)
	{
	chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set-360.0);
	}
	else if(chassis.turn_BL.set-chassis.turn_BL.now<=-180.0)
	{
	chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set+360.0);
	}
	else
	{
		chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020[BL], chassis.turn_BL.now, chassis.turn_BL.set);
	}
	//BR
	if(chassis.turn_BR.set-chassis.turn_BR.now>=180.0)
	{
	chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set-360.0);
	}
	else if(chassis.turn_BR.set-chassis.turn_BR.now<=-180.0)
	{
	chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set+360.0);
	}
	else
	{
	chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020[BR], chassis.turn_BR.now, chassis.turn_BR.set);
	}
  /******************************************************************/
//--�����µ������
	set_motor(pid_cal(&motor_speed_6020[FR], get_motor_data(chassis_turn_FR).speed_rpm, chassis.turn_FR.turn_FR_speed), chassis_turn_FR);
	set_motor(pid_cal(&motor_speed_6020[FL], get_motor_data(chassis_turn_FL).speed_rpm, chassis.turn_FL.turn_FL_speed), chassis_turn_FL);
	set_motor(pid_cal(&motor_speed_6020[BL], get_motor_data(chassis_turn_BL).speed_rpm, chassis.turn_BL.turn_BL_speed), chassis_turn_BL);
	set_motor(pid_cal(&motor_speed_6020[BR], get_motor_data(chassis_turn_BR).speed_rpm, chassis.turn_BR.turn_BR_speed), chassis_turn_BR);
//	set_motor(0, chassis_turn_FR);
//	set_motor(0, chassis_turn_FL);
//	set_motor(0, chassis_turn_BL);
//	set_motor(0, chassis_turn_BR);
}
//�����н�����ٶȼ�����
void Chassis_velocity_calc(float vx, float vy, float vw)
{
  //
  //������ٶ�
	vw=vw*Rotation_radius;
	float  V[4]={0,0,0,0};
	//�ֱ�����ĸ����ӵ��ٶȴ�С
	V[3]=sqrt((vx+vw*sinf(deta[0]))*(vx+vw*sinf(deta[0]))+(vy-vw*cosf(deta[0]))*(vy-vw*cosf(deta[0])));
	V[2]=sqrt((vx-vw*sinf(deta[1]))*(vx-vw*sinf(deta[1]))+(vy-vw*cosf(deta[1]))*(vy-vw*cosf(deta[1])));
	V[1]=sqrt((vx-vw*sinf(deta[2]))*(vx-vw*sinf(deta[2]))+(vy+vw*cosf(deta[2]))*(vy+vw*cosf(deta[2])));
	V[0]=sqrt((vx+vw*sinf(deta[3]))*(vx+vw*sinf(deta[3]))+(vy+vw*cosf(deta[3]))*(vy+vw*cosf(deta[3])));

	if(vw != 0)
	{
	V[0] = V[0];
	V[1] = V[1];
	V[2] = V[2];
	V[3] = V[3];

	}
	

	//����ٶ�����
	val_limit(&vx, chassis.speed.max_x);
	val_limit(&vy, chassis.speed.max_y);
	val_limit(&vw, chassis.speed.max_r);
//	// ������ٶ�
//	uint32_t now_time = Get_sys_time_ms();
//	static uint32_t last_time = 0;
//	float dt = (now_time - last_time) / 1000.0f;
//	last_time = now_time;
//	chassis.acc.now_x = (chassis.speed_RC.x - chassis.speed.now_x) / dt;
//	chassis.acc.now_y = (chassis.speed.y - chassis.speed.now_y) / dt;
//	chassis.acc.now_r = (chassis.speed.r - chassis.speed.now_r) / dt;
//	// ���Ƽ��ٶ�
//	if (fabs(chassis.acc.now_x) > chassis.acc.max_x)
//	{
//		if (chassis.speed.x < 0.0f)
//			chassis.speed.x = -(chassis.acc.max_x * dt + chassis.speed.now_x);
//		chassis.speed.x = chassis.acc.max_x * dt + chassis.speed.now_x;
//	}
//	if (fabs(chassis.acc.now_y) > chassis.acc.max_y)
//	{
//		if (chassis.speed.y < 0.0f)
//			chassis.speed.y = -(chassis.acc.max_y * dt + chassis.speed.now_y);
//		chassis.speed.y = chassis.acc.max_y * dt + chassis.speed.now_y;
//	}
//	if (fabs(chassis.acc.now_r) > chassis.acc.max_r)
//	{
//		if (chassis.speed.r < 0.0f)
//			chassis.speed.r = -(chassis.acc.max_r * dt + chassis.speed.now_r);
//		chassis.speed.r = chassis.acc.max_r * dt + chassis.speed.now_r;
//	}
   //--������ݸ���
	decode_as_3508(chassis_move_FR);
	decode_as_3508(chassis_move_FL);
	decode_as_3508(chassis_move_BL);
	decode_as_3508(chassis_move_BR);
   //--ȷ��Ŀ���ٶȵ�����
   target_velocity[FR] =     V[0]*dirt[0];
   target_velocity[FL] =     V[1]*dirt[1];
   target_velocity[BL] =     V[2]*dirt[2];
	 target_velocity[BR] =     V[3]*dirt[3];


	//�����������
	chassis.wheel_current[FR] = pid_cal(&motor_speed_3508[FR], get_motor_data(chassis_move_FR).round_speed * WHEEL_RADIUS * PI, target_velocity[FR]);
	chassis.wheel_current[FL] = pid_cal(&motor_speed_3508[FL], get_motor_data(chassis_move_FL).round_speed * WHEEL_RADIUS * PI, target_velocity[FL]);
	chassis.wheel_current[BL] = pid_cal(&motor_speed_3508[BL], get_motor_data(chassis_move_BL).round_speed * WHEEL_RADIUS * PI, target_velocity[BL]);
	chassis.wheel_current[BR] = pid_cal(&motor_speed_3508[BR], get_motor_data(chassis_move_BR).round_speed * WHEEL_RADIUS * PI, target_velocity[BR]);

	//����������� ��Ӧ��freeRTOS���ͣ�
	set_motor(chassis.wheel_current[FR], chassis_move_FR);
	set_motor(chassis.wheel_current[FL], chassis_move_FL);
	set_motor(chassis.wheel_current[BL], chassis_move_BL);
	set_motor(chassis.wheel_current[BR], chassis_move_BR);
//		set_motor(0, chassis_move_FR);
//		set_motor(0, chassis_move_FL);
//		set_motor(0, chassis_move_BL);
//		set_motor(0, chassis_move_BR);
}
/*******************************�ͽ�תλ***********************************/
void steer_transfer_nearby()
{
//  for (int i = 0; i < 4; i++)
//    {
        
        tmp_delta_angle[0] = obtain_modulus_normalization(chassis.turn_FR.set - chassis.turn_FR.now, 360.0f);
        tmp_delta_angle[1] = obtain_modulus_normalization(chassis.turn_FL.set - chassis.turn_FL.now, 360.0f);
        tmp_delta_angle[2] = obtain_modulus_normalization(chassis.turn_BL.set - chassis.turn_BL.now, 360.0f);
        tmp_delta_angle[3] = obtain_modulus_normalization(chassis.turn_BR.set - chassis.turn_BR.now, 360.0f);

        // ����ת���Ƕȷ�Χ�����Ƿ���Ҫ�ͽ�תλ FR
        if (-90.0f <= tmp_delta_angle[0] && tmp_delta_angle[0] <= 90.0f)
        {

            // ��PI / 2֮�����跴��ͽ�תλ
            chassis.turn_FR.set = tmp_delta_angle[0] + chassis.turn_FR.now;
                      dirt[0] = -1;

        }
        else
        {
            // ��Ҫ��ת��Ȧ���
            chassis.turn_FR.set = obtain_modulus_normalization(tmp_delta_angle[0] + 180.0f, 360.0f) + chassis.turn_FR.now;
            dirt[0] = 1;
        }
        // ����ת���Ƕȷ�Χ�����Ƿ���Ҫ�ͽ�תλ FL
        if (-90.0f <= tmp_delta_angle[1] && tmp_delta_angle[1] <= 90.0f)
        {

            // ��PI / 2֮�����跴��ͽ�תλ
            chassis.turn_FL.set = tmp_delta_angle[1] + chassis.turn_FL.now;
                      dirt[1] = 1;

        }
        else
        {
            // ��Ҫ��ת��Ȧ���
            chassis.turn_FL.set = obtain_modulus_normalization(tmp_delta_angle[1] + 180.0f, 360.0f) + chassis.turn_FL.now;
            dirt[1] = -1;
        }
        // ����ת���Ƕȷ�Χ�����Ƿ���Ҫ�ͽ�תλ BL
        if (-90.0f <= tmp_delta_angle[2] && tmp_delta_angle[2] <= 90.0f)
        {
            // ��PI / 2֮�����跴��ͽ�תλ
            chassis.turn_BL.set = tmp_delta_angle[2] + chassis.turn_BL.now;
            dirt[2] = 1;

        }
        else
        {
            // ��Ҫ��ת��Ȧ���
            chassis.turn_BL.set = obtain_modulus_normalization(tmp_delta_angle[2] + 180.0f, 360.0f) + chassis.turn_BL.now;
            dirt[2] = -1;
        }
        // ����ת���Ƕȷ�Χ�����Ƿ���Ҫ�ͽ�תλ BR
        if (-90.0f <= tmp_delta_angle[3] && tmp_delta_angle[3] <= 90.0f)
        {
            // ��PI / 2֮�����跴��ͽ�תλ
            chassis.turn_BR.set = tmp_delta_angle[3] + chassis.turn_BR.now;
                        dirt[3] = -1;

        }
        else
        {
            // ��Ҫ��ת��Ȧ���
            chassis.turn_BR.set = obtain_modulus_normalization(tmp_delta_angle[3] + 180.0f, 360.0f) + chassis.turn_BR.now;
            dirt[3] = 1;
        }
    
}
/*******************************�ͽ�תλ***********************************/

// ������������
float power_limit(int32_t current[4])
{

	float max_p; // = REFEREE_DATA.Chassis_Power_Limit - 2.0f; // 2w����

	if (cap.remain_vol <= 6)
		max_p = REFEREE_DATA.Chassis_Power_Limit - 2.0f; // 2w����
	else if (cap.remain_vol > 6)
	{
		if (Global.input.fly_status == 1)
			max_p = REFEREE_DATA.Chassis_Power_Limit + cap.remain_vol * 15; // ��������� = �����ѹ * 14A ��Ȧ������
		if (Global.input.fly_status != 1)
			max_p = REFEREE_DATA.Chassis_Power_Limit + cap.remain_vol * 14;
	}

	now_p = 0;

	const float a = 1.23e-07;	// k1
	const float k2 = 1.453e-07; // k2
	const float constant = 4.081f;
	// ���ٱȸ�Ϊ14�ø�ϵ�������ļ��ٱ�Ϊ19Ϊע�ͺ�ϵ��
	const float toque_coefficient = (20.0f / 16384.0f) * (0.3f) * (187.0f / 3591.0f) / 9.55f; // (20/16384)*(0.3)*(187/3591)/9.55=1.99688994e-6f P19
	for (int i = 0; i < 4; i++)
	{
		// ���㹦��
		// ���������֣�https://github.com/MaxwellDemonLin/Motor-modeling-and-power-control/blob/master/chassis_power_control.c#L89

		// ��ǰ���� = ������ * ��������ת��ϵ�� * �ٶ� + K2 * �ٶȵ�ƽ�� + a * ������ƽ���� / Ч��
		// K2 �������ٶ����ޣ����ٳ���������K2
		// a ����������ʱ�����������𲽳���������a
		// Ч��һ�㲻�ö���ԼΪ0.6-0.85
		now_p += fabs(current[i] * toque_coefficient * get_motor_data(i).speed_rpm +
					  k2 * get_motor_data(i).speed_rpm * get_motor_data(i).speed_rpm +
					  a * current[i] * current[i] + constant) /
				 0.85f;
	}

	// ����Ԥ�⹦�ģ���������ı���
	float percentage = max_p / now_p;

	if (percentage > 1.0f)
		return 1.0f;
	return percentage - b; // b ��һ����ѧ�Ĳ��������ڲ�������ٶ�ʱ���ʳ��������
}

//void chassis_receive_from_gimbal(uint8_t data[8])
//{
//	 chassis.speed_RC.x= (data[0] << 8) | data[1];
//    chassis.speed_RC.y = (data[2] << 8) | data[3];
//    chassis.speed_RC.r = (data[4] << 8) | data[5];
//    chassis.speed_RC.yaw = (data[6] << 8) | data[7];
//}
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
		chassis.speed_RC.big_yaw = speed_yaw*PI/180.f;		//��λ rad
}



//void Send_to_Gimbal_yaw2(uint8_t data[8])
//{
//	static uint8_t can_send_data[8];
//	static CAN_TxHeaderTypeDef tx_message;
//	uint32_t send_mail_box;

//	tx_message.StdId = 0x304;
//	tx_message.IDE = CAN_ID_STD;
//	tx_message.RTR = CAN_RTR_DATA;
//	tx_message.DLC = 0x08;  

//	uint32_t temp1;
//	temp1 = IMU_data.AHRS.yaw_rad_cnt;
//	  can_send_data[0] = (temp1 >> 24) & 0xFF;
//    can_send_data[1] = (temp1 >> 16) & 0xFF;
//    can_send_data[2] = (temp1 >> 8) & 0xFF;
//    can_send_data[3] = temp1 & 0xFF;

//	uint32_t temp2;
//	temp2 = IMU_data.AHRS.yaw_angle_cnt;
//	  can_send_data[4] = (temp2 >> 24) & 0xFF;
//    can_send_data[5] = (temp2 >> 16) & 0xFF;
//    can_send_data[6] = (temp2 >> 8) & 0xFF;
//    can_send_data[7] = temp2 & 0xFF;
//	HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
//}

// ����ֵ
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
// ���Ʊ仯��
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

//����С����
//��ȡģ�黯  ת���Ƕȿ�����-PI----PI
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
void ZeroCheck_f(float set_encoder ,float now_encoder, float max)
{
	float angle;
	angle = set_encoder - now_encoder;
	
		if(angle<-(max/2))
		{
			set_encoder+=max;
		}
		else if(angle>(max/2))
		{
			set_encoder-=max;
		}
		else
		{
			set_encoder = set_encoder;
		}
  
}
void ZeroCheck_uint(uint16_t* angle ,uint16_t max)
{
	while((*angle<-(max/2)) ||(*angle>(max/2)))
	{
		if(*angle<-(max/2))
		{
			*angle+=max;
		}
		else if(*angle>(max/2))
		{
			*angle-=max;
		}
  }
}
fp32 Find_min_Angle(fp32 angle1,fp32 angle2)
{
	  fp32 err;
    err = (fp32)angle1 - angle2;
    if(fabs(err) > 4096)
    {
        err = 8192 - fabs(err);
    }
    return err;
}
float Angle_Limit (float angle ,float max)
{
		if(angle > max)
			angle -= max;
		if(angle < 0)
			angle += max; 
		return angle;
}
//float Angle_Limit(float angle, float max)
//{
//    angle = fmodf(angle, max);
//    // �������Ƕ�
//    if(angle < 0)
//        angle += max;
//    
//    return angle;
//}
// ���ֽ�����ת��Ϊfloat
float bytes_to_float(uint8_t *bytes) 
	{
    uint32_t temp = 0;
    temp |= (bytes[0] << 0);  // ����ֽ�
    temp |= (bytes[1] << 8);  // �ڶ����ֽ�
    temp |= (bytes[2] << 16); // �������ֽ�
    temp |= (bytes[3] << 24); // ����ֽ�
    return *(float *)&temp;   // ��uint32_tָ��ǿ��ת��Ϊfloatָ��
	}
	//���·��
	float calculateShortestDistance(float now, float set,float* reverflag) 
	{
		float clock_encoder = fmodf((set-now+8192.0),8192.0);
		float counter_encoder = 8192.0-clock_encoder;
		float reverse_encoder = fabsf(fmodf(set - now + 4096.0, 8192.0)) - 4096.0;
		shortest_encoder = clock_encoder;
		if(counter_encoder<shortest_encoder)
		{
			shortest_encoder = -counter_encoder;
		}
		*reverflag = 1.0;
		//�������������붼Ҫ����2048
		if(fabs(shortest_encoder)>2048.0)
		{
		//��ת
		float flipped_encoder_now = now + 4096.0;
		
		if (flipped_encoder_now >= 8192.0) 
		{
			flipped_encoder_now -= 8192.0;
		}
		if(clock_encoder > counter_encoder)//δ��תǰ������ڷ�����ת��ȡ����
		{
			reverse_encoder = fmodf((set - flipped_encoder_now + 8192.0), 8192.0);//���������
		}
		else
		{
			reverse_encoder = -fmodf(flipped_encoder_now - set + 8192.0, 8192.0);//�������
		}
		*reverflag = -1.0;
        shortest_encoder = reverse_encoder;
		}

    	return shortest_encoder;
	}
	//������
	float ZeroCrossing(float steer_set)
	{
		float steer_last_set;
		float PreError;
		float circle_count;
		PreError = steer_set - steer_last_set;

    // �ж��Ƿ�����Խ��һȦ
    if (PreError > 0.7 * 8192) 
		{
        PreError -= 8192;
        circle_count++;
    } 
		
    // �ж��Ƿ���Խ��һȦ
    else if (PreError < -0.7 * 8192) 
		{
        PreError += 8192;
        circle_count--;
    }

		return (steer_set - circle_count * 8192) ;
    
	}
//	void calculateRoundCnt()    
//{
//	static float last_encoder[4] = {0};
//	float now_encoder[4];
//	float chassis_offset_ecd[4];
//	float circle_cnt[4];
//	chassis_offset_ecd[0] = TURN_FR_ANGLE;
//	chassis_offset_ecd[1] = TURN_FL_ANGLE;
//	chassis_offset_ecd[2] = TURN_BL_ANGLE;
//	chassis_offset_ecd[3] = TURN_BR_ANGLE;
//	for(uint8_t i=0;i<4;i++)
//	{
//		now_encoder[i] = get_motor_data(i).ecd -	chassis_offset_ecd[i];
//		now_encoder[i] = now_encoder[i]/8192*360;
//		
//		if(now_encoder[i] - last_encoder[i] > 180)
//			circle_cnt[i] --;
//		else if(now_encoder[i] - last_encoder[i] < -180) 
//			circle_cnt[i] ++;
//		
//		last_encoder[i] = now_encoder[i];
//		chassis_handle->steeringAngle[i] = -now_encoder[i] -  circle_cnt[i]*360;
//		chassis_handle->chassis_steer_motor[i].sensor.relative_angle=chassis_handle->steeringAngle[i]*-1;
//	}
//}
/*****************************************************/
	//���ת���ڲ࣬��������
	fp32 Find_min_angle(fp32 angle1,fp32 angle2)
	{
		fp32 err;
		err = angle1 - angle2;
		if(fabs(err)>4096)
		{
			err = 8192 - fabs(err) ;
		}
		return err;
	}
	
	//���ӻ��ж�
//	void arc_justment()
//	{
//		if(fabs(Find_min_Angle(chassis_move_control_Speed->motor_chassis[4].chassis_motor_measure->ecd,chassis_move_control_Speed->AGV_wheel_Angle[0]))>2048)
//	{
//		for(int i=0;i<4;i++)
//		{
//			chassis_move_control_Speed->AGV_wheel_Angle[i] += 4096;		
//			chassis_move_control_Speed->AGV_wheel_Angle[i]=Angle_Limit(chassis_move_control_Speed->AGV_wheel_Angle[i],8192);
//		}
//			chassis_move_control_Speed->drct = -1;
//	}
//	else
//			chassis_move_control_Speed->drct=1;
//	}
/******************************************************/
	
/*********************���º���û�б��õ��������Է��Ժ��õ�***********************/
// ����С���� ������
float random_anti_vision_r_s(float min, float max)
{
	static int run_count = 100;
	run_count++;
	if (run_count > refresh_interval && smaller_than_2_count < 2)
	{
		valve = generate_random_float(min, max); // ���޺�����

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

// ��ָ������ʽȥ���������Ǹ������������acc=ae^bv-a+c,��aΪ����cΪ��ʼ���ٶ�,�����ٶ�Ϊ�Ա���
float fly_speed_up_ex(float a, float b, float c, float v)
{
	return a * exp(b * v) - a + c;
}
// �տ�ʼʱĬ�ϼ��ٶȣ�Ȼ�����Խ��Խ��
// ����ֵ��ÿ�����м��ٶ�����ֵ���ٶ�����ֵ
float fly_speed_up(float *set, float acceleration_increase_rate, float limit)
{
	static float initial_acceleration = 0.2; // ��ʼ���ٶ���Ϊ0.2//ÿ������+���ٶ�;
	if (set == 0)							 // Ҳ���Ǵ�ͷ��ʼ�ˣ��Ǿ����ü��ٶ�
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

/*��ֵĶ��ຯ������ȥʵ�飬û�þ�ɾ��*/


