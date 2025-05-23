#ifndef __RM_CILENT_UI__
#define __RM_CILENT_UI__

#include "stm32f4xx.h"
#include "stdarg.h"
#include "usart.h"
#include "struct_typedef.h"
#include "referee_handle_pack.h"
#pragma pack(1) // 按1字节对齐

#define NULL 0
#define __FALSE 100

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x0110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0      // 直线
#define UI_Graph_Rectangle 1 // 矩形
#define UI_Graph_Circle 2    // 整圆
#define UI_Graph_Ellipse 3   // 椭圆
#define UI_Graph_Arc 4       // 圆弧
#define UI_Graph_Float 5     // 浮点型
#define UI_Graph_Int 6       // 整形
#define UI_Graph_Char 7      // 字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0 // 红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 // 紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6 // 青色
#define UI_Color_Black 7
#define UI_Color_White 8

extern uint16_t Robot_ID;
extern uint16_t Cilent_ID;

typedef unsigned char Uint8_t;
typedef unsigned char UINT8_T;

typedef struct
{
   uint8_t SOF;          // 起始字节,固定0xA5
   uint16_t Data_Length; // 帧数据长度
   uint8_t Seq;          // 包序号
   uint8_t CRC8;         // CRC8校验值
   uint16_t CMD_ID;      // 命令ID
} UI_Packhead;           // 帧头

typedef struct
{
   uint16_t Data_ID;     // 内容ID
   uint16_t Sender_ID;   // 发送者ID
   uint16_t Receiver_ID; // 接收者ID
} UI_Data_Operate;       // 操作定义帧

typedef struct
{
   uint8_t Delete_Operate; // 删除操作
   uint8_t Layer;          // 删除图层
} UI_Data_Delete;          // 删除图层帧


typedef struct
{
   uint8_t graphic_name[3];
   uint32_t operate_tpye : 3;
   uint32_t graphic_tpye : 3;
   uint32_t layer : 4;
   uint32_t color : 4;
   uint32_t start_angle : 9;
   uint32_t end_angle : 9;
   uint32_t width : 10;
   uint32_t start_x : 11;
   uint32_t start_y : 11;
   float graph_Float; // 浮点数据
} Float_Data;


typedef __packed struct
{
   uint8_t graphic_name[3];
   uint32_t operate_tpye : 3;
   uint32_t graphic_tpye : 3;
   uint32_t layer : 4;
   uint32_t color : 4;
   uint32_t start_angle : 9;
   uint32_t end_angle : 9;
   uint32_t width : 10;
   uint32_t start_x : 11;
   uint32_t start_y : 11;
   uint32_t radius : 10;
   uint32_t end_x : 11;
   uint32_t end_y : 11; // 图形数据
} Graph_Data;

typedef __packed struct
{
   Graph_Data Graph_Control;
   uint8_t show_Data[30];
} String_Data; // 打印字符串数据




void UI_Delete(uint8_t Del_Operate, uint8_t Del_Layer);
int UI_ReFresh(int cnt, ...);
int Char_ReFresh(String_Data string_Data);
void Line_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
void Rectangle_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
void Char_Draw(String_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *Char_Data);
void Arc_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);
void Circle_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);













void press_refrsh(void);
void infanty_hero_status_UI_add(void);
void UI_task_init(void);


void shoot_refresh(int16_t status);//射击更新
void trigger_refresh(int16_t status);//拨弹更新


void CHASSIS_ReFresh(void); //图示底盘刷新
void Chassis_Refresh(float angle);//图示底盘更新
void chassis_status_refresh(uint8_t status);//底盘模式切换更新

void Auto_Frame(void);//自瞄框刷新

void auto_refresh(void);//其他字符串刷新
void Auto_aim_circle_refresh(uint8_t status); 
void Super_POWER(float cap_voltage);//电量更新

void auto_aim_ui_update(bool_t yes_or_no);//自瞄更新

void super_cap_speedup(bool_t yes_or_no);//BUFF更新

void cover(uint16_t status);//弹舱盖更新

void fly(uint16_t status); //弹舱盖更新


void Base_sentry_Refresh(uint8_t robot_id,ext_game_robot_HP_t game_robot_HP_t); 

void ammo_update(uint16_t ammo_reserve);
void anti( uint16_t status);
int sentry_send(void);

void UI_id_check(void);


#endif
