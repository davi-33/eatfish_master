#include "math.h"
#include "pid.h"

void pid_set(pid_t *PidSet,float p_set,float i_set,float d_set,float lim_out_set,float lim_i_outset)//PID设置
{
  PidSet->p = p_set;   PidSet->i = i_set;   PidSet->d = d_set;
//	PidSet->p_out = 0.0f;
//	PidSet->i_out = 0.0f;
//	PidSet->d_out = 0.0f;
  //PidSet->total_out = 0.0f;
  //PidSet->set = 0.0f;
  PidSet->lim_out = lim_out_set;   PidSet->lim_i_out = lim_i_outset;//将设置赋值
}

//PID计算
float pid_cal(pid_t *PidGoal,float Now,float Set)//PID??
{
	PidGoal->set = Set;
  PidGoal->err_last = PidGoal->err;
  PidGoal->err = Set - Now;//计算误差
  PidGoal->lastdiff=PidGoal->diff;
  PidGoal->diff=PidGoal->err-PidGoal->err_last;
  
//  if(PidGoal->diff>0.9f)//微分环节低通滤波
//    PidGoal->diff=0.9f;
 
  PidGoal->p_out = PidGoal->p * PidGoal->err;
	if(PidGoal->i != 0)
		PidGoal->i_out += PidGoal->i * PidGoal->err;
  PidGoal->d_out = PidGoal->d *PidGoal->diff ;//pid运算
  
  
  if(fabs(PidGoal->i_out) > PidGoal->lim_i_out)//防止积分过大
  {
    if(PidGoal->i_out < 0)
      PidGoal->i_out = -PidGoal->lim_i_out;
    else
      PidGoal->i_out = PidGoal->lim_i_out;
  }
	
  PidGoal->total_out = PidGoal->p_out + PidGoal->i_out + PidGoal->d_out;//计算总和输出  

	if(fabs(PidGoal->total_out) > PidGoal->lim_out)//防止总和过大
  {
    if(PidGoal->total_out < 0)
      PidGoal->total_out = -PidGoal->lim_out;
    else
      PidGoal->total_out = PidGoal->lim_out;
  }
	
  return PidGoal->total_out;
}

float calculate_feedforward(pid_t* pid, float setpoint) 
{
    float delta_setpoint = setpoint - pid->prev_setpoint;
    pid->prev_setpoint = setpoint;  // 更新记录
    return pid->Kf * delta_setpoint; // 前馈量 = Kf * 设定值变化率
}//融合DeepSeek生成简单前馈算法数据

float pid_update(pid_t* pid, float setpoint, float measurement) 
{
    float error = setpoint - measurement;
    float Up = pid->Kp * error + pid->total_out;  // 原PID计算
    float Uf = calculate_feedforward(pid, setpoint); // 前馈计算
    return Up + Uf;  // 合并输出
}//简单前馈最终输出总和