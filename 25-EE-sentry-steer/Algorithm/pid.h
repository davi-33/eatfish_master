typedef struct//PID控制数据结构体
{
  float p,i,d;//比例，积分，微分
  float set,err,err_last,get;//设置值，误差值，上次的误差值
  float diff,lastdiff,filter;//微分值，上次微分值，滤波参数（不知道有没有用）
  float p_out,i_out,d_out,total_out;//比例，积分，微分的输出和总输出
  float lim_i_out,lim_out;//输出积分限制，总输出限制
  
  float Kp, Ki, Kd;      // PID参数 
  float Kf;              // 前馈增益（新增）
  float prev_setpoint;   // 上次设定值（新增）
}pid_t;



extern pid_t pitch_speed_pid;
extern pid_t pitch_location_pid;

extern pid_t yaw_speed_pid;
extern pid_t yaw_location_pid;

extern pid_t trigger_speed_pid;
extern pid_t trigger_location_pid;

extern pid_t shoot1_speed_pid;
extern pid_t shoot2_speed_pid;



void pid_set(pid_t *PidSet,float p_set,float i_set,float d_set,float lim_out_set,float lim_i_outset);//PID设置
float pid_cal(pid_t *PidGoal,float Now,float Set);//PID计算

float calculate_feedforward(pid_t* pid, float setpoint);
float pid_update(pid_t* pid, float setpoint, float measurement);