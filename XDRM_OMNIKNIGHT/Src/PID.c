
/* Includes ------------------------------------------------------------------*/

#include "PID.h"




PID_Regulator_t RBMSpeedPID = BELT_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t LBMSpeedPID = BELT_MOTOR_SPEED_PID_DEFAULT;//位置环要不要加以后再说
PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t AMRotatePID = CHASSIS_MOTOR_SPEED_PID_DEFAULT; 














void PID_Reset(PID_Regulator_t *pid)
{
	pid->err[0]=0;
	pid->err[1]=0;
	pid->err[2]=0;
	pid->err[3]=0;
	pid->output=0;
}


void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[3]=pid->err[2];//只有在增量式pid中才用到了上上次误差及err3
	pid->err[2]=pid->err[1];//上次误差
	pid->err[1]=pid->ref-pid->fdb;//本次误差
	pid->err[0]+=pid->err[1];//误差积分
	VAL_LIMIT(pid->ki,-pid->componentKiMax,pid->componentKiMax);	//抗饱和积分
	if(pid->type == POSITION_PID)
	{
		pid->output=pid->kp*pid->err[1]+pid->ki*pid->err[2]+pid->kd*(pid->err[1]-pid->err[0]);	//最后一个是误差微分
	}
	else
		pid->output = pid->kp*(pid->err[1]-pid->err[2])+pid->ki*pid->err[1]+pid->kd*(pid->err[1] - 2*pid->err[2]+pid->err[3]);//增量式pid
	VAL_LIMIT(pid->output,-pid->outputMax,pid->outputMax);
}
/**
	* @brief PID计算
	* @param PID_Regulator_t *PID_Stucture
	* @param float ref
	* @param float fdb
	* @retval float output
*/
float PID_Task(PID_Regulator_t *PID_Stucture, float ref, float fdb)
{
	PID_Stucture->ref = ref;
	PID_Stucture->fdb = fdb;
	PID_Stucture->Calc(PID_Stucture);
	return PID_Stucture->output;
}





//网上有说为了抗微小扰动可以在反馈加一个低通滤波，但会让反应减慢

//ki != Ti
//kd != Td

//另一种写法是：
//#define Kpp   Kp * ( 1 + (T / Ti) + (Td / T) )
//#define Ki   (-Kp) * ( 1 + (2 * Td / T ) )
//#define Kd    Kp * Td / T


//Ek = ref - feb;
//Uo = Kpp*Ek + Ki*Ei + Kd*Ed;//PID计算
//Ed = Ei;
//Ei = Ek;

//而我们是kp*差值+ki*差值积分+kd*差值微分













