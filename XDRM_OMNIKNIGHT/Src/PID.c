
/* Includes ------------------------------------------------------------------*/

#include "PID.h"




PID_Regulator_t RBMSpeedPID = BELT_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t LBMSpeedPID = BELT_MOTOR_SPEED_PID_DEFAULT;//λ�û�Ҫ��Ҫ���Ժ���˵
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
	pid->err[3]=pid->err[2];//ֻ��������ʽpid�в��õ������ϴ���err3
	pid->err[2]=pid->err[1];//�ϴ����
	pid->err[1]=pid->ref-pid->fdb;//�������
	pid->err[0]+=pid->err[1];//������
	VAL_LIMIT(pid->ki,-pid->componentKiMax,pid->componentKiMax);	//�����ͻ���
	if(pid->type == POSITION_PID)
	{
		pid->output=pid->kp*pid->err[1]+pid->ki*pid->err[2]+pid->kd*(pid->err[1]-pid->err[0]);	//���һ�������΢��
	}
	else
		pid->output = pid->kp*(pid->err[1]-pid->err[2])+pid->ki*pid->err[1]+pid->kd*(pid->err[1] - 2*pid->err[2]+pid->err[3]);//����ʽpid
	VAL_LIMIT(pid->output,-pid->outputMax,pid->outputMax);
}
/**
	* @brief PID����
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





//������˵Ϊ�˿�΢С�Ŷ������ڷ�����һ����ͨ�˲��������÷�Ӧ����

//ki != Ti
//kd != Td

//��һ��д���ǣ�
//#define Kpp   Kp * ( 1 + (T / Ti) + (Td / T) )
//#define Ki   (-Kp) * ( 1 + (2 * Td / T ) )
//#define Kd    Kp * Td / T


//Ek = ref - feb;
//Uo = Kpp*Ek + Ki*Ei + Kd*Ed;//PID����
//Ed = Ei;
//Ei = Ek;

//��������kp*��ֵ+ki*��ֵ����+kd*��ֵ΢��













