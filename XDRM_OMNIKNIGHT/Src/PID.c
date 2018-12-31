
/* Includes ------------------------------------------------------------------*/

#include "PID.h"





void PID_Reset(PID_Regulator_t *pid)
{
	pid->err[0]=0;
	pid->err[1]=0;
	pid->err[2]=0;
	pid->output=0;
}

void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[0]=pid->err[1];//�ϴ����
	pid->err[1]=pid->ref-pid->fdb;//�������
	pid->err[2]+=pid->err[1];//������
	VAL_LIMIT(pid->ki,-pid->componentKiMax,pid->componentKiMax);	//�����ͻ���
	pid->output=pid->kp*pid->err[1]+pid->ki*pid->err[2]+pid->kd*(pid->err[1]-pid->err[0]);	//���һ�������΢��
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













