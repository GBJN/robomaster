#include "Driver_Beltraise.h"

#include "PID.h"
#include "CanBusTask.h"

#include "Driver_Remote.h"

int16_t BeltMotorSpeedRef[2] = {0,0};


PID_Regulator_t RBMSpeedPID = BELT_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t LBMSpeedPID = BELT_MOTOR_SPEED_PID_DEFAULT;//位置环要不要加以后再说
int kp = 60;

//现在要想一下整个工程大概怎么写了。
void BM_Get_PID(void)
{

	RBMSpeedPID.kp = kp;//60
	RBMSpeedPID.ki = 0;
	RBMSpeedPID.kd = 0;//
	
	
	LBMSpeedPID.kp = kp;//60
	LBMSpeedPID.ki = 0;
	LBMSpeedPID.kd = 0;//

}




void 	BM_Calc_Output(void)
{
	PID_Task(&LBMSpeedPID,BeltMotorSpeedRef[0],LBeltM_Measure.speed_rpm/10.0);//float /10.0
	PID_Task(&RBMSpeedPID,BeltMotorSpeedRef[1],RBeltM_Measure.speed_rpm/10.0);

	if(LBMSpeedPID.output>-800 && LBMSpeedPID.output<800 ) LBMSpeedPID.output=0;
	if(RBMSpeedPID.output>-800 && RBMSpeedPID.output<800 ) RBMSpeedPID.output=0;


}
//还要改！
extern uint8_t S_switch;
uint16_t raise_speed = 250;
uint8_t Foward_flag = 0;
void BM_Get_SpeedRef(void)
{	
	if(RC_CtrlData.rc.s1 == 3)
	{
		BeltMotorSpeedRef[0] = RC_CtrlData.rc.ch3/2;
		BeltMotorSpeedRef[1] = -RC_CtrlData.rc.ch3/2;
	}
	else if(RC_CtrlData.rc.s1 == 1)//或者
	{
		BeltMotorSpeedRef[0] = raise_speed;//250
	  BeltMotorSpeedRef[1] = -raise_speed;
		static int32_t LBM_Angle = 0;
		static int32_t RBM_Angle = 0;
		static uint16_t count = 0;
		if(S_switch == 1)
		{
			count++;
		}
		if(count>13)//1000/72
		{
			LBM_Angle = LBeltM_Measure.ecd_angle;
			RBM_Angle = RBeltM_Measure.ecd_angle;
			
			count = 0;
		}
		if(abs(LBeltM_Measure.ecd_angle - LBM_Angle) > THRESHOLD)//9000
		{
			BeltMotorSpeedRef[0] = 100;
			BeltMotorSpeedRef[1] = -100;//120也行
			Foward_flag = 1;
		}
	}
	else
	{
	
		BeltMotorSpeedRef[0] = 0;
		BeltMotorSpeedRef[1] = -0;
}
		
}


void BM_Set_Current(void)
{
		if(RC_CtrlData.rc.s1 == 2)
		{
			Can_Send_BM(0,0,0,0);
		}
		else
		{
			Can_Send_BM(RBMSpeedPID.output,LBMSpeedPID.output,0,0);//不要在这里output加负号
	//		Can_Send_BM(-LBMSpeedPID.output,RBMSpeedPID.output,0,0);
		}


}


void Belt_Control(void)
{
	BM_Get_PID();
	BM_Get_SpeedRef();
	BM_Calc_Output();
	BM_Set_Current();

}




















