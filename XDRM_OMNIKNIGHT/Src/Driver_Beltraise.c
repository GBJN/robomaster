#include "Driver_Beltraise.h"

#include "PID.h"
#include "CanBusTask.h"




PID_Regulator_t BM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t BM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//位置环要不要加以后再说


//现在要想一下整个工程大概怎么写了。
void BM_Get_PID(void)
{

	BM1SpeedPID.kp = 10;//60
	BM1SpeedPID.ki = 0;
	BM1SpeedPID.kd = 0;//
	
	
	BM2SpeedPID.kp = 10;//60
	BM2SpeedPID.ki = 0;
	BM2SpeedPID.kd = 0;//

}




void 	BM_Calc_Output(void)
{
//	PID_Task(&CM1SpeedPID,BeltData.,Motor1_Measure.speed_rpm/10.0);//float /10.0
//	PID_Task(&CM2SpeedPID,ChassisData.ChassisWheelSpeedRef[1],Motor2_Measure.speed_rpm/10.0);



}


void BM_Get_SpeedRef(void)
{



}


void BM_Set_Current(void)
{
	Can_Send_BM(0,0,0,0);


}



void Belt_Control(void)
{
	BM_Get_PID();
	BM_Get_SpeedRef();
	BM_Calc_Output();
	BM_Set_Current();

}




















