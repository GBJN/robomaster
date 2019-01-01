#ifndef __CANBUSTASK_H
#define __CANBUSTASK_H



#include "Config.h"


#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID           0x202
#define CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS2_BELTMOTOR1_FEEDBACK_MSG_ID				0x205
#define CAN_BUS2_BELTMOTOR2_FEEDBACK_MSG_ID				0x206



#define CAN_SEND_NUM 3


#define RATE_BUF_SIZE 6
typedef struct{
	int16_t	 	speed_rpm;//转速
	int16_t  	real_current;//实际转矩
	uint8_t  	Temperature;
	uint16_t 	angle;//角度
	uint16_t  lastangle;
	int32_t   ecd_angle;
	int16_t   round_cnt;
}Measure;


void Can_Send_CM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4);
void Can_Send(void);
void Can_Send_Task(void const * argument);
void Can_Msg_Process(void);
void Can_Send_BM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4);

extern Measure Motor1_Measure;
extern Measure Motor2_Measure;
extern Measure Motor3_Measure;
extern Measure Motor4_Measure;

extern Measure LBeltM_Measure;
extern Measure RBeltM_Measure;

#endif


