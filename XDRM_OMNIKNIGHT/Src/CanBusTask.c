#include "CanBusTask.h"



#include "can.h"
#include "BSP_Data.h"











Measure Motor1_Measure = {0,0,0,0};
Measure Motor2_Measure = {0,0,0,0};
Measure Motor3_Measure = {0,0,0,0};
Measure Motor4_Measure = {0,0,0,0};

Measure Turntable_Measure = {0,0,0,0};


Encoder CM1Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Encoder CM2Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Encoder CM3Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Encoder CM4Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Encoder GMYawEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Encoder GMPitchEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};
Encoder TurntableEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};


/**
  * @brief  处理编码值,将其转换为连续的角度							//[0][1]机械角度
  * @param  msg电机由CAN回传的信息 v编码器结构体				//[2][3]实际转矩电流测量值
  * @retval void																			//[4][5]转矩电流给定值
  */
void get_measure(Measure *mea, Can_Msg *msg)//查看C620说明书
{
	mea->angle = (uint16_t)(msg->data[0] << 8 | msg->data[1]);
	mea->speed_rpm = (int16_t)(msg->data[2] << 8 | msg->data[3]);
	mea->real_current = (int16_t)((msg->data[4] << 8) | (msg->data[5]));
	mea->Temperature = msg->data[6];
}



void GetEncoderBias(Encoder *v,Can_Msg *msg)
{
	v->ecd_bias = (msg->data[0]<<8)|(msg->data[1]);
	v->ecd_value = v->ecd_bias;
	v->last_raw_value = v->ecd_bias;
	v->temp_count++;
}



void EncoderProcess(Encoder *v,Can_Msg *msg)//yaw轴转，pitch的pid也会变，ecd-angle的问题
{
	int i = 0;
	int32_t temp_sum = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->data[0]<<8)|(msg->data[1]);
	v->diff =  v->raw_value-v->last_raw_value;
	if(v->diff < -6400)
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff > 6400)
	{
		v->round_cnt--;//这里写成了++然后yaw和pitch有问题，ecd_angle只要你快一点来回转yaw和pitch，就会叠加，然后变得很大。
		v->ecd_raw_rate = v->diff - 8192;
	}
	else
	{
	  v->ecd_raw_rate = v->diff;
	}
	v->ecd_value = v->raw_value+ v->round_cnt * 8192;
	v->ecd_angle = (float)(v->raw_value-v->ecd_bias) * 360.0f/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	for (i = 0; i < RATE_BUF_SIZE; i++)//我觉得这里有问题，6次0一次速率。									
	{																	 //没问题的，他只是回到数组头而已，其实就是滑动滤波
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum / RATE_BUF_SIZE);

}


/**
  * @needs	处理并发送云台、底盘、拨盘电机电流或电压值，接收并处理电调回传的电机信息，帧率均为1k，并为其他通信留空间
  * @brief  CAN1、CAN2
  * @note   CAN总线发送:底盘电机4*M3508 C620电调,发送 data∈(-16384,16384) 0x4000	20A
  * 				0x200或     云台电机自带电调GM3510,data∈(-29000,29000) 0x7148  				云台电机标识符需用0x1FF,ID5为YAWID6为PITCH 3510ID可以是5/6/7
	*         0x1FF(各对           			 RM6623,data∈(-5000,5000)   0x1388         
  *         应四个ID的 	拨盘电机RM2006  C610电调  发送 data∈(-10000,10000) 0x2710  10A
  * 				电调)1-4 5-8
	*        
  *         CAN总线接收:			C610	  	C620 0x200+ID号		GM6623 P 0x206	Y 0x206	  GM3510	0x204+1/2/3																												
  *         data[0]data[1]  机械角度		机械角度						机械角度										机械角度								0-8191 360°
	*        	data[2]data[3]	转子转速		转子转速						实际电流		13000						输出转矩
  *         data[4]data[5]	输出转矩		实际电流						给定电流
	*         data[6]data[7]						温度
  * @param  None  朱利豪
  */

//接收不需要怎么改
//发送的形式想好了，直接把id也push进去然后pop的时候注意就行了
//由频率决定发送的时机,想好了不知道可以不。
//还要做的就是CAN2


uint32_t can_count;



























void Can_Msg_Process(void)
{
	can_count++;
	switch(CAN1_Receive.rx_header.StdId)
	{
		case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
			{
	//			ChassisFrameCounter[0]++;

				get_measure(&Motor1_Measure, &CAN1_Receive.msg);

			}
			break;
			case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
			{
	//			ChassisFrameCounter[1]++;
				get_measure(&Motor2_Measure, &CAN1_Receive.msg);

			}
			break;
			case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
			{
	//			ChassisFrameCounter[2]++;
				get_measure(&Motor3_Measure, &CAN1_Receive.msg);

			}
			break;
			case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
			{
	//			ChassisFrameCounter[3]++;
				get_measure(&Motor4_Measure, &CAN1_Receive.msg);
			}
			break;
			case CAN_BUS2_GMYAW_FEEDBACK_MSG_ID:
			{
	//			YawFrameCounter++;
				static uint32_t yaw_count = 0;
				EncoderProcess(&GMYawEncoder, &CAN1_Receive.msg);
				// 比较保存编码器的值和偏差值，如果编码器的值和初始偏差之间差距超过阈值，将偏差值做处理，防止出现云台反方向运动
				if (yaw_count++<=100 || can_count < 100) //准备阶段要求二者之间的差值一定不能大于阈值，否则肯定是出现了临界切换
				{
					if ((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) < -4000)
					{
	//					GMYawEncoder.ecd_bias = gAppParamStruct.GimbalCaliData.GimbalYawOffset + 8192;
					}
					else if ((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) > 4000)
					{
//						GMYawEncoder.ecd_bias = gAppParamStruct.GimbalCaliData.GimbalYawOffset - 8192;
					}
				}
			}
			break;
			case CAN_BUS2_GMPITCH_FEEDBACK_MSG_ID:
			{
				static  uint32_t pitch_count = 0;
//				PitchFrameCounter++;
				EncoderProcess(&GMPitchEncoder, &CAN1_Receive.msg);
				//码盘中间值设定也需要修改
				if (pitch_count++ <= 100 || can_count < 100)
				{
					if ((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) < -4000)
					{
//					GMPitchEncoder.ecd_bias = gAppParamStruct.GimbalCaliData.GimbalPitchOffset + 8192;
					}
					else if ((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) > 4000)
					{
//						GMPitchEncoder.ecd_bias = gAppParamStruct.GimbalCaliData.GimbalPitchOffset - 8192;
					}
				}
			}
			break;
			case CAN_BUS2_TURNTABLE_FEEDBACK_MSG_ID:
			{
				get_measure(&Turntable_Measure, &CAN1_Receive.msg);
				EncoderProcess(&TurntableEncoder, &CAN1_Receive.msg);
		//		TurntableFrameCounter++;
			}
			break;
			default:
			{
	
			}
			break;
		}
}

//void Can_Send_Other(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4)
//{
//	CAN1_ReadyToSend.tx_header.StdId = 0x198;
//	CAN1_ReadyToSend.msg.data[0] = (unsigned char)( iq_1 >>8);
//	CAN1_ReadyToSend.msg.data[1] = (unsigned char)iq_1;
//	CAN1_ReadyToSend.msg.data[2] = (unsigned char)( iq_2 >>8);
//	CAN1_ReadyToSend.msg.data[3] = (unsigned char)iq_2;
//	CAN1_ReadyToSend.msg.data[4] = (unsigned char)( iq_3 >>8);
//	CAN1_ReadyToSend.msg.data[5] = (unsigned char)iq_3;
//	CAN1_ReadyToSend.msg.data[6] = (unsigned char)( iq_4 >>8);
//	CAN1_ReadyToSend.msg.data[7] = (unsigned char)iq_4;
//	
//	CAN_bufferPush(&Que_CAN1_Tx,CAN1_ReadyToSend);

//}


//void Can_Send_CM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4)
//{
//	CAN1_ReadyToSend.tx_header.StdId = 0x200;
//	CAN1_ReadyToSend.msg.data[0] = (unsigned char)( iq_1 >>8);
//	CAN1_ReadyToSend.msg.data[1] = (unsigned char)iq_1;
//	CAN1_ReadyToSend.msg.data[2] = (unsigned char)( iq_2 >>8);
//	CAN1_ReadyToSend.msg.data[3] = (unsigned char)iq_2;
//	CAN1_ReadyToSend.msg.data[4] = (unsigned char)( iq_3 >>8);
//	CAN1_ReadyToSend.msg.data[5] = (unsigned char)iq_3;
//	CAN1_ReadyToSend.msg.data[6] = (unsigned char)( iq_4 >>8);
//	CAN1_ReadyToSend.msg.data[7] = (unsigned char)iq_4;
//	
//	CAN_bufferPush(&Que_CAN1_Tx,CAN1_ReadyToSend);
//	
//}



//GM3510 电压范围-29000 - 29000
//0x7148  111000101001000
//怎么传负值？ 补码
void Can_Send_GM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4)
{
	CAN1_ReadyToSend.tx_header.StdId = 0x1FF;//这个变量以后再说小问题
	CAN1_ReadyToSend.msg.data[0] = (unsigned char)( iq_1 >>8);
	CAN1_ReadyToSend.msg.data[1] = (unsigned char)iq_1;
	CAN1_ReadyToSend.msg.data[2] = (unsigned char)( iq_2 >>8);
	CAN1_ReadyToSend.msg.data[3] = (unsigned char)iq_2;
	CAN1_ReadyToSend.msg.data[4] = (unsigned char)( iq_3 >>8);
	CAN1_ReadyToSend.msg.data[5] = (unsigned char)iq_3;
	CAN1_ReadyToSend.msg.data[6] = (unsigned char)( iq_4 >>8);
	CAN1_ReadyToSend.msg.data[7] = (unsigned char)iq_4;
	
	CAN_bufferPush(&Que_CAN1_Tx,CAN1_ReadyToSend);
	
}


uint32_t TxMailFreeNum = 0;
uint32_t count;
uint32_t can_send_rate;
extern uint32_t send_rate;
void Can_Send(void)
{		
	static int16_t q;
	static int16_t i =2000;
		i--;
		q = i*1.5;


	
	Can_Send_GM(q,q,q,q);
//	Can_Send_CM(0,0,0,0);
//	Can_Send_Other(q,q,q,q);
//	
	uint16_t testlen = CAN_bufferlen(&Que_CAN1_Tx);
//如果都在一个线程里发的话等发的数据多了就会不够用了
//想不出更好方法，暂且就这样，can线程里面发一次，剩下的到中断里发
//时间触发模式好像可以，但是没必要而且数据就少了
		CAN_bufferPop(&Que_CAN1_Tx,&CAN1_ReallySend);

			HAL_CAN_AddTxMessage(&hcan1,&CAN1_ReallySend.tx_header,CAN1_ReallySend.msg.data,(uint32_t*)CAN_TX_MAILBOX0);//若0邮箱满了，就自动加到下一个


		if(i == 1)
		{
			i = 2000;
		}
		
		
	TxMailFreeNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

}






/**
* @brief 所需实现的功能：每ms接收来自
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Can_Send_Task */
uint16_t aa = 0;
uint16_t bb = 0;
void Can_Send_Task(void const * argument)
{

  /* USER CODE BEGIN Can_Send_Task */
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {

			
			Can_Send();
			
			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//此时处于阻塞态

	

		
		
		
		
  }
  /* USER CODE END Can_Send_Task */
}
