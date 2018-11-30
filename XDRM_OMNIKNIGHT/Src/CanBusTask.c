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
  * @brief  �������ֵ,����ת��Ϊ�����ĽǶ�							//[0][1]��е�Ƕ�
  * @param  msg�����CAN�ش�����Ϣ v�������ṹ��				//[2][3]ʵ��ת�ص�������ֵ
  * @retval void																			//[4][5]ת�ص�������ֵ
  */
void get_measure(Measure *mea, Can_Msg *msg)//�鿴C620˵����
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



void EncoderProcess(Encoder *v,Can_Msg *msg)//yaw��ת��pitch��pidҲ��䣬ecd-angle������
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
		v->round_cnt--;//����д����++Ȼ��yaw��pitch�����⣬ecd_angleֻҪ���һ������תyaw��pitch���ͻ���ӣ�Ȼ���úܴ�
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
	for (i = 0; i < RATE_BUF_SIZE; i++)//�Ҿ������������⣬6��0һ�����ʡ�									
	{																	 //û����ģ���ֻ�ǻص�����ͷ���ѣ���ʵ���ǻ����˲�
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum / RATE_BUF_SIZE);

}


/**
  * @needs	����������̨�����̡����̵���������ѹֵ�����ղ��������ش��ĵ����Ϣ��֡�ʾ�Ϊ1k����Ϊ����ͨ�����ռ�
  * @brief  CAN1��CAN2
  * @note   CAN���߷���:���̵��4*M3508 C620���,���� data��(-16384,16384) 0x4000	20A
  * 				0x200��     ��̨����Դ����GM3510,data��(-29000,29000) 0x7148  				��̨�����ʶ������0x1FF,ID5ΪYAWID6ΪPITCH 3510ID������5/6/7
	*         0x1FF(����           			 RM6623,data��(-5000,5000)   0x1388         
  *         Ӧ�ĸ�ID�� 	���̵��RM2006  C610���  ���� data��(-10000,10000) 0x2710  10A
  * 				���)1-4 5-8
	*        
  *         CAN���߽���:			C610	  	C620 0x200+ID��		GM6623 P 0x206	Y 0x206	  GM3510	0x204+1/2/3																												
  *         data[0]data[1]  ��е�Ƕ�		��е�Ƕ�						��е�Ƕ�										��е�Ƕ�								0-8191 360��
	*        	data[2]data[3]	ת��ת��		ת��ת��						ʵ�ʵ���		13000						���ת��
  *         data[4]data[5]	���ת��		ʵ�ʵ���						��������
	*         data[6]data[7]						�¶�
  * @param  None  ������
  */

//���ղ���Ҫ��ô��
//���͵���ʽ����ˣ�ֱ�Ӱ�idҲpush��ȥȻ��pop��ʱ��ע�������
//��Ƶ�ʾ������͵�ʱ��,����˲�֪�����Բ���
//��Ҫ���ľ���CAN2


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
				// �Ƚϱ����������ֵ��ƫ��ֵ�������������ֵ�ͳ�ʼƫ��֮���೬����ֵ����ƫ��ֵ��������ֹ������̨�������˶�
				if (yaw_count++<=100 || can_count < 100) //׼���׶�Ҫ�����֮��Ĳ�ֵһ�����ܴ�����ֵ������϶��ǳ������ٽ��л�
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
				//�����м�ֵ�趨Ҳ��Ҫ�޸�
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



//GM3510 ��ѹ��Χ-29000 - 29000
//0x7148  111000101001000
//��ô����ֵ�� ����
void Can_Send_GM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4)
{
	CAN1_ReadyToSend.tx_header.StdId = 0x1FF;//��������Ժ���˵С����
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
//�������һ���߳��﷢�Ļ��ȷ������ݶ��˾ͻ᲻������
//�벻�����÷��������Ҿ�������can�߳����淢һ�Σ�ʣ�µĵ��ж��﷢
//ʱ�䴥��ģʽ������ԣ�����û��Ҫ�������ݾ�����
		CAN_bufferPop(&Que_CAN1_Tx,&CAN1_ReallySend);

			HAL_CAN_AddTxMessage(&hcan1,&CAN1_ReallySend.tx_header,CAN1_ReallySend.msg.data,(uint32_t*)CAN_TX_MAILBOX0);//��0�������ˣ����Զ��ӵ���һ��


		if(i == 1)
		{
			i = 2000;
		}
		
		
	TxMailFreeNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

}






/**
* @brief ����ʵ�ֵĹ��ܣ�ÿms��������
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
			
			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//��ʱ��������̬

	

		
		
		
		
  }
  /* USER CODE END Can_Send_Task */
}
