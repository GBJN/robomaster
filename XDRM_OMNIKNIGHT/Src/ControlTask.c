#include "ControlTask.h"

#include "Driver_Chassis.h"
#include "Driver_Beltraise.h"


void ControlTask(void)
{
	Chassis_Control();
	Belt_Control();
}

void Drivers_Control_Task(void const * argument)
{

  
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {

			
			ControlTask();//����ǵø�5
			
			vTaskDelayUntil(&xLastWakeTime,5/portTICK_RATE_MS);//��ʱ��������̬

	

		
		
		
		
  }
  /* USER CODE END Can_Send_Task */
}
