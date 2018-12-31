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

			
			ControlTask();//下面记得改5
			
			vTaskDelayUntil(&xLastWakeTime,5/portTICK_RATE_MS);//此时处于阻塞态

	

		
		
		
		
  }
  /* USER CODE END Can_Send_Task */
}
