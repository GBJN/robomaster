#include "ControlTask.h"

#include "Driver_Chassis.h"
#include "Driver_Beltraise.h"
#include "Driver_GuideWheel.h"
#include "Driver_Sensor.h"
#include "Driver_Manipulator.h"

#include "CanBusTask.h"
void ControlTask(void)
{
	static uint32_t tick = 0;
	tick++;

	InfraredSensor_StateGet();
	if(tick%4==0)
	{
		Chassis_Control();
		
	}
	Manipulator_Control();
	GuideWheel_Control();
	Belt_Control();
	
}



void Drivers_Control_Task(void const * argument)
{

  
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {

			
			ControlTask();
			
			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//此时处于阻塞态

	

		
		
		
		
  }
  /* USER CODE END Can_Send_Task */
}
