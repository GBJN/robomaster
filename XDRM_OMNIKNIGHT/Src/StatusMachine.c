#include "StatusMachine.h"






void StatusMachine_Update(void)
{

	
	
	
	
	
	


}
	

void StatusMachine(void const * argument)
{

  
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {

			
			StatusMachine_Update();
			
			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//��ʱ��������̬
		
		
  }
  /* USER CODE END Can_Send_Task */
}





