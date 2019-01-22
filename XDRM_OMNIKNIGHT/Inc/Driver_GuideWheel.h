#ifndef __GUIDEWHEEL_H
#define __GUIDEWHEEL_H

#include "config.h"

#define MotorStart()									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6|GPIO_PIN_12,GPIO_PIN_SET);
#define MotorStop()									  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6|GPIO_PIN_12,GPIO_PIN_RESET);


void MotorInit(void);
void MotorSpeedSet(void);
void GuideWheel_Control(void);
	

#endif

