#ifndef __STATUSMACHINE_H
#define __STATUSMACHINE_H

#include "Driver_Beltraise.h"
#include "Driver_Chassis.h"
#include "Driver_GuideWheel.h"
#include "Driver_Manipulator.h"
#include "Driver_Remote.h"
#include "Driver_Sensor.h"

#include "config.h"

typedef enum
{
    Stop_Move_Mode,
    Normal_Move_Mode,
    Auto_Up_Island_Mode,
    Auto_Down_Island_Mode,
    Auto_Get_Box_Mode
}CarMoveModeTypeDef;

typedef enum
{
    Up_Island_Stop,
    Up_Island_PrePare,
    Up_Island_Up_First,
    Up_Island_Advance_First,
    Up_Island_Up_Twice,
    Up_Island_Advance_Twice
}UpIslandStateTypeDef;

void StatusMachine_Init(void);
void StatusMachine(void const * argument);
extern InputMode_e	InputMode;
extern GuideWheelModeTypeDef    GuideWheelMode;
extern BeltModeTypeDef          BeltMode;
extern ChassisModeTypeDef       ChassisMode;
extern UpIslandStateTypeDef UpIslandState;

#endif

