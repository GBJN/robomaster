#ifndef __Driver_BELTRAISE_H
#define __Driver_BELTRAISE_H


#include "config.h"



#define THRESHOLD   9000





void Belt_Control(void);
void BM_Get_SpeedRef(void);
void BM_Set_Current(void);
void BM_PID_Task(void);
void BM_Get_PID(void);

#endif

