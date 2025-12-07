#ifndef __PID_H
#define __PID_H
#include "stm32g4xx_hal.h"



extern volatile uint32_t PID_1ms;
extern float Actual_U,Actual_I,Out,ErrorInt;
#endif
