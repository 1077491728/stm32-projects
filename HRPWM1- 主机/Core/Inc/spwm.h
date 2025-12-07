#ifndef __SPWM_H
#define __SPWM_H
#define SINTABLE_LENGTH 500   
void InitTimerVariable(void);
extern float Modulation;
extern volatile uint16_t DutyPointer;
extern float Sin_Duty[SINTABLE_LENGTH];
extern float SinTable[SINTABLE_LENGTH];
#endif
