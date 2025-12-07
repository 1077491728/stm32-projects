#ifndef __KEY_H
#define __KEY_H

void Key_Init(void);
uint16_t Key_GetNum(void);
void Key_Tick(void);
extern volatile uint32_t Timer_1ms;

#endif
