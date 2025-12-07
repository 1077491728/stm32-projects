#ifndef __SVPWM_H
#define __SVPWM_H
#include "main.h"

// 全局变量声明
extern volatile uint16_t compare[6];

// 函数声明
void ThreeLevel_SVPWM(float SinA, float SinB, float SinC, uint16_t Period, uint16_t* Compare);

#endif /* __SVPWM_H */