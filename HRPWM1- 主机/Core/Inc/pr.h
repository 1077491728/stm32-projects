#ifndef __PR_H
#define __PR_H
#include <stdint.h>  // 如果使用了uint32_t等类型
#include <stdbool.h> // 如果使用了bool类型

#define PRTABLE_LENGTH 500 
extern float PRTable[PRTABLE_LENGTH];
typedef struct {//pr控制

//参数

float Kp; // 比例系数

float Kr; // 谐振系数

float w0;

float wc;

float Ts;

float b0_prime;
float b2_prime;
float a1_prime;
float a2_prime;//离散化系数

//状态变量

float e_prev1; // e[k-1]
float e_prev2; // e[k-2]
float yr_prev1; // yr[k-1]
float yr_prev2; // yr[k-2]
}QPRController;
float PID_Control_U(void);
void QPR_Init(QPRController *ctrl,float w0, float wc,float Ts,float Kr,float Kp);
float QPR_Update(QPRController *ctrl, float Reference, float FeedBack);
#endif
