#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// PID结构体定义
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float setpoint;     // 设定值
    float current;      // 当前值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分项累计值
    float output;       // PID输出
    float max_output;   // 最大输出限制
    float min_output;   // 最小输出限制
    float max_integral; // 积分项上限
    float dt;           // 采样时间
} PID_HandleTypeDef;

// 函数声明
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, float dt);
void PID_SetPoint(PID_HandleTypeDef *pid, float setpoint);
float PID_Calculate(PID_HandleTypeDef *pid, float current);
void PID_Reset(PID_HandleTypeDef *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */