#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float input;
    float output;
    float last_error;
    float integral;
    float derivative;
    float min_output;
    float max_output;
    float dt;  // 采样时间
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float min_out, float max_out, float dt);
float PID_Calculate(PID_TypeDef *pid, float input);
void PID_Reset(PID_TypeDef *pid);

#endif