#include "pid.h"
#include "math.h"

/**
  * @brief  PID初始化函数
  * @param  pid: PID结构体指针
  * @param  kp: 比例系数
  * @param  ki: 积分系数
  * @param  kd: 微分系数
  * @param  min_out: 输出最小值
  * @param  max_out: 输出最大值
  * @param  dt: 采样时间
  * @retval 无
  */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float min_out, float max_out, float dt)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->setpoint = 0.0f;
    pid->input = 0.0f;
    pid->output = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->min_output = min_out;
    pid->max_output = max_out;
    pid->dt = dt;
}

/**
  * @brief  PID计算函数
  * @param  pid: PID结构体指针
  * @param  input: 输入值
  * @retval PID计算输出值
  */
float PID_Calculate(PID_TypeDef *pid, float input)
{
    float error, p_term, i_term, d_term;
    
    // 计算误差
    error = pid->setpoint - input;
    
    // 比例项
    p_term = pid->Kp * error;
    
    // 积分项
    pid->integral += error * pid->dt;
    i_term = pid->Ki * pid->integral;
    
    // 微分项
    pid->derivative = (error - pid->last_error) / pid->dt;
    d_term = pid->Kd * pid->derivative;
    
    // 计算输出
    pid->output = p_term + i_term + d_term;
    
    // 限制输出范围
    if (pid->output > pid->max_output)
        pid->output = pid->max_output;
    else if (pid->output < pid->min_output)
        pid->output = pid->min_output;
    
    // 保存当前误差
    pid->last_error = error;
    
    return pid->output;
}

/**
  * @brief  PID重置函数
  * @param  pid: PID结构体指针
  * @retval 无
  */
void PID_Reset(PID_TypeDef *pid)
{
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
}