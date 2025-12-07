#include "pid.h"
#include "main.h"

/**
 * @brief  初始化PID控制器
 * @param  pid: PID控制器句柄
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @param  dt: 采样时间
 * @retval 无
 */
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    
    pid->setpoint = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    
    pid->max_output = 1000.0f;   // 默认最大输出
    pid->min_output = -1000.0f;  // 默认最小输出
    pid->max_integral = 1000.0f; // 默认积分上限
}

/**
 * @brief  设置PID目标值
 * @param  pid: PID控制器句柄
 * @param  setpoint: 目标值
 * @retval 无
 */
void PID_SetPoint(PID_HandleTypeDef *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/**
 * @brief  PID计算
 * @param  pid: PID控制器句柄
 * @param  current: 当前反馈值
 * @retval PID输出值
 */
float PID_Calculate(PID_HandleTypeDef *pid, float current)
{
    // 保存当前值
    pid->current = current;
    
    // 计算误差
    pid->error = pid->setpoint - current;
    
    // 比例项
    float proportional = pid->kp * pid->error;
    
    // 积分项
    pid->integral += pid->ki * pid->error * pid->dt;
    
    // 积分限幅
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    
    // 微分项
    float derivative = pid->kd * (pid->error - pid->last_error) / pid->dt;
    
    // 计算输出
    pid->output = proportional + pid->integral + derivative;
    
    // 输出限幅
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < pid->min_output) {
        pid->output = pid->min_output;
    }
    
    // 保存当前误差作为下次的上次误差
    pid->last_error = pid->error;
    
    return pid->output;
}

/**
 * @brief  重置PID控制器
 * @param  pid: PID控制器句柄
 * @retval 无
 */
void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}