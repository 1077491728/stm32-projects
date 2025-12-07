#include "main.h"
#include "pid.h"

// PID控制器结构体定义
PID_TypeDef voltage_pid;   // 电压外环PID
PID_TypeDef current_pid;   // 电流内环PID

// 目标值
#define VOLTAGE_TARGET 12.0f  // 目标电压12V
#define CURRENT_TARGET 12.0f  // 目标电流12A

// ADC变量
extern uint32_t adci1_aver;
extern uint32_t adci2_aver;
extern uint32_t adcu_aver;
extern uint32_t m;

// 控制输出变量
float voltage_output = 0.0f;
float current_output = 0.0f;

/**
  * @brief  初始化串级PID控制器
  * @param  无
  * @retval 无
  */
void CascadePID_Init(void)
{
    // 初始化电压外环PID控制器
    // 参数需要根据实际系统调整
    PID_Init(&voltage_pid, 0.5f, 0.1f, 0.01f, 0.0f, 100.0f, 0.001f);  // 1kHz采样频率
    voltage_pid.setpoint = VOLTAGE_TARGET;
    
    // 初始化电流内环PID控制器
    // 参数需要根据实际系统调整
    PID_Init(&current_pid, 1.0f, 0.2f, 0.05f, 0.0f, 100.0f, 0.001f);  // 1kHz采样频率
    current_pid.setpoint = CURRENT_TARGET;
}

/**
  * @brief  串级PID控制算法实现
  * @param  无
  * @retval 无
  */
void CascadePID_Control(void)
{
    float voltage_feedback, current_feedback;
    float voltage_pid_out, current_pid_out;
    
    // 获取反馈值（需要根据实际ADC转换关系进行转换）
    voltage_feedback = (float)adcu_aver * 3.3f / 4096.0f * 10.0f;  // 假设有10倍分压
    current_feedback = (float)adci1_aver * 3.3f / 4096.0f * 5.0f;   // 假设电流传感器比例
    
    // 电压外环PID计算
    voltage_pid_out = PID_Calculate(&voltage_pid, voltage_feedback);
    
    // 将电压环输出作为电流环设定值
    current_pid.setpoint = voltage_pid_out;
    
    // 电流内环PID计算
    current_pid_out = PID_Calculate(&current_pid, current_feedback);
    
    // 输出控制信号（需要根据实际硬件接口进行修改）
    // 例如：设置PWM占空比
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)current_pid_out);
}