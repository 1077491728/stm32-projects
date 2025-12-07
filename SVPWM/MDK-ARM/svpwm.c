#include "stm32g4xx_hal.h"
#include "measure.h"
#include "main.h"
#include "OLED.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// 全局变量定义
volatile uint16_t compare[6];  // 修正：移除指针数组定义，改为实际数组

/**
 * @brief  三电平SVPWM算法实现
 * @param  SinA, SinB, SinC: 三相参考电压
 * @param  Period: PWM周期
 * @param  Compare: 比较寄存器数组指针
 * @retval None
 */
void ThreeLevel_SVPWM(float SinA, float SinB, float SinC, uint16_t Period, uint16_t* Compare)
{
    // Clark变换，将三相静止坐标系转换到两相静止坐标系
    float Valpha = SinA;  // alpha轴参考电压
    float Vbeta = (SinA + 2*SinB) / sqrt_3;  // beta轴参考电压 (sqrt_3已在main.h中定义)
    
    // 计算参考电压矢量幅值
    float VrefMag = sqrtf(Valpha * Valpha + Vbeta * Vbeta);
    
    // 计算调制比
    float m = sqrt_3 * VrefMag / 1.0f;  // 假设直流母线电压为1.0，根据实际需要调整
    
    // 过调制处理 - 确保调制比不超过理论最大值
    if (m > 1.0f) {
        float scale = 1.0f / m;
        Valpha *= scale;
        Vbeta *= scale;
        m = 1.0f;
    }
    
    // 扇区判断
    float Ualfa1 = Vbeta;
    float Ualfa2 = (sqrt_3 * Valpha - Vbeta) / 2.0f;
    float Ualfa3 = (-sqrt_3 * Valpha - Vbeta) / 2.0f;
    
    uint8_t Sector = 0;
    if (Ualfa1 > 0) Sector++;
    if (Ualfa2 > 0) Sector += 2;
    if (Ualfa3 > 0) Sector += 4;
    
    // 基本矢量作用时间计算
    float T1, T2;
    switch(Sector) {
        case 1:
            T1 = Ualfa2;
            T2 = Ualfa3;
            break;
        case 2:
            T1 = Ualfa3;
            T2 = -Ualfa1;
            break;
        case 3:
            T1 = -Ualfa2;
            T2 = Ualfa1;
            break;
        case 4:
            T1 = -Ualfa3;
            T2 = -Ualfa2;
            break;
        case 5:
            T1 = -Ualfa1;
            T2 = -Ualfa3;
            break;
        case 6:
            T1 = Ualfa1;
            T2 = Ualfa2;
            break;
        default:
            T1 = 0;
            T2 = 0;
            break;
    }

    // 时间归一化
    T1 *= Period;
    T2 *= Period;
    
    // 确保作用时间不超过PWM周期
    if ((T1 + T2) > Period) {
        float scale = (float)Period / (T1 + T2);
        T1 *= scale;
        T2 *= scale;
    }

    float T0 = (Period - T1 - T2) / 2.0f;  // 零矢量作用时间
    
    // 根据扇区计算各相比较值
    switch(Sector) {
        case 1:  // 扇区1: T1, T2, T0
            Compare[0] = (uint16_t)(T0 + T1 + T2);  // TA
            Compare[1] = (uint16_t)(T0 + T2);       // TB
            Compare[2] = (uint16_t)(T0);            // TC
            break;
        case 2:  // 扇区2: T2, T1, T0
            Compare[0] = (uint16_t)(T0 + T1);       // TA
            Compare[1] = (uint16_t)(T0 + T1 + T2);  // TB
            Compare[2] = (uint16_t)(T0);            // TC
            break;
        case 3:  // 扇区3: T0, T1, T2
            Compare[0] = (uint16_t)(T0 + T1);       // TA
            Compare[1] = (uint16_t)(T0 + T1 + T2);  // TB
            Compare[2] = (uint16_t)(T0 + T1 + T2);  // TC
            break;
        case 4:  // 扇区4: T0, T2, T1
            Compare[0] = (uint16_t)(T0);            // TA
            Compare[1] = (uint16_t)(T0 + T2);       // TB
            Compare[2] = (uint16_t)(T0 + T1 + T2);  // TC
            break;
        case 5:  // 扇区5: T2, T0, T1
            Compare[0] = (uint16_t)(T0);            // TA
            Compare[1] = (uint16_t)(T0 + T1 + T2);  // TB
            Compare[2] = (uint16_t)(T0 + T1);       // TC
            break;
        case 6:  // 扇区6: T1, T0, T2
            Compare[0] = (uint16_t)(T0 + T2);       // TA
            Compare[1] = (uint16_t)(T0);            // TB
            Compare[2] = (uint16_t)(T0 + T1 + T2);  // TC
            break;
        default:
            Compare[0] = Compare[1] = Compare[2] = Period / 2;  // 默认占空比50%
            break;
    }
    
    // 限制比较值在有效范围内
    for (int i = 0; i < 3; i++) {
        if (Compare[i] > Period) {
            Compare[i] = Period;
        }
    }
}