#ifndef __PFC_H
#define __PFC_H
#include "stm32g4xx_hal.h"
#include "arm_math.h"

#define FSW 50e3

#define ssrf_ts 1 / FSW
#define ssrf_kp 1.007f
#define sser_ki 0.006f
#define ssrf_up_limt 63
#define ssrf_low_limt 43
#define value_2pi 2.0f * 3.1415926f

//[0.003405429629029,0,-0.003405429629029]
//[0.010982881827120,0,-0.010982881827120]
#define SIGO_U_GAIN 1
#define SIGO_U_B0 0.010982881827120
#define SIGO_U_B1 0
#define SIGO_U_B2 -0.010982881827120

//[1,-1.993165860418354,0.993189140741942]
//[1,-1.977790221205283,0.978034236345759]
#define SIGO_U_A1 -1.977790221205283
#define SIGO_U_A2 0.978034236345759

// qu coeff [8.229594388365577e-06,1.645918877673115e-05,8.229594388365577e-06]
//[8.625935215831610e-05,1.725187043166322e-04,8.625935215831610e-05]
#define SIGO_QU_GAIN 1.0e-03
#define SIGO_QU_B0 0.08625935215831610
#define SIGO_QU_B1 0.1725187043166322
#define SIGO_QU_B2 0.08625935215831610

// q coef  [1,-1.993165860418354,0.993189140741942]
//[1,-1.977790221205283,0.978034236345759]
#define SIGO_QU_A1 -1.977790221205283
#define SIGO_QU_A2 0.978034236345759
//
// Const data define
//
typedef struct DIS_2ORDER_TF_COEF_TAG
{
    float32_t gain;
    float32_t B0;
    float32_t B1;
    float32_t B2;
    float32_t A1;
    float32_t A2;
} DIS_2ORDER_TF_COEF_DEF;

typedef struct DIS_2ORDER_TF_DATA_TAG
{
    float32_t output;
    float32_t w0;
    float32_t w1;
    float32_t w2;
} DIS_2ORDER_TF_DATA_DEF;

//
// SOGI PLL date struct
//
typedef struct SOGI_PLL_DATA_STRUCT_TAG
{
    float32_t ac_u;
    float32_t ac_qu;
    float32_t u_q;
    float32_t u_d;
    float32_t theta;
    float32_t theta_1;
    float32_t cos_theta;
    float32_t sin_theta;
    float32_t grid_freq;
    float32_t pll_freq_out;
    float32_t delta_t;

    DIS_2ORDER_TF_COEF_DEF sogi_u_coeff;
    DIS_2ORDER_TF_DATA_DEF sogi_u_data;

    DIS_2ORDER_TF_COEF_DEF sogi_qu_coeff;
    DIS_2ORDER_TF_DATA_DEF sogi_qu_data;

    float32_t spll_kp;
    float32_t spll_ki;
    float32_t spll_integrator;
    float32_t spll_freq_min_limt;
    float32_t spp_freq_max_limt;
} SOGI_PLL_DATA_DEF;

void sogi_pll_init(SOGI_PLL_DATA_DEF *spll_obj, float32_t grid_freq, float32_t ts);
float32_t discrete_2order_tf(const float32_t input, DIS_2ORDER_TF_COEF_DEF *coeff, DIS_2ORDER_TF_DATA_DEF *data);
void spll_sogi_func(SOGI_PLL_DATA_DEF *spll_obj, float32_t grid_volt_sen);
//void TIM1_UP_TIM10_IRQHandler(); // TIM1ÖÐ¶Ï
#endif
