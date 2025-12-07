/* Includes_BEGIN */
#include "Totem_PFC.h"
#include <math.h>
/* Includes_END */

AC_DC_CTRL_DEF g_acdc_ctrlpar;
CNTL_PI_F Vloop_pi = {
	0,						//Ref;   			// Input: reference set-point
	0,                      //Fbk;   			// Input: feedback
	0,                      //Out;   			// Output: controller output
	VLOOP_KP,               //Kp;				// Parameter: proportional loop gain
	VLOOP_KI,               //Ki;			    // Parameter: integral gain
	Vloop_UP,               //Umax;				// Parameter: upper saturation limit
	Vloop_LOW,              //Umin;				// Parameter: lower saturation limit
	0,                      //up;				// Data: proportional term
	0,                      //ui;				// Data: integral term
	0,                      //v1;				// Data: pre-saturated controller output
	0,                      //i1;				// Data: integrator storage: ui(k-1)
	0,                      //w1;				// Data: saturation record: [u(k-1) - v(k-1)]
};


float Totem_PFC_ctrl(AC_DC_CTRL_DEF* ctrlpar)
{
	Vloop_pi.Fbk = ctrlpar->vbus;
	Vloop_pi.Ref = ctrlpar->vbus_ref;
	CNTL_PI_F_FUNC(&Vloop_pi);                                                                
	float I_set=Vloop_pi.Out;
	
	const float Iac_ref = ctrlpar->sintheta * I_set;//电流参考
	float Iac_err = Iac_ref - ctrlpar->iac;         //电流误差
	float Voz;
	float D;
		
	Voz = ctrlpar->vbus_ref;                               //直流母线电压  输出
	D =  ctrlpar->vac / Voz - Iac_err * Iloop_K;   //直接计算占空比
	
	D = (D >  0.95) ?  0.95 : D;	
	D = (D < -0.95) ? -0.95 : D;
	return D;	
}



//float vac       = VAC[0];
//float iac       = IAC[0];
//float sintheta  = SINTHETA[0];
//float Vo_sen    = VO_SEN[0];
//float Va    = VA[0];
//g_acdc_ctrlpar.vbus_f = Vo_sen;// 母线电压实时值

//g_acdc_ctrlpar.vbus = Vo_sen;          // 母线电压实时值
//g_acdc_ctrlpar.vbus_ref = 48.0f;      // 母线电压参考
//g_acdc_ctrlpar.iac = iac;              // 电网输入电流
//g_acdc_ctrlpar.vac = vac;              // 电网电压实时值
//g_acdc_ctrlpar.sintheta =sintheta;     // 电网单位正弦电压实时值 （sin(wt)）

//g_acdc_ctrlpar.ctrl_vol = acdc_bidirect_ctrl2(&g_acdc_ctrlpar,Va);


////D1[0] = (1.0f+g_acdc_ctrlpar.ctrl_vol)*0.5f;
////D2[0] = (1.0f-g_acdc_ctrlpar.ctrl_vol)*0.5f;


////VBUS_REF[0] = g_acdc_ctrlpar.vbus_ref;//i_set1;


