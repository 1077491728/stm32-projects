#ifndef __TOTEM_PFC_H__
#define __TOTEM_PFC_H__

#include "CNTL_PI_F.h"
#include "main.h"

#define VLOOP_KP            0.06f
#define VLOOP_KI            0.025f  //增加可以减小过冲0.5-1.0，减小可以减少低频纹波

//最大交流电流限幅（最大值）  = 额定功率/交流电压*1.414
#define Vloop_UP            4.5f
#define Vloop_LOW          -4.5f

#define  Iloop_K             0.08f   

typedef struct AC_DC_CTRL_TAG {
    float  vbus;             // 母线电压实时值
    float  vbus_ref;         // 母线电压参考
    float  iac;              // 电网输入电流
    float  vac;              // 电网电压实时值
    float  vbus_f;
    float  sintheta;         // 电网单位正弦电压实时值 （sin(wt)）
	 float  ctrl_vol;         //
} AC_DC_CTRL_DEF;




float Totem_PFC_ctrl(AC_DC_CTRL_DEF* ctrlpar);


	

#endif 
