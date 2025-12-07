#ifndef __PLL_H
#define __PLL_H

#include "math.h"
#include "arm_math.h"

#define fz 0.00005f
#define w_grid 314.15926f


typedef float float32;

typedef struct{
	float32	osg_k;
	float32	osg_x;
	float32	osg_y;
	float32	osg_b0;
	float32	osg_b2;
	float32	osg_a1;
	float32	osg_a2;
	float32	osg_qb0;
	float32	osg_qb1;
	float32	osg_qb2;
}SPLL_1ph_SOGI_F_OSG_COEFF;

typedef struct{
	float32	B1_lf;
	float32	B0_lf;
	float32	A1_lf;
}SPLL_1ph_SOGI_F_LPF_COEFF;

typedef struct{
	float32	u[3];  // Ac Input
	float32   osg_u[3];
	float32   osg_qu[3];
	float32   u_Q[2];
	float32   u_D[2];
	float32   ylf[2];
	float32   fo; // output frequency of PLL
	float32   fn; //nominal frequency
	float32	theta[2];
	float32	cos;
	float32	sin;
	float32   delta_T;
	SPLL_1ph_SOGI_F_OSG_COEFF osg_coeff;
	SPLL_1ph_SOGI_F_LPF_COEFF lpf_coeff;
}SPLL_1ph_SOGI_F;

void SOGI_PLL_Start(void);
void SPLL_1ph_SOGI_F_FUNC(SPLL_1ph_SOGI_F *spll_obj);


#endif
