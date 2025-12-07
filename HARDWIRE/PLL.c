#include "PLL.h"
#include <math.h>


#include <math.h>
#include "arm_math.h"


//*********** Structure Init Function ****//
void SPLL_1ph_SOGI_F_init(int Grid_freq, float32 DELTA_T, SPLL_1ph_SOGI_F *spll_obj)
{
	spll_obj->u[0]=(float32)(0.0f);
	spll_obj->u[1]=(float32)(0.0f);
	spll_obj->u[2]=(float32)(0.0f);
	
	spll_obj->osg_u[0]=(float32)(0.0f);
	spll_obj->osg_u[1]=(float32)(0.0f);
	spll_obj->osg_u[2]=(float32)(0.0f);
	
	spll_obj->osg_qu[0]=(float32)(0.0f);
	spll_obj->osg_qu[1]=(float32)(0.0f);
	spll_obj->osg_qu[2]=(float32)(0.0f);
	
	spll_obj->u_Q[0]=(float32)(0.0f);
	spll_obj->u_Q[1]=(float32)(0.0f);
								  
	spll_obj->u_D[0]=(float32)(0.0f);
	spll_obj->u_D[1]=(float32)(0.0f);
								  
	spll_obj->ylf[0]=(float32)(0.0f);
	spll_obj->ylf[1]=(float32)(0.0f);
	
	spll_obj->fo=(float32)(0.0f);
	spll_obj->fn=(float32)(Grid_freq);
	
	spll_obj->theta[0]=(float32)(0.0f);
	spll_obj->theta[1]=(float32)(0.0f);
	
	spll_obj->sin=(float32)(0.0f);
	spll_obj->cos=(float32)(0.0f);

	// loop filter coefficients for 20kHz
	spll_obj->lpf_coeff.B0_lf=(float32)(166.9743f);
	spll_obj->lpf_coeff.B1_lf=(float32)(-166.266f);
	spll_obj->lpf_coeff.A1_lf=(float32)(-1.0f);
	
	spll_obj->delta_T=DELTA_T;
}

//*********** Structure Coeff Update *****//
void SPLL_1ph_SOGI_F_coeff_update(float32 delta_T, float32 wn, SPLL_1ph_SOGI_F *spll)
{
	float32 osgx,osgy,temp;
	spll->osg_coeff.osg_k=(float32)(0.5f);
	osgx=(float32)(2.0f*0.5f*wn*delta_T);
	spll->osg_coeff.osg_x=(float32)(osgx);
	osgy=(float32)(wn*delta_T*wn*delta_T);
	spll->osg_coeff.osg_y=(float32)(osgy);
	temp=(float32)1.0f/(osgx+osgy+4.0f);
	spll->osg_coeff.osg_b0=((float32)osgx*temp);
	spll->osg_coeff.osg_b2=((float32)(-1.0f)*spll->osg_coeff.osg_b0);
	spll->osg_coeff.osg_a1=((float32)(2.0f*(4.0f-osgy))*temp);
	spll->osg_coeff.osg_a2=((float32)(osgx-osgy-4.0f)*temp);
	spll->osg_coeff.osg_qb0=((float32)(0.5f*osgy)*temp);
	spll->osg_coeff.osg_qb1=(spll->osg_coeff.osg_qb0*(float32)(2.0f));
	spll->osg_coeff.osg_qb2=spll->osg_coeff.osg_qb0;
}

//*********** Function Definition ********//
void SPLL_1ph_SOGI_F_FUNC(SPLL_1ph_SOGI_F *spll_obj)
{
	// Update the spll_obj->u[0] with the grid value before calling this routine

	//-------------------------------//
	// Orthogonal Signal Generator 	 //
	//-------------------------------//
	spll_obj->osg_u[0]=(spll_obj->osg_coeff.osg_b0*(spll_obj->u[0]-spll_obj->u[2])) + (spll_obj->osg_coeff.osg_a1*spll_obj->osg_u[1]) + (spll_obj->osg_coeff.osg_a2*spll_obj->osg_u[2]);

	spll_obj->osg_u[2]=spll_obj->osg_u[1];
	spll_obj->osg_u[1]=spll_obj->osg_u[0];

	spll_obj->osg_qu[0]=(spll_obj->osg_coeff.osg_qb0*spll_obj->u[0]) + (spll_obj->osg_coeff.osg_qb1*spll_obj->u[1]) + (spll_obj->osg_coeff.osg_qb2*spll_obj->u[2]) + (spll_obj->osg_coeff.osg_a1*spll_obj->osg_qu[1]) + (spll_obj->osg_coeff.osg_a2*spll_obj->osg_qu[2]);

	spll_obj->osg_qu[2]=spll_obj->osg_qu[1];
	spll_obj->osg_qu[1]=spll_obj->osg_qu[0];

	spll_obj->u[2]=spll_obj->u[1];
	spll_obj->u[1]=spll_obj->u[0];
	
	//-------------------------------------------------------//
	// Park Transform from alpha beta to d-q axis 			 //
	//-------------------------------------------------------//
	spll_obj->u_Q[0]= (spll_obj->cos*spll_obj->osg_u[0]) + (spll_obj->sin*spll_obj->osg_qu[0]);
	spll_obj->u_D[0]= -(spll_obj->cos*spll_obj->osg_qu[0]) + (spll_obj->sin*spll_obj->osg_u[0]);

	//---------------------------------//
	// Loop Filter                     //
	//---------------------------------//
	spll_obj->ylf[0]=spll_obj->ylf[1] + (0.5f*spll_obj->u_Q[0]*(1.0f)) + (-0.5f*spll_obj->u_Q[1]*(1.0f));
	spll_obj->ylf[1]=spll_obj->ylf[0];

	spll_obj->u_Q[1]=spll_obj->u_Q[0];

	//---------------------------------//
	// VCO                             //
	//---------------------------------//
	spll_obj->fo=spll_obj->fn+spll_obj->ylf[0];
	
	spll_obj->theta[0]=spll_obj->theta[1] + (spll_obj->fo*spll_obj->delta_T)*(float32)(2.0f*3.1415926f);
	while(spll_obj->theta[0]>(float32)(2.0f*3.1415926f)||spll_obj->theta[0]<(float32)(0.0f))
    {
	    if(spll_obj->theta[0]>(float32)(2.0f*3.1415926f))
		    spll_obj->theta[0]-=(float32)(2.0f*3.1415926f);
        else if(spll_obj->theta[0]<(float32)(0.0f))
		    spll_obj->theta[0]+=(float32)(2.0f*3.1415926f);
	}
	
	spll_obj->theta[1]=spll_obj->theta[0];
	//dsp
	spll_obj->cos=(float32)arm_cos_f32(spll_obj->theta[0]);
    spll_obj->sin= (float32)arm_sin_f32(spll_obj->theta[0]);
}




SPLL_1ph_SOGI_F spll1;
void SOGI_PLL_Start(void)
{
/* Start_BEGIN */
	SPLL_1ph_SOGI_F_init(50.0f,fz,&spll1);
	SPLL_1ph_SOGI_F_coeff_update(fz,w_grid,&spll1);
/* Start_END */
}


//{
//spll1.u[0]=(float32)(u0[0]); 
//// SPLL call 
//SPLL_1ph_SOGI_F_FUNC(&spll1);
//y0[0] = spll1.u[0];
//y0[1] = spll1.osg_qu[0];
//y1[0] = spll1.u_D[0];
//y1[1] = spll1.u_Q[0];
//y2[0] = spll1.theta[0];
//y3[0] = spll1.sin;

//}

