#include "CNTL_PI_F.h"

//*********** Structure Init Function ****//
void CNTL_PI_F_init(CNTL_PI_F *k)
{
	/* Initialize variables */
	k->Ref = 	0.0f;
	k->Fbk = 	0.0f;
	k->Out = 	0.0f;
	k->Kp = 	0.0f;
	k->Ki =	 	0.0f;
	k->Umax = 	(1.0f);
	k->Umin = 	(0.0f);
	k->up = 	0.0f;
	k->ui = 	0.0f;
	k->v1 = 	0.0f;
	k->i1 = 	0.0f;
	k->w1 = 	0.0f;
}

//*********** Function Definition ********//
void CNTL_PI_F_FUNC(CNTL_PI_F *v)
{
	/* proportional term */
	v->up = ((v->Ref - v->Fbk)*v->Kp);

	/* integral term */
	v->ui = (v->Out == v->v1)?((v->Ki* v->up)+ v->i1) : v->i1;
	v->i1 = v->ui;

	/* control output */
	v->v1 = (v->up + v->ui);
	v->Out = (v->v1 > v->Umax)?(v->Umax) : v->v1;
	v->Out = (v->Out < v->Umin)?(v->Umin) : v->Out;
}

