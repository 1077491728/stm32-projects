#include "stm32g4xx_hal.h"
#include "main.h"
#include "pid.h"
#include "math.h"
#include "stdio.h"
/*定义变量*/
float Target_U, Actual_U,Target_I, Actual_I, Out=0;			//目标值，实际值，输出值
float Kp_U, Ki_U, Kd_U;					//比例项，积分项，微分项的权重
float Error0, Error1, Error2, ErrorInt;		//本次误差，上次误差，误差积分
volatile uint32_t PID_1ms=0;
float PID_Control_U(void)
{           
	        Target_U=13.0f;
	        Kp_U=3.0, Ki_U=0.1, Kd_U=0;
			/*获取本次误差、上次误差和上上次误差*/
			Error2 = Error1;			//获取上上次误差
			Error1 = Error0;			//获取上次误差
			Error0 = Target_U - Actual_U;	//获取本次误差，目标值减实际值，即为误差值
			

			/*定义一个系数C，表示积分的速度，C的值与误差绝对值大小呈反比，误差绝对值越大，积分速度越慢*/
			float C = 1 / (0.2 * fabs(Error0) + 1);		//根据公式计算得到系数C
			
			/*误差积分*/
			ErrorInt = C * Error0;		//积分的速度由C确定，C的取值范围是0~1
			
			/*注意：变速积分本身没有防止积分饱和的效果，为了避免积分饱和，此处可以再加入积分限幅的逻辑*/
			
			if (ErrorInt > 5) {ErrorInt = 5;}		//限制ErrorInt输出值最大为100
			if (ErrorInt < -5) {ErrorInt = -5;}	//限制ErrorInt输出值最小为100
			/*PID计算*/
			/*使用增量式PID公式，计算得到输出值*/
			Out += Kp_U * (Error0 - Error1) + Ki_U * ErrorInt+ Kd_U * (Error0 - 2 * Error1 + Error2);
			
			/*输出限幅*/
			if (Out > 20) {Out = 20;}		//限制Out输出值最大为40
			if (Out < -20) {Out = -20;}	//限制Out输出值最小为-40
			return Out;
}




