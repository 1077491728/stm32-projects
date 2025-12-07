#include "stm32g4xx_hal.h"
#include "measure.h"
#include "adc.h"
#include "main.h"
#include "math.h"



#define SAMPLE_LENGTH 1000
float ADC_Value_U[SAMPLE_LENGTH]={0};
float ADC_Value_I[SAMPLE_LENGTH]={0};
float measure_rms_u(void) //均方根算法
{
  float sum=0,result;
  int j;
  for(j=0;j<SAMPLE_LENGTH;j++)
	{
		sum+=(ADC_Value_U[j]*ADC_Value_U[j]*1.0f);
	}
	sum=sum/(SAMPLE_LENGTH*1.0f);
	//result=0.101525911f*sqrt(sum)-154.562627f;                            
	result=sqrt(sum); 
	return result;
	
}
float measure_rms_i(void) //均方根算法
{
  float sum=0,result;
  int j;
  for(j=0;j<SAMPLE_LENGTH;j++)
	{
		sum+=(ADC_Value_I[j]*ADC_Value_I[j]*1.0f);
	}
	sum=sum/(SAMPLE_LENGTH*1.0f);
	//result=0.016671301f*sqrt(sum)-25.80713844f;                           
    result=sqrt(sum);	
	return result;
	
}
