#ifndef __MEASURE_H
#define __MEASURE_H

#define SAMPLE_LENGTH 1000
extern float  ADC_Value_U[SAMPLE_LENGTH];
extern float  ADC_Value_I[SAMPLE_LENGTH];
float measure_rms_u(void);
float measure_rms_i(void);



#endif
