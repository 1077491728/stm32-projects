#include"stdio.h"
#include"main.h"
#include"adcget.h"
#include "stm32g4xx.h"
#include "adc.h"

void getadc_value()
{for (int i = 0; i < 2; i++)
{HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 50);
    adc[i]= HAL_ADC_GetValue(&hadc1);
        // Í£Ö¹ADC×ª»»
    /* code */

}
HAL_ADC_Start(&hadc2);
HAL_ADC_PollForConversion(&hadc2, 50);
adci2= HAL_ADC_GetValue(&hadc2);

adcu=adc[0];
adci1=adc[1];
}
