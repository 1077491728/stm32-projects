#include "stm32g4xx_hal.h"
#include "key.h"
#include "tim.h"
#include "gpio.h"

volatile uint32_t Timer_1ms=0;
uint16_t Key_Num;


void Key_Tick(void);

uint16_t Key_GetState(void)
{

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)==0)
        {
        return 1;
        }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)==0)
        {
        return 2;
        }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)==0)
        {
        return 3;
        }
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)==0)
        {
        return 4;
        }
	if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2)==0)
        {
        return 5;
        }
        return 0;
}
uint16_t Key_GetNum(void)
{
    uint16_t Temp;
    if (Key_Num!=0)
    {
        Temp = Key_Num;
        Key_Num = 0;
        return Temp;
    }
    return 0;
}
void Key_Tick(void)
{
    static uint16_t Count;
    static uint16_t CurrState, PrevState;
    Count++;
    if (Count >= 20)
    {
		
        Count = 0;
        PrevState = CurrState;
        CurrState = Key_GetState();
        if (CurrState == 0 && PrevState != 0)
        {
                Key_Num = PrevState;
			
        }
    }
}	
