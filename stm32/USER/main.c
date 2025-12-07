

///////////////////////////////////////////////////////////
/*头文件包含区*/
#include "usart.h"
#include "delay.h"
#include "string.h"
#include "led.h"

#include "stm32f10x.h"                  // Device header

#include "OLED.h"
#include "bep.h"
#include "hc.h"
#include "key.h"
///////////////////////////////////////////////////////////
/*宏定义区*/

///////////////////////////////////////////////////////////
/*外部变量声明区*/
//下面变量声明可以从HuaweiIOT_at_esp8266.c文件的【步骤3】提示中复制
uint8_t uart1_rec_i=0;
extern uint8_t atok_rec_flag;
char uart1_recdata=0,uart1_recstring[256],sub_string[150],analysis_Str[256];
/*函数声明区*/

///////////////////////////////////////////////////////////

/*
*********************************************************************************************************
* 函 数 名: main
* 功能说明: 主函数
* 形 参：无
* 返 回 值: 无
*********************************************************************************************************
*/
int main(void)
{	
	float length;
	uint8_t i=0;
	uint8_t red=10;
	uint8_t yellow=5;
	uint8_t green=8;
	LED_Init();	
	OLED_Init();
	beep_Init();
	Hcsr04Init();  //超声波初始化
	delay_init();	     //delay初始化
	
	while(1){
	
	  length = Hcsr04GetLength();   //获取超声波传感器测得值
	
	  OLED_ShowString(3,73, ">>");
		OLED_ShowCHinese(5,80,11);//子
		OLED_ShowCHinese(5,95,12);//子
	
	  OLED_ShowCHinese(0,0,4);
		OLED_ShowCHinese(0,15,5);
		OLED_ShowCHinese(0,30,6);
		OLED_ShowCHinese(0,45,7);
		OLED_ShowCHinese(0,60,8);
		OLED_ShowCHinese(0,75,9);
		OLED_ShowCHinese(0,90,10);
		
	
	
	  OLED_ShowCHinese(2,0,0);
		OLED_ShowCHinese(2,20,1);
		OLED_ShowCHinese(2,40,2);
		OLED_ShowCHinese(2,60,3);
	  OLED_ShowNum(2, 75, length, 2);
	  OLED_ShowString(2,77, "cm");
		
		
		if (length < 5)
		{
			
			while(1){
				OLED_Clear();	
		length = Hcsr04GetLength();   //获取超声波传感器测得值
		OLED_ShowString(3,73, ">>");
		OLED_ShowCHinese(5,80,11);//子
		OLED_ShowCHinese(5,95,12);//子
			
		OLED_ShowCHinese(0,0,13);
		OLED_ShowCHinese(0,15,14);
		OLED_ShowCHinese(0,30,15);
		OLED_ShowCHinese(0,45,16);
		OLED_ShowCHinese(0,60,17);
			
		OLED_ShowCHinese(2,0,18);
		OLED_ShowCHinese(2,20,19);
		OLED_ShowCHinese(2,40,20);
		OLED_ShowCHinese(2,60,21);
		OLED_ShowNum(2, 75, length, 2);
		OLED_ShowString(2,77, "cm");
			
		LED1=0;
		bep=0;
		
		delay_ms(500);
		delay_ms(500);
			
		if (length>5)
			
		
		
		{
			LED1=1;
		bep=1;
			break;
		
		}
		
	}}}
		
		
		
		
	
	
}

