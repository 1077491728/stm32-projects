#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define LED0 PCout(13)	// PC13
	
#define LED1 PAout(0)	// PA0  ºìµÆ

#define LED2 PAout(1)	// PA1  »ÆµÆ
#define LED3 PAout(2)	// PA2  ÂÌµÆ


void LED_Init(void);//³õÊ¼»¯

		 				    
#endif
