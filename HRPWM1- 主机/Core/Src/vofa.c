///*
//	MIT License
//	Copyright (c) 2021 Jelin
//*/

//#include "vofa.h"
//#include "usart.h"
//#include <stdarg.h>
//#include <stdio.h>
//#include "stm32g4xx.h"                  // Device header
//extern uint8_t buffer[200];
//extern uint8_t len;
//static const uint8_t cmdTail[] = VOFA_CMD_TAIL;
//static const uint8_t justFloatTail[4] = {0x00, 0x00, 0x80, 0x7f};


//void Vofa_Init(Vofa_HandleTypedef* handle,Vofa_ModeTypeDef mode)
//{
//	handle->rxBuffer.rp = handle->rxBuffer.buffer;
//	handle->rxBuffer.wp = handle->rxBuffer.buffer;
//	handle->mode = mode;
//}

//void Vofa_SendData(Vofa_HandleTypedef* handle,uint8_t* data,uint16_t num)
//{
//	Vofa_SendDataCallBack(handle, data, num);
//}

//void Vofa_JustFloat(Vofa_HandleTypedef *handle, float *data, uint16_t num)
//{
//	Vofa_SendDataCallBack(handle, (uint8_t *)data, num * sizeof(float));
//	Vofa_SendDataCallBack(handle, (uint8_t *)justFloatTail, 4);
//}

//void Vofa_Printf(Vofa_HandleTypedef *handle, const char *format, ...)
//{
//	uint32_t n;
//	va_list args;
//	va_start(args, format);
//	n = vsnprintf((char*)handle->txBuffer, VOFA_BUFFER_SIZE, format, args);
//	Vofa_SendDataCallBack(handle, handle->txBuffer, n);
//	va_end(args);
//}

//void Vofa_ReceiveData(Vofa_HandleTypedef *handle)
//{
//	uint8_t data = Vofa_GetDataCallBack(handle);

//	if (handle->rxBuffer.overflow && handle->mode == VOFA_MODE_BLOCK_IF_FIFO_FULL)
//	{
//		return;
//	}

//	*handle->rxBuffer.wp = data;
//	handle->rxBuffer.wp++;

//	if (handle->rxBuffer.wp == (handle->rxBuffer.buffer + VOFA_BUFFER_SIZE))
//	{
//		handle->rxBuffer.wp = handle->rxBuffer.buffer;
//	}
//	if (handle->rxBuffer.wp == handle->rxBuffer.rp)
//	{
//		handle->rxBuffer.overflow = true;
//	}
//}

//static uint8_t Vofa_GetByte(Vofa_HandleTypedef* handle,uint8_t* byte)
//{
//	if (handle->rxBuffer.rp == handle->rxBuffer.wp && !handle->rxBuffer.overflow)
//	{
//		return false;
//	}

//	if (handle->rxBuffer.overflow)
//	{
//		handle->rxBuffer.overflow = false;
//	}

//	*byte = *handle->rxBuffer.rp;
//	*handle->rxBuffer.rp = 0;
//	handle->rxBuffer.rp++;

//	if (handle->rxBuffer.rp == (handle->rxBuffer.buffer + VOFA_BUFFER_SIZE))
//	{
//		handle->rxBuffer.rp = handle->rxBuffer.buffer;
//	}

//	return true;
//}

//uint16_t Vofa_ReadCmd(Vofa_HandleTypedef* handle,uint8_t* buffer,uint16_t bufferLen)
//{
//	uint16_t length = 0;
//	uint16_t i = 0;
//	uint16_t tailCount = 0;

//	for(i = 0;i<bufferLen && Vofa_GetByte(handle,&buffer[i]) && tailCount < sizeof(cmdTail);i++)
//	{
//		if(buffer[i] == cmdTail[tailCount])
//		{
//			tailCount++;
//		}
//		else
//		{
//			tailCount = 0;
//		}

//		length++;
//	}
//	return length;
//}


//uint16_t Vofa_ReadLine(Vofa_HandleTypedef* handle,uint8_t* buffer,uint16_t bufferLen)
//{
//	uint16_t length = 0;
//	uint16_t i = 0;

//	for(i = 0;i < bufferLen && Vofa_GetByte(handle,&buffer[i]) && (buffer[i] != '\n');i++)
//	{
//		length++;
//	}
//	return length;
//}

//uint16_t Vofa_ReadData(Vofa_HandleTypedef* handle,uint8_t* buffer,uint16_t bufferLen)
//{
//	uint16_t length = 0;
//	uint16_t i = 0;

//	for(i = 0;i < bufferLen && Vofa_GetByte(handle,&buffer[i]);i++)
//	{
//		length++;
//	}
//	return length;
//}

////#ifdef __GNUC__
////__attribute__((weak))
//void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t length)
//{
//	uint16_t i;
//for(i = 0;i<length;i++)
//	{
//		HAL_UART_Transmit(&huart1,&data[i], length, 16);
//	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
//	}
//	
//	return;
//}

//__attribute__((weak))
//uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef *handle)
//{
//	return HAL_UART_Receive(&huart1, buffer,16 , 0);
//}
////#endif
/*
    使用说明
    调用函数
    Vofa_Send_Message 即可
*/
#include "vofa.h"
#include "string.h"
#include <stdbool.h>

Vofa_Data_t Vofa_Buffer =   { .data = {0x00, 0x00, 0x00, 0x00},
                              .tail = {0x00, 0x00, 0x80, 0x7f},
                            };
Vofa_Data_u Vofa_u;

/*
    brief: vofa调试使用，显示波形，justfloat协议
    param: huart: 使用的串口,在CUBE中开启对应的DMA
    return: None
*/
void Vofa_Send_Message(fp32* data)
{ 
	for(int i = 0; i < 4; i++)
	{
     Vofa_Buffer.data[i] = data[i];
  }
	Vofa_u.vofa_data = Vofa_Buffer;
	HAL_UART_Transmit(&huart1,(uint8_t*)Vofa_u.data,20,10);

}

