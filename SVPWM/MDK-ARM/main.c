/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "measure.h"
#include "arm_math.h"
#include <stdio.h>
#include "OLED.h"
#include "OLED_Data.h"
#include "svpwm.h"
#define	sqrt_2		1.414213f
#define	sqrt_3		1.732051f  // 添加sqrt(3)定义

#define duty_max	0.95f	
#define duty_min	0.05f	
#define	period		1799.0f
#define SINTABLE_LENGTH 500               // 正弦表长度
#define SAMPLE_LENGTH 1000                // 采样长度
#define AVERAGE_LENGTH 100
#define PRTABLE_LENGTH 500           
volatile float Vac_Offset = 2040.0f;
volatile float Vac_Factor = 45.5816f;
 
volatile float IL_Offset = 2051.7f;
volatile float IL_Factor = 687.82f;
 float Target_U, Actual_U,Target_I, Actual_I, Out=0;		

volatile float Vbus_Offset = 2042.0f;
volatile float Vbus_Factor = 45.2416f;
volatile uint16_t x = 0;
volatile uint16_t sin_n = 0;
volatile float cos_PF = 0.0f;
volatile float D_p = 0.0f, D_n = 0.0f;
volatile float adc[1600] = {0};
volatile float adc1[800] = {0};//PR???????
int16_t CCR,CCR1;
uint16_t LED_1ms,ADC_1ms,Aver_1ms,PR_1ms,SamplePointer,ThetaPointer,PRPointer;
uint16_t ADC_Ua,ADC_Ub,ADC_Ub_Max;
float I_Aver,I_Max;
uint16_t KeyNum;
uint16_t ADC_Single_U[AVERAGE_LENGTH]={0};
uint16_t ADC_Single_I[AVERAGE_LENGTH]={0};
float Feed[PRTABLE_LENGTH]={0};
__IO uint8_t AdcConvEnd = 0;
uint8_t buffer[202];
float ch[4]={0};
//Vofa_HandleTypedef vofa1;
float ADC_RMS_Value_U,ADC_RMS_Value_I,Reference,FeedBack;
float SIN_THETA[SAMPLE_LENGTH]={0};
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	
   {


	if (htim->Instance == TIM7) 
		{

		       ThreeLevel_SVPWM(arm_cos_f32(ADC_Ua),arm_cos_f32(ADC_Ub),arm_cos_f32(-ADC_Ua-ADC_Ub),24000,&compare);
			  if(CCR>5900){CCR=5900;}
			  if(CCR<-5900){CCR=-5900;}
			   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = CCR+6000;
			   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = 12000-(CCR+6000);
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP1xR = 12000-(CCR+6000);
		}
	}		
   
void ADC_GetValue(void)
  {

			if(ADC_1ms >= 20)
			{	float temp;
				temp=0;
				uint32_t i;
				ADC_1ms = 0;
				ADC_RMS_Value_U=measure_rms_u();                 
                Actual_U=ADC_RMS_Value_U;
				ADC_RMS_Value_I=measure_rms_i();
				Actual_I=ADC_RMS_Value_I;
		
			  for(i=0;i<SAMPLE_LENGTH;i++)
			{
				temp+=ADC_Value_I[i];
//                if(ADC_Ub_Max<=ADC_Value_I[i])
//				{
//					ADC_Ub_Max=ADC_Value_I[i];
//				}					
			}
			I_Aver=(temp/(SAMPLE_LENGTH*1.0f));
				temp=0;
			}


  }
  
void PID_Control(void)                              //PID??????��?
{
	
}

void OLED_UPDATE(void)                                              //OLED????????
  {


	      if (LED_1ms >= 100)
    {
	     
	
	}
  }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_HRTIM1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
    OLED_Init();
	
//	Vofa_Init(&vofa1, VOFA_MODE_SKIP);

	   HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_C);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);

    HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_D);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);

	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_E);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
	
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_F);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
	
//	HAL_TIM_Base_Start_IT(&htim1); 
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Ua,1);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&ADC_Ub,1);
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			// 更新OLED显示数据
			OLED_ShowString(0, 0, "Uo:", OLED_6X8);
			OLED_ShowFloat(24, 0, Actual_U, 2, OLED_6X8);  // 显示实际电压值
			OLED_ShowString(0, 10, "Io:", OLED_6X8);
			OLED_ShowFloat(24, 10, Actual_I, 2, OLED_6X8); // 显示实际电流值
			OLED_ShowString(0, 20, "Out:", OLED_6X8);
			OLED_ShowFloat(30, 20, Out, 2, OLED_6X8);      // 显示输出值
			OLED_ShowString(0, 30, "Erri:", OLED_6X8);     // 显示误差
			OLED_ShowString(0, 40, "M:", OLED_6X8);        // 显示调制比
			OLED_ShowString(0, 50, "Iave:", OLED_6X8);     // 显示平均电流
			OLED_ShowFloat(36, 50, I_Aver, 2, OLED_6X8);   // 显示平均电流值
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
