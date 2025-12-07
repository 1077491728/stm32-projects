/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dac.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "PLL.h"
#include "arm_math.h"
#include "CNTL_PI_F.h"
#include "niming.h"
#include "Totem_PFC.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	sqrt_2	1.414213f

#define duty_max	0.95f	
#define duty_min	0.05f	
#define	period		1799.0f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

volatile float Vac_Offset = 2040.0f;
volatile float Vac_Factor = 45.5816f;
 
volatile float IL_Offset = 2051.7f;
volatile float IL_Factor = 687.82f;
 
volatile float Vbus_Offset = 2042.0f;
volatile float Vbus_Factor = 45.2416f;

volatile float adc[1600] = {0};
volatile float adc1[800] = {0};


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//volatile  优化变量在内存里读写     没有volatile的话直接在寄存器读写
extern SPLL_1ph_SOGI_F spll1;
extern AC_DC_CTRL_DEF g_acdc_ctrlpar;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //通道打开
//	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A); //开启子定时器A
	
	OLED_Init();
	//ADC校准
	HAL_Delay(50);
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);    //AD校准
	HAL_Delay(50);
	
	//SOGI初始化
	SOGI_PLL_Start();
	
	OLED_ShowString(1,1,"SUCCESS_PFC_20K");	//到目前成功初始化
	
	//开启ADC注入组
	HAL_ADCEx_InjectedStart(&hadc1);
	//开启pwm
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//触发采样  同步
	
	//DAC
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);  //开启DAC1
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4000); 
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4000); 
	
	//TIM2中断开启
	HAL_TIM_Base_Start_IT(&htim2);
	TIM1->CCR2 = 100;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

volatile uint16_t x = 0;
volatile uint16_t sin_n = 0;
volatile float cos_PF = 0.0f;
volatile float D_p = 0.0f, D_n = 0.0f;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *hitm)
{
	if(hitm ->Instance ==TIM2)
	{ 
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
		
		//开环测试
//		TIM1->CCR1 = (uint16_t)((SPWM[sin_n]-900)*0.95f+900);
//		TIM1->CCR2 = (uint16_t)((SPWM[sin_n++]-900)*0.95f+900);
//		if(sin_n == 400)	sin_n = 0;
		
		//PLL
		float vac       = (hadc1.Instance->JDR1 - Vac_Offset ) /  Vac_Factor;//交流电压
		float iac       = (hadc1.Instance->JDR3 - IL_Offset  ) / IL_Factor;//交流电流
		float Vbus      = (hadc1.Instance->JDR2-  Vbus_Offset) / Vbus_Factor;//母线电压
		
//		adc1[x] = vac;
		adc[x++] = Vbus;
		x%=1600;
		
		spll1.u[0] = vac;
		SPLL_1ph_SOGI_F_FUNC(&spll1);
		
		float Va    = spll1.u[0];				//PLL生成的α轴	 = 交流实时电压
		g_acdc_ctrlpar.vbus_f = Vbus;			// 母线电压实时值
		g_acdc_ctrlpar.vbus = Vbus;           // 母线电压实时值
		g_acdc_ctrlpar.vbus_ref = 30.0f;        // 母线电压参考
		g_acdc_ctrlpar.iac = iac;               // 电网输入电流   PFC电感电流
		g_acdc_ctrlpar.vac = vac;               // 电网电压实时值
		g_acdc_ctrlpar.sintheta =spll1.sin;     // 电网单位正弦电压实时值 （sin(wt)） 

		g_acdc_ctrlpar.ctrl_vol = Totem_PFC_ctrl(&g_acdc_ctrlpar);

		D_p = (1.0f+g_acdc_ctrlpar.ctrl_vol)*0.5f;//接对应交流线1相对于的半桥并且接在电压检测的正
		D_n = (1.0f-g_acdc_ctrlpar.ctrl_vol)*0.5f;//接对应交流线2相对于的半桥并且接在电压检测的负
		
		
		TIM1->CCR1 = (uint16_t)(D_p*period);
        TIM1->CCR2 = (uint16_t)(D_n*period);

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)(3000)); 
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, hadc1.Instance->JDR1); 
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
	}
}


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
