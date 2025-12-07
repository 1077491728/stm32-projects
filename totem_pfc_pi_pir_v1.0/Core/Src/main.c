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
#include "dac.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

__IO float theat					 = 0.0f;//6.25176808f;//0.0f;
//float sin_ref 			 	 = 0.0f;
//__IO float Duty 					 = 0.0f;

__IO uint16_t i = 0, x = 0 ,sample_cnt = 0;
__IO float ADC[1200] = {0.0f};
//ac
__IO float ac_voltage_pow_sum = 0;
__IO float ac_current_pow_sum = 0;
__IO float ac_voltage_rms = 0.0f;
__IO float ac_current_rms = 0.0f;
//----------------------------------------------------
//dc
__IO uint32_t dc_voltage_sum = 0;
__IO uint32_t dc_current_sum = 0;
__IO uint16_t dc_voltage_avg = 0;
__IO uint16_t dc_current_avg = 0;
__IO float dc_voltage = 0.0f;
__IO float dc_current = 0.0f;
//------------------------------------------------------
//soft_starer
float soft_starter_deep = 0.8f;
uint16_t soft_starter_cnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*---------------------------------------------------------------------------------
Totem_pfc 参考matlab仿真得出的码

Fs		20k
stm32f334r8平台
使用TIM1的PWM和IT

PLL		参考ti
PI		参考ti

-----------------------------------------------------------------------------------*/
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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
	GPIOC->ODR |= (1 << 14);
	OLED_Init();
	
	//ADC校准
	HAL_Delay(50);
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);    //AD校准
	HAL_Delay(50);
	
	//SOGI初始化
	SOGI_PLL_Init(50.0f, 20e3f, &spll1);
	
	//Notch_Fltr
	Notch_Fltr_Init(&Bus_Volt_notch, &notch_TwiceGridFreq, 
					 0.25f, 0.00001f, 100.0f, 20e3f);
	PR_Init(&pr);
	
	
	//开启ADC注入组
	HAL_ADCEx_InjectedStart(&hadc1);
  
  	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//用来触发同步采样
	
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);  //开启DAC1
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4000); 
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4000); 
	
	
	HAL_TIM_Base_Start_IT(&htim1);			//开启中断
	
  
	OLED_ShowString(2,1,"***Totem_PFC***");	//到目前成功初始化
	OLED_ShowString(3,1,"AC V:  .  I: .  ");
	OLED_ShowString(4,1,"DC V:  .  I: .  ");	
	GPIOC->ODR &= ~(1 << 14);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	  OLED_ShowNum(3,  6, (uint8_t)	 ac_voltage_rms, 2);
	  OLED_ShowNum(3,  9, (uint16_t)(ac_voltage_rms * 100) % 100, 2);
	  OLED_ShowNum(3, 13, (uint8_t)  ac_current_rms, 1);
	  OLED_ShowNum(3, 15, (uint16_t)(ac_current_rms * 100) % 100, 2);
	  OLED_ShowNum(4,  6, (uint8_t)dc_voltage, 2);
	  OLED_ShowNum(4,  9, (uint16_t)(dc_voltage * 100) % 100, 2);
	  OLED_ShowNum(4, 13, (uint8_t)dc_current, 1);
	  OLED_ShowNum(4, 15, (uint16_t)(dc_current * 100) % 100, 2);
	  OLED_ShowNum(1, 1, (uint16_t)(I_set_ref * 100), 3);
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

/***************ADC_Indicate****************/
//hadc1.Instance->JDR1		AC_I
//hadc1.Instance->JDR2		AC_V
//hadc1.Instance->JDR3		DC_V
//hadc1.Instance->JDR4		DC_I

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		GPIOC->ODR |= (1 << 13);
	
/********************************---temp sample v_i start---***********************************///--------------------		
		float vac       = (hadc1.Instance->JDR2 - Vac_Offset  ) * Vac_Factor;		//Vac
		float iac       = (hadc1.Instance->JDR1 - IL_Offset   ) * IL_Factor;		//Iac
		float Vbus_      =  hadc1.Instance->JDR3 * Vbus_Factor   - Vbus_Factor_B;	//Vbus
		float Ibus      = (hadc1.Instance->JDR4	- Ibus_Offset ) * Ibus_Factor;		//Ibus
		
		//Notch Fltr  直流母线陷波 
		Bus_Volt_notch.In = Vbus_; 
		NOTCH_FLTR_F_run(&Bus_Volt_notch, &notch_TwiceGridFreq); 
		float Vbus = Bus_Volt_notch.Out;
/********************************---temp sample v_i end---***********************************///--------------------		

/********************************---SPLL start---***********************************///--------------------		
		spll1.u[0] = vac;
		SPLL_1ph_SOGI_F_FUNC(&spll1);
/********************************---SPLL  end---***********************************///--------------------		

	//soft_starter
	if(5000 > soft_starter_cnt)
	{
		soft_starter_cnt++;
		soft_starter_deep += 0.00004f;
	}
		
	
/********************************---Totem_pfc_loop start---***********************************///--------------------			
		g_acdc_ctrlpar.vbus_f = Vbus;			// 母线电压实时值
		g_acdc_ctrlpar.vbus = Vbus;           	// 母线电压实时值
		g_acdc_ctrlpar.vbus_ref = 45.0f * soft_starter_deep;    // 母线电压参考
		g_acdc_ctrlpar.iac = iac;               // 电网输入电流   PFC电感电流
		g_acdc_ctrlpar.vac = vac;               // 电网电压实时值
		g_acdc_ctrlpar.sintheta =spll1.sin;     // 电网单位正弦电压实时值 （sin(wt)）
		

//		g_acdc_ctrlpar.ctrl_vol = Totem_PFC_ctrl(&g_acdc_ctrlpar) * 0.5f;
		g_acdc_ctrlpar.ctrl_vol = V_pi_I_pr_Loop(&pr, 45.0f * soft_starter_deep,
					 Vbus, vac, iac, arm_sin_f32(spll1.theta[0] - 0.0f)) * 0.5f;

/********************************---Totem_pfc_loop  end---***********************************///--------------------			
		
/********************************---duty write start---***********************************///--------------------			
		float D_p = 0.5f+g_acdc_ctrlpar.ctrl_vol;//接对应交流线1相对于的半桥并且接在电压检测的正  PA8 ccr1
		float D_n = 0.5f-g_acdc_ctrlpar.ctrl_vol;//接对应交流线2相对于的半桥并且接在电压检测的负  PA9 ccr2
		TIM1->CCR1 = (uint32_t) (D_p * (period - 1));
		TIM1->CCR2 = (uint32_t) (D_n * (period - 1));
/********************************---duty write end---***********************************///--------------------			

/********************************---sample start---***********************************///--------------------		
		//平方和---AC
		ac_voltage_pow_sum += vac * vac;
		ac_current_pow_sum += iac * iac;
		//Vbus
		dc_voltage_sum += hadc1.Instance->JDR3 - dc_voltage_avg;
		dc_voltage_avg 	= dc_voltage_sum >>7;
		dc_voltage 		= dc_voltage_avg * Vbus_Factor - Vbus_Factor_B;
		//Ibus
		dc_current_sum += hadc1.Instance->JDR4 - dc_current_avg - 2066;
		dc_current_avg 	= dc_current_sum >>10;
		dc_current 		= dc_current_avg * Ibus_Factor;
		//flag
		sample_cnt++;
		if(800 == sample_cnt)
		{	
			sample_cnt = 0;
			float ac_voltage_sqrt = 0.0f;
			arm_sqrt_f32(ac_voltage_pow_sum, &ac_voltage_sqrt);
			//RMS
			ac_voltage_rms = ac_voltage_sqrt * 0.035355339f;			
			ac_voltage_pow_sum = 0.0f;                                
			
			float ac_current_sqrt = 0.0f;
			arm_sqrt_f32(ac_current_pow_sum, &ac_current_sqrt);
			//RMS
			ac_current_rms = ac_current_sqrt * 0.035355339f ;    // *0.99208     
			ac_current_pow_sum = 0.0f;                                 
		}
/********************************---sample END---***********************************///--------------------	
		
/********************************---Dbug save start---***********************************///--------------------				
//		ADC[x] = spll1.sin;
//		x++;
//		x %=1200;
		
		DAC1->DHR12R1 = (spll1.sin + 1) * 2000;
		DAC1->DHR12R2 = hadc1.Instance->JDR2; 
		
/********************************---Dbug save END---***********************************///--------------------		
	
/********************************---Inverter start---***********************************///--------------------
/********************************---Inverter end---***********************************///--------------------		
		
		//C13=0
		GPIOC->ODR &= ~(1 << 13);
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
