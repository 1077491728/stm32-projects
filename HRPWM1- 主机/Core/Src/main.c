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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "key.h"
#include "spwm.h"
#include "measure.h"
#include "pid.h"
#include "pfc.h"
#include "pr.h"
#include "arm_math.h"
#include <stdio.h>
#include "OLED.h"
#include "vofa.h"
#include "PLL.h"
#include "arm_math.h"
#include "CNTL_PI_F.h"
#include "Totem_PFC.h"
#include "OLED_Data.h"
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
#define	sqrt_2	1.414213f

#define duty_max	0.95f	
#define duty_min	0.05f	
#define	period		1799.0f
#define SINTABLE_LENGTH 500               //���ұ�ȡ����
#define SAMPLE_LENGTH 1000                //��������ȡ����
#define AVERAGE_LENGTH 100
#define PRTABLE_LENGTH 500           
volatile float Vac_Offset = 2040.0f;
volatile float Vac_Factor = 45.5816f;
 
volatile float IL_Offset = 2051.7f;
volatile float IL_Factor = 687.82f;
 

volatile float Vbus_Offset = 2042.0f;
volatile float Vbus_Factor = 45.2416f;
extern SPLL_1ph_SOGI_F spll1;
extern AC_DC_CTRL_DEF g_acdc_ctrlpar;
volatile uint16_t x = 0;
volatile uint16_t sin_n = 0;
volatile float cos_PF = 0.0f;
volatile float D_p = 0.0f, D_n = 0.0f;
volatile float adc[1600] = {0};
volatile float adc1[800] = {0};//PR���Ƶ���
int16_t CCR,CCR1;
uint16_t LED_1ms,ADC_1ms,Aver_1ms,PR_1ms,SamplePointer,ThetaPointer,PRPointer;
uint16_t ADC_U,ADC_I,ADC_I_Max;
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
//float erro;

QPRController prctrl1;


SOGI_PLL_DATA_DEF spll_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void InitTimerVariable(void);//��ʼ��SPWM

	
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	
   {
//    if (htim->Instance == TIM1) 
//	{

//	}
	if (htim->Instance == TIM6) 
		{

			Key_Tick();
			ADC_1ms+=1;
			PID_1ms+=1;
			LED_1ms+=1;
			Aver_1ms+=1;
            PR_1ms+=1;

		}
	if (htim->Instance == TIM7) 
		{

			ADC_Value_U[SamplePointer]=((ADC_U/4096.0f)*3.3f)*83.36518768f-43.78698146f;
			ADC_Value_I[SamplePointer]=((ADC_I/4096.0f)*3.3f)*7.220598f-10.8656f;         ;  //(((((float)ADC_I/2048.0f*3.3f)-1.65f)/(20.0f/13.7f))/10000.0f)*17000.0f;

    		  spll_sogi_func(&spll_data,ADC_Value_U[SamplePointer]-2032);            //SOGI,����
//      	  erro =((ADC_Value_U[SamplePointer]-2032)/(2771-2032))-spll_data.cos_theta;
			  SIN_THETA[ThetaPointer]=spll_data.cos_theta;               //�����������λ

//           if(Aver_1ms>=1000)                                  //����
//		   {
//		   Reference =1.0f*PRTable[PRPointer]/1000.0f;  //PR�ο����ұ�,��һ��ϵ���ɸı��ֵ
//		   FeedBack=ADC_Value_I[SamplePointer]-I_Aver;		                        //PR��������
//		   Feed[PRPointer]=FeedBack;
//		   Sin_Duty[DutyPointer] = QPR_Update(&prctrl1,Reference,FeedBack);     //PR���
//		   Aver_1ms=1000;
								                                                                           
			   CCR=Modulation/1000.0f*(Sin_Duty[DutyPointer])*1.0f;                    //��PRʱ��-6000.0f
			   CCR1=(SinTable[DutyPointer]-6000.0f)*1.0f;
			  if(CCR>5900){CCR=5900;}
			  if(CCR<-5900){CCR=-5900;}
			   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = CCR+6000;
			   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = 12000-(CCR+6000);
//			   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP1xR = CCR1+6000;
//			   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].CMP1xR = 12000-(CCR1+6000);   
//	   }
//           else
//		   {
			   CCR=Modulation/1000.0f*(Sin_Duty[DutyPointer]-6000.0f)*1.0f;                    //��PRʱ��-6000.0f
			   CCR1=(SinTable[DutyPointer]-6000.0f)*1.0f;
			  if(CCR>6000){CCR=6000;}
			  if(CCR<-6000){CCR=-6000;}
float vac       = ADC_U /  Vac_Factor;//������ѹ
		float iac       = ADC_I / IL_Factor;//��������
	float Vbus      = 10;//hadc1.Instance->JDR2-  Vbus_Offset) / Vbus_Factor;//ĸ�ߵ�ѹ
		
//		adc1[x] = vac;
		adc[x++] = Vbus;
		x%=1600;
		
		spll1.u[0] = vac;
		SPLL_1ph_SOGI_F_FUNC(&spll1);
		
		float Va    = spll1.u[0];				//PLL���ɵĦ���	 = ����ʵʱ��ѹ
		g_acdc_ctrlpar.vbus_f = Vbus;			// ĸ�ߵ�ѹʵʱֵ
		g_acdc_ctrlpar.vbus = Vbus;           // ĸ�ߵ�ѹʵʱֵ
		g_acdc_ctrlpar.vbus_ref = 30.0f;        // ĸ�ߵ�ѹ�ο�
		g_acdc_ctrlpar.iac = iac;               // �����������   PFC��е���
		g_acdc_ctrlpar.vac = vac;               // ������ѹʵʱֵ
		g_acdc_ctrlpar.sintheta =spll1.sin;     // ������λ���ҵ�ѹʵʱֵ ��sin(wt)�� 

		g_acdc_ctrlpar.ctrl_vol = Totem_PFC_ctrl(&g_acdc_ctrlpar);

		D_p = (1.0f+g_acdc_ctrlpar.ctrl_vol)*0.5f;//�Ӷ�Ӧ������1����ڵİ��Ų��ҽ��ڵ�ѹ������
		D_n = (1.0f-g_acdc_ctrlpar.ctrl_vol)*0.5f;//�Ӷ�Ӧ������2����ڵİ��Ų��ҽ��ڵ�ѹ���ĸ�
//			   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = (uint16_t)(D_p*period);
//			   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = (uint16_t)(D_n*period);
//		TIM1->CCR1 = (uint16_t)((SPWM[sin_n]-900)*0.95f+900);
//		TIM1->CCR2 = (uint16_t)((SPWM[sin_n++]-900)*0.95f+900);开环
			   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = CCR1+6000;
			   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = 12000-(CCR1+6000);  
//		   }
		   	  SamplePointer++;
				if(SamplePointer>SAMPLE_LENGTH-1)                                     //��װ������
			  {
				  SamplePointer=0;
			  } 
			  ThetaPointer++;
			  PRPointer++;
			  
		if(PRPointer>PRTABLE_LENGTH-1)                                     //��װ������
			  {
				  PRPointer=0;
			  } 
		if(ThetaPointer>SAMPLE_LENGTH-1)                                     //��װ������
			  {
				  ThetaPointer=0;
			  } 
               DutyPointer++;
		if(DutyPointer >SINTABLE_LENGTH-1)
              {
                  DutyPointer = 0;
              }
  
		}
	}		
   
void ADC_GetValue(void)
  {

			if(ADC_1ms >= 20)
			{	float temp;
				temp=0;
				uint32_t i;
				ADC_1ms = 0;
				ADC_RMS_Value_U=measure_rms_u();                 //�õ�������ѹ��Чֵ
                Actual_U=ADC_RMS_Value_U;
				ADC_RMS_Value_I=measure_rms_i();
				Actual_I=ADC_RMS_Value_I;
		
			  for(i=0;i<SAMPLE_LENGTH;i++)
			{
				temp+=ADC_Value_I[i];
//                if(ADC_I_Max<=ADC_Value_I[i])
//				{
//					ADC_I_Max=ADC_Value_I[i];
//				}					
			}
			I_Aver=(temp/(SAMPLE_LENGTH*1.0f));
				temp=0;
			}


  }
  
void PID_Control(void)                              //PID�ص�ѹ��Чֵ
{
	if(PID_1ms>=20)
	{
		PID_1ms=0;

//	    Modulation+=PID_Control_U();                //PID���Ƶ������         //�ӻ�
	    if(Modulation>=1080){Modulation=1080;}
	    if(Modulation<=400){Modulation=400;}
    }
}

void Key_readpin(void)                                             //�������
  {

      KeyNum = Key_GetNum();
	      if (KeyNum == 1)
    {
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
		Modulation-=100;
	}
  }
void OLED_UPDATE(void)                                              //OLED��ʾ��ˢ��
  {


	      if (LED_1ms >= 100)
    {
	      OLED_ShowFloatNum(40, 0, Actual_U, 4,3,OLED_6X8);
          OLED_ShowFloatNum(40, 10, Actual_I, 4,3,OLED_6X8);
	      OLED_ShowFloatNum(40, 20, Out, 2,4,OLED_6X8);
	      OLED_ShowFloatNum(40, 30, ErrorInt, 4,4,OLED_6X8);
		  OLED_ShowFloatNum(40, 40, Modulation, 4,2,OLED_6X8);
		  OLED_ShowFloatNum(40, 50, I_Aver, 4,2,OLED_6X8);
		  OLED_Update();
          LED_1ms=0;
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
  MX_HRTIM1_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	SOGI_PLL_Start();
    QPR_Init(&prctrl1,50.0f*value_2pi,20.0f,0.00002f,5500.0f,0.01f);//��Ƶ�ʣ���ֹ���������Ƶ�ʣ�kr��kp
    sogi_pll_init(&spll_data,50.0f*value_2pi,0.00002f);         //�ṹ�壬ԲƵ�ʣ�����Ƶ��
	
	InitTimerVariable();                                        //��ʼ��SPWM
    OLED_Init();
	
//	Vofa_Init(&vofa1, VOFA_MODE_SKIP);
	OLED_ShowString(0, 0, "Uo:", OLED_6X8);
	OLED_ShowString(0, 10, "Io:", OLED_6X8);
	OLED_ShowString(0, 20, "Out:", OLED_6X8);
	OLED_ShowString(0, 30, "Erri:", OLED_6X8);
	OLED_ShowString(0, 40, "M:", OLED_6X8);
	OLED_ShowString(0, 50, "Iave:", OLED_6X8);
//	int num;                 //OLED����
//	OLED_ShowNum(0, 0, num, 5,OLED_8X16);
//	num++;

    HAL_UART_Receive_DMA(&huart1,buffer,200);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
//	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_A);
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1);
//	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = 6000;      //�޸�TIMA1��CCR
	
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
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_U,1);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&ADC_I,1);
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
//    uint8_t justFloatTail[4] = {0x00, 0x00, 0x80, 0x7f};


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {            

          Key_readpin();
          OLED_UPDATE();
		  ADC_GetValue();
		  PID_Control();


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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
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
