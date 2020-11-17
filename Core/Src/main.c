/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom.h"
#include "usbd_cdc_if.h"
#include "lcd4bit.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define cls             10
#define up              11
#define down            12
#define exit            13
#define ok              14
#define dot             15

#define	Min1			0		//ADC
#define	Max1			4095

#define tedad_ersal			75
#define	Min_Real_Throttle	3272
#define	Max_Real_Throttle	6546

#define PWM_htim		&htim3
#define PWM_chanel		TIM_CHANNEL_4
#define	encoder_htim	TIM1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
const char info[] ={"\r\n***************************************\r\n"
                        "************* AeroPendulum ************\r\n"
                        "******* Seyyed Amir Ali Masoumi *******\r\n"
                        "**** PhoneNumber: +98 930 927 1137 ****\r\n"
                        "********* Telegram: @Mashroti *********\r\n"
                        "****** Email: Mashroty@gmail.com ******\r\n"
                        "***************************************\r\n"};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
struct Status
{
	uint8_t motor	: 1;
	uint8_t matlab	: 1;
	uint8_t whiles	: 1;
	uint8_t PID_whil: 1;
	uint8_t PID_OK	: 1;
	uint8_t Get_Num	: 1;
	uint8_t get_data: 1;
	uint8_t Online	: 1;
	uint8_t exe		: 1;
}Status;

struct PID{
	int8_t SetPoint;
	uint8_t time;
	double Kp;
	double Ki;
	double Kd;
}PID={45,0,20,0,0};

char RXBuffer[20],USARTbuffer[1000];


uint8_t NewDataLineCount=0;
uint8_t RXBufferCount=0;

uint8_t UsartNewDataLineCount=0;
uint8_t UsartRXBufferCount=0;
uint8_t NewDataInt;



uint16_t Min_Throttle ;	//PWM
uint16_t Max_Throttle ;	//6546
uint16_t throttle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void startup (void);
void Process_UART_Data(char* Data);
char GET_KEY(void);
void Matlab(void);
void KeyPad(void);
void Volume(void);
void Verify_Unique(void);
void PID_setting(void);
void Get_Number (char *NUMBER);

void USART_Process_Data(void);
void Online(void);
void WiFi_Setting(void);

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

void Log(char* log)
{
	CDC_Transmit_FS((uint8_t*)log,strlen(log));
}

void USART_log(char* log)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)log,strlen(log),100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	USARTbuffer[UsartRXBufferCount++]=NewDataInt;
	USARTbuffer[UsartRXBufferCount]=0;
	if(NewDataInt=='\n')
	{
		//USART_log(USARTbuffer);
		UsartNewDataLineCount=UsartRXBufferCount;
		UsartRXBufferCount=0;
	}
	else
	{
		HAL_UART_Receive_IT(&huart1,&NewDataInt,1);
	}
}
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
	int8_t Num;
	Status.matlab = 0;
	Status.Online = 0;
	Status.exe = 0;
	Status.get_data = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_FLASH_Unlock();
	EE_Init();
	EE_ReadVariable(0,&Min_Throttle);
	EE_ReadVariable(1,&Max_Throttle);
	HAL_FLASH_Lock();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(PWM_htim,PWM_chanel);
  __HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);

  lcd_init();
  lcd_hide_cursor();
  startup();
  Verify_Unique();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  lcd_clear();
  Num = 20;

  while (1)
  {
	if((Num >= 0 && Num <= 9 ) || Num == 20)
	{
		lcd_clear();
		Status.whiles=1;
			 if(Num == 2)	Matlab();
		else if(Num == 3)	KeyPad();
		else if(Num == 9)	Volume();
		else if(Num == 0)	WiFi_Setting();
		else if(Num == 4)
		{
			Status.exe = 1;
			KeyPad();
		}
		else if(Num == 1)
		{
			Status.Online = 1;
			HAL_UART_Receive_IT(&huart1,&NewDataInt,1);
			KeyPad();
		}
		lcd_clear();
		lcd_puts_XY(0,0,"1> Online");
		lcd_puts_XY(0,1,"2> Matlab");
		lcd_puts_XY(0,2,"3> Micro Controller");
		lcd_puts_XY(0,3,"4> Windows app");
		//lcd_puts_XY(0,3,"8> Manual adjustment");

	}
	Num = GET_KEY();
	if (Num==cls)__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 21;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65453;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_E_Pin|LCD_D7_Pin|LCD_D6_Pin
                          |LCD_D5_Pin|LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Trigger_Pin|Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin LCD_D7_Pin LCD_D6_Pin
                           LCD_D5_Pin LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin|LCD_D7_Pin|LCD_D6_Pin
                          |LCD_D5_Pin|LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Trigger_Pin Reset_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin|Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void startup(void)
{

	lcd_clear();
	uint8_t Num = GET_KEY();

	if(Num!=16)
	{
	  lcd_puts_XY(0,0,"SeyyedAmirAliMasoumi");
	  lcd_puts_XY(0,1,"Telegram : @Mashroti");
	  lcd_puts_XY(2,2,"+98 930 927 1137");
	  lcd_puts_XY(4,3,"AeroPendulum");
	  HAL_Delay(6000);
	}
	else
	{
	  lcd_puts_XY(4,0,"AeroPendulum");
	  lcd_puts_XY(1,1,"Motor Initializing");
	  lcd_puts_XY(4,2,"Please Wait");
	  for(uint8_t i=0 ;i<19 ;i++)
	  {
		  lcd_puts_XY(i,3,"\xff>");
		  HAL_Delay(300);
	  }
	}
	Log((char*)info);
	USART_log((char*)info);
}
/*****************************************************************************************************************
 ***********************************************                **************************************************
 ***********************************************   Usart Data   **************************************************
 ***********************************************                **************************************************
 *****************************************************************************************************************/
void Process_UART_Data(char* Data)
{
	char buffer[20];
	float p,i,d;
	int sp,time;
	NewDataLineCount=0;

	if(strstr(Data,"Angle?")!=0)
	{
		int16_t Angle = ((int16_t)TIM1->CNT)/4;
	    sprintf(buffer,"Angle:%04d\r\n",Angle);
	    Log(buffer);
	}

	else if(strstr(Data,"zenc")!=0)
	{
		TIM1->CNT = 0;
	}

	else if(strstr(Data,"PWM")!=0)
	{
		uint16_t track = atoi(strstr(Data,"PWM:")+4);
		__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,track);
	}
	else
	{
		int x = sscanf(Data,"kp%fki%fkd%fsp%dtime%d",&p,&i,&d,&sp,&time);
		if(x == 5)
		{
			PID.Kp = p;
			PID.Ki = i;
			PID.Kd = d;
			PID.SetPoint = sp;

			if(time == 555) Status.motor = 1;
			if(time == 444)
				{
					Status.motor = 0;
					__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);
				}
		}
	}
}
/*****************************************************************************************************************
 ***********************************************                **************************************************
 ***********************************************  VerifyUnique  **************************************************
 ***********************************************                **************************************************
 *****************************************************************************************************************/
void Verify_Unique(void)
{
	uint8_t i;
	uint8_t ERROR[13] = {75,111,115,32,78,97,78,97,116,32,58,41,0};

	uint32_t unique_id1[3] = {0X44EDD34, 0X350E3756, 0X2202021E};
	uint32_t unique_id2[3] = {0x18d21fd, 0x2910fc, 0x2f238418};
	//uint32_t unique_id2[3] = {0x21A2203, 0xfd2a18f7, 0x21013535};
/********************** GET ID **********************/
/*
	uint32_t unique_id[3];
	for(i=0 ; i<3 ;i++)
	{
		unique_id[i] = *(__IO uint32_t*)(UID_BASE+i*4);
	}

	for(i=0 ; i<3 ;i++)
	{
		unique_id2[i] = unique_id[i] - unique_id1[i];
	}
*/
/********************** GET ID **********************/

	for(i=0 ; i<3 ;i++)
	{
		if((uint32_t)(unique_id1[i]+unique_id2[i]) != *(__IO uint32_t*)(UID_BASE+i*4))
		{
		  while(1)
		  {
			  lcd_gotoxy(4, 2);
			  lcd_puts((char*)ERROR);
			  HAL_Delay(1000);
			  lcd_clear();
			  HAL_Delay(500);
		  }
		}
	}
}
/****************************************************************************************************************
 ***********************************************               **************************************************
 ***********************************************  Read KeyPad  **************************************************
 ***********************************************               **************************************************
 ****************************************************************************************************************/
char GET_KEY(void)
{
	unsigned char Num=16;
    char code[17]={ 1, 2, 3, 10,
                    4, 5, 6, 11,
                    7, 8, 9, 12,
                   15, 0,14, 13, 16};

    GPIOA->CRL = 0x88888888;
    GPIOA->ODR |= 0xFF;

    for(char i=0;i<4;i++)
    {
    	GPIOA->ODR &= ~(0x0001<<i);				//Pin reset (0)
        GPIOA->CRL &= ~(0x0000000f << (i*4));
        GPIOA->CRL |=  (0x00000002 << (i*4));	//Pin output

        HAL_Delay(2);

        if(((GPIOA->IDR & 0x000000FF) & 0xF0)!=0xF0)
        {
        	for(char x=0;x<4;x++)
        	{
        		if(((GPIOA->IDR & 0x000000FF) & 1<<(4+x)) != 1<<(4+x)) Num = i*4+x;
        	}
        }

        GPIOA->CRL &= ~(0x0000000f << (i*4));
        GPIOA->CRL |=  (0x00000008 << (i*4));	//Pin input
        GPIOA->ODR |=  (0x0001<<i);				//Pin set (1)
    }
    return code[Num];
}
/****************************************************************************************************************
 ***********************************************               **************************************************
 ***********************************************  Matlab Mode  **************************************************
 ***********************************************               **************************************************
 ****************************************************************************************************************/
void Matlab(void)
{
	int16_t Angle, pwm, darsad;
	uint8_t Buffer[21];

	char Num, Prev_Num;
	union u_type{
		double pid_exit;
		uint8_t u[8];
	}var;

	Status.motor=1;
	Status.matlab=1;
	NewDataLineCount = 1;

	while(Status.whiles)
	{

	    Angle = TIM1->CNT;	//get data from encoder
	    Angle /= 4;			//calculation Angle

		if(NewDataLineCount != 0)	//wait for new data
		{
			if(strstr(RXBuffer,"Angle?")!=0)
			{
				sprintf((char*)Buffer,"%04d\r\n",(int8_t)Angle);
				Log((char*)Buffer);
			}
			else
			{
				var.u[0] = RXBuffer[0];
				var.u[1] = RXBuffer[1];
				var.u[2] = RXBuffer[2];
				var.u[3] = RXBuffer[3];
				var.u[4] = RXBuffer[4];
				var.u[5] = RXBuffer[5];
				var.u[6] = RXBuffer[6];
				var.u[7] = RXBuffer[7];	//change received data to int16_t

				pwm = var.pid_exit + Min_Throttle;	//calculation PWM

				if(pwm > Max_Throttle) pwm = Max_Throttle;	//Limit Maximum pulse
				if(pwm < Min_Throttle) pwm = Min_Throttle;	//Limit Minimum pulse
				if(Status.motor) __HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,pwm);
			}

			NewDataLineCount = 0;		//clear flag
		}

		sprintf((char*)Buffer,"Angle= %d  ",(int8_t)Angle);
		lcd_puts_XY(0,0,(char*)Buffer);

		darsad = (((pwm-Min_Real_Throttle)*100)/(Max_Real_Throttle - Min_Real_Throttle));
		sprintf((char*)Buffer,"Puls=%%%03d ",darsad);
		lcd_puts_XY(0,2,(char*)Buffer);


		Num = GET_KEY();
		if(Num != Prev_Num)
		{
			switch (Num)
			{
				case exit:
					Status.whiles=0;
					break;
				case 0:
					TIM1->CNT = 0;
					break;
				case cls:
					Status.motor ^= 1;
					break;
			}
			Prev_Num = Num;
		}
		if(!Status.motor)
		{
			__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);
			lcd_puts_XY(11,2,"Motor OFF");
		}
		else lcd_puts_XY(11,2,"Motor ON ");
	}
	__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);
	Status.matlab=0;
}
/****************************************************************************************************************
 ***********************************************               **************************************************
 ***********************************************  KeyPad Mode  **************************************************
 ***********************************************               **************************************************
 ****************************************************************************************************************/
void KeyPad(void)
{
	double Error=0, Previous_error=0;
	double p_term=0, d_term=0, i_term=0, dt=0;

	int16_t Value=0, PWM=0, Angle=0;
	uint32_t online_time,timePrev, TickStart = HAL_GetTick();
	uint16_t darsad, exe_time,exe_tickstart;

	char Num, Prev_Num=ok;
	uint8_t Buffer[21],i;
	int16_t angle_send[200];
	uint16_t time_send[200];

	Status.motor = 0;
	throttle =  Max_Throttle - Min_Throttle;

	while(Status.whiles)
	{
		Num = GET_KEY();
		if(Num != Prev_Num)
		{
			switch (Num)
			{
				case exit:
					Status.whiles=0;
					break;
				case 0:
					TIM1->CNT = 0;
					break;
				case up:
					PID.SetPoint++;
					break;
				case down:
					PID.SetPoint--;
					break;
				case ok:
					__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);
					Status.motor = 0;i_term=0;d_term=0;
					PID_setting();
					break;
				case cls:
					Status.motor ^= 1;
					if(Status.motor == 0)
					{
						i_term=0;d_term=0;
						__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);
						if(Status.Online)
						{
							HAL_UART_Transmit(&huart1,(uint8_t*) "D", 1, 20);
							HAL_UART_Receive_IT(&huart1,&NewDataInt,1);
							PID.time = 0;
						}
					}
					TickStart = HAL_GetTick();
					break;
			}
			if(Num == ok)Prev_Num = exit;
			else Prev_Num = Num;
		}

	    Angle = ((int16_t)TIM1->CNT)/4;

	    if(Status.exe)
	    {
			if(NewDataLineCount != 0)
			{
			  Process_UART_Data(RXBuffer);
			  i_term = 0;
			  exe_time = 0;
			  TickStart = HAL_GetTick();
			  exe_tickstart = HAL_GetTick();
			}
	    }

		if(Status.Online)
		{
			if(UsartNewDataLineCount!=0)
			{
				USART_Process_Data();
				if(Status.get_data==0)
				{
					Status.get_data = 1;
					Status.motor = 1;
					online_time = HAL_GetTick();
					i=0;
				}
			}
		}

		if(Status.motor)
		{
		    timePrev	=	TickStart;
		    TickStart	=	HAL_GetTick();
		    dt			= 	((float)(TickStart - timePrev) / 1000.00);

			Error		=	PID.SetPoint - Angle;

			if(Status.exe)
			{
				exe_time = HAL_GetTick() - exe_tickstart;
				sprintf((char*)Buffer,"data,%d,%d\r\n",Angle,exe_time);
				Log((char*)Buffer);
			}

			if(Status.Online)
			{
				angle_send[i] = (int16_t)Angle;
				time_send[i] = HAL_GetTick() - online_time;

				if(time_send[i] >= PID.time*1000)
				{
					HAL_UART_Receive_IT(&huart1,&NewDataInt,1);
					Status.motor = 0;
					PID.time = 0;
				}

				i++;
				if(i >= tedad_ersal || Status.motor == 0)
				{
					for(uint8_t x=0 ; x<i ; x++)
					{
						sprintf((char*)Buffer,"%d@%d,",angle_send[x],(uint16_t)time_send[x]);
						HAL_UART_Transmit(&huart1,(uint8_t*)Buffer, strlen((char*)Buffer), 50);
					}
					if(Status.motor == 0)HAL_UART_Transmit(&huart1,(uint8_t*) "D", 1, 20);
					else HAL_UART_Transmit(&huart1,(uint8_t*) "E", 1, 20);
					i=0;
				}
			}

			p_term = PID.Kp * Error;

			i_term		=	(PID.Ki * (Error + Previous_error) * dt / 2) + i_term ;
			if(i_term > throttle) i_term = throttle;
			if(i_term <-throttle) i_term = -throttle;

            /*N 			=	(1.0 / dt) * 0.9 ;
            filter		+=	dt * d_term;
            d_term 		= 	(PID.Kd * Error - filter) * N;*/

			d_term = (Error - Previous_error) / dt * PID.Kd;
            if(d_term > throttle)	d_term = throttle;
            if(d_term <-throttle)	d_term = -throttle;

			PWM			=	(int16_t)(p_term + i_term + d_term);

			if (PWM > throttle)	PWM = throttle;
			if (PWM < 0)		PWM = 0;

			Value = PWM + Min_Throttle;
			__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Value);

			Previous_error = Error;
		}

		if(Status.motor) {lcd_puts_XY(11,3,"Motor ON ");}
		else
		{
			lcd_puts_XY(11,3,"Motor OFF");
			Value = Min_Real_Throttle;
			i_term = 0;d_term=0;
			if(Status.Online)__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);
		}
		sprintf((char*)Buffer,"SP= %03d    PV= %03d ",PID.SetPoint,Angle);
		lcd_puts_XY(0,0,(char*)Buffer);

		darsad = (((Value-Min_Real_Throttle)*100)/(Max_Real_Throttle - Min_Real_Throttle));
		sprintf((char*)Buffer,"P=%07.3f  Puls=%%%03d",PID.Kp,darsad);
		lcd_puts_XY(0,1,(char*)Buffer);

		sprintf((char*)Buffer,"I=%07.3f",PID.Ki);
		lcd_puts_XY(0,2,(char*)Buffer);
		sprintf((char*)Buffer,"D=%07.3f",PID.Kd);
		lcd_puts_XY(0,3,(char*)Buffer);
	}

	__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);
	Status.motor=0;Status.Online=0;
}
/****************************************************************************************************************
 ***********************************************               **************************************************
 ***********************************************  Volume Mode  **************************************************
 ***********************************************               **************************************************
 ****************************************************************************************************************/
void Volume(void)
{
	char Num,Prev_Num=ok;
	int Value = Min_Real_Throttle;
	char Buffer[15];
	int16_t encoder;

	Status.motor = 0;
	lcd_puts_XY(4,3,"Motor is OFF");

	HAL_FLASH_Unlock();
	EE_Init();

	while(Status.whiles)
	{
		Num = GET_KEY();
		if(Prev_Num == up || Prev_Num == down)Prev_Num=16;
		if(Num != Prev_Num)
		{
			switch (Num)
			{
			case exit:
				Status.whiles=0;
				break;
			case 0:
				TIM1->CNT = 0;
				break;
			case 1:
				Value = Min_Real_Throttle;
				break;
			case 2:
				Value = Max_Real_Throttle;
				break;
			case 7:
				Min_Throttle = Value;EE_WriteVariable(0,Min_Throttle);HAL_Delay(10);
				lcd_puts_XY(6,1,"Min Saved");HAL_Delay(250);lcd_puts_XY(6,1,"         ")
				break;
			case 8:
				Max_Throttle = Value;EE_WriteVariable(1,Max_Throttle);HAL_Delay(10);
				lcd_puts_XY(6,1,"Max Saved");HAL_Delay(250);lcd_puts_XY(6,1,"         ")
				break;
			case up:
				if(Value < Max_Real_Throttle)Value += 2;
				break;
			case down:
				if(Value > Min_Real_Throttle)Value -= 2;
				break;
			case cls:
				Status.motor ^= 1;
				if(Status.motor) {lcd_puts_XY(4,3,"Motor is ON ");}
				else
				{
					lcd_puts_XY(4,3,"Motor is OFF");
					Value = Min_Real_Throttle;
					__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Value);
				}
				break;
			case ok:
				break;
			}
			Prev_Num = Num;
		}

		if(Status.motor)__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Value);
		else 			__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);

		encoder = TIM1->CNT;
		float Angle = (float)encoder / 4;
	    sprintf(Buffer,"Angle= %05.2f  ",Angle);
	    lcd_puts_XY(4,0,Buffer);

		float darsad = (float)((Value - Min_Real_Throttle)*100)/(Max_Real_Throttle - Min_Real_Throttle);
		sprintf(Buffer,"Pulse= %%%3.2f  ",darsad);
		lcd_puts_XY(4,2,Buffer);

	    HAL_Delay(1);
	}
	__HAL_TIM_SET_COMPARE(PWM_htim,PWM_chanel,Min_Real_Throttle);

	HAL_FLASH_Lock();
}
/****************************************************************************************************************
 ***********************************************               **************************************************
 ***********************************************    Setting    **************************************************
 ***********************************************               **************************************************
 ****************************************************************************************************************/
void PID_setting(void)
{
	uint8_t Prev_Num=ok,Num;
	char Buffer[20];
	char NUMBER[9];
	int8_t i=0;
	double x;

	lcd_clear();

	do
	{
		lcd_puts_XY(6,0,"SetPoint");
		lcd_puts_XY(9,1,"KP");
		lcd_puts_XY(9,2,"KI");
		lcd_puts_XY(9,3,"KD");

		Status.whiles=1; Status.PID_whil=0;
		while(Status.whiles)
		{
			Num = GET_KEY();

			if(Num != Prev_Num)
			{
				lcd_puts_XY(3,i," ");
				lcd_puts_XY(17,i," ");

				if(Num==up)i--;
				if(Num==down)i++;
				if(i<0)i=3;
				if(i>3)i=0;

				lcd_puts_XY(3,i,">");
				lcd_puts_XY(17,i,"<");
				Prev_Num = Num;

				if(Num == ok){Status.whiles=0;Status.PID_whil=1;}
				if(Num == exit)Status.whiles=0;
			}
		}
		if(Status.PID_whil)
		{
			lcd_clear();

			if(i==0)
			{
				lcd_puts_XY(0,0,"------>SET SP<------");
				sprintf(Buffer,"Last SP= %02d ",PID.SetPoint);
				lcd_puts_XY(2,2,Buffer);
				lcd_puts_XY(3,3,"NEW SP= ");
			}
			if(i==1)
			{
				lcd_puts_XY(0,0,"------>SET KP<------");
				sprintf(Buffer,"Last KP= %07.4f  ",PID.Kp);
				lcd_puts_XY(2,2,Buffer);
				lcd_puts_XY(3,3,"NEW KP= ");
			}
			if(i==2)
			{
				lcd_puts_XY(0,0,"------>SET KI<------");
				sprintf(Buffer,"Last KI= %07.4f  ",PID.Ki);
				lcd_puts_XY(2,2,Buffer);
				lcd_puts_XY(3,3,"NEW KI= ");
			}
			if(i==3)
			{
				lcd_puts_XY(0,0,"------>SET KD<------");
				sprintf(Buffer,"Last KD= %07.4f  ",PID.Kd);
				lcd_puts_XY(2,2,Buffer);
				lcd_puts_XY(3,3,"NEW KD= ");
			}

			Get_Number(NUMBER);
			if(Status.PID_OK)
			{
				Prev_Num = ok;
				x = atof(NUMBER);
				if(x > 999) x = 999;
				switch(i)
				{
					case 0:
						if(x > 90) x = 90;
						if(x < 0) x = 0;
						PID.SetPoint = x;
					break;
					case 1:
						PID.Kp = x;
					break;
					case 2:
						PID.Ki = x;
					break;
					case 3:
						PID.Kd = x;
					break;
				}
			}else Prev_Num = exit;

		}
	}while(Status.whiles);

	Status.whiles=1;
	lcd_clear();
}
/****************************************************************************************************************
 ***********************************************               **************************************************
 ***********************************************   Get Number  **************************************************
 ***********************************************               **************************************************
 ****************************************************************************************************************/
void Get_Number (char *NUMBER)
{
  char Prev_Num=ok ,Num;
  uint8_t i;

  for(i=0;i<7;i++)*(NUMBER+i)=' ';
  i=0;

  Status.Get_Num=1;
  while(Status.Get_Num)
  {
    Num = GET_KEY();
    if(Prev_Num != Num && Num !=16)
    {
       switch (Num)
       {
        case cls:
            if(i>0)i--;
            NUMBER[i] = ' ';
            lcd_gotoxy(11+i, 3);
            lcd_putch(' ');
        break;

        case ok:
        	Status.Get_Num = 0;
            Status.PID_OK = 1;
        break;

        case exit:
        	Status.Get_Num = 0;
            Status.PID_OK = 0;
        break;

        case up:
        break;

        case down:
        break;

        case dot:
		if(i<7)
		{
		  NUMBER[i] = '.';
		  lcd_gotoxy(11+i, 3);
		  lcd_putch(NUMBER[i]);
		  i++;
		}
        break;

        default:
            if(i<8)
            {
              NUMBER[i] = Num + '0';
    		  lcd_gotoxy(11+i, 3);
    		  lcd_putch(NUMBER[i]);
              i++;
            }
       };

    }
    Prev_Num = Num;
  }
  Status.whiles=1;
  lcd_clear();
}

/****************************************************************************************************************
 ***********************************************               **************************************************
 ***********************************************     ONLINE    **************************************************
 ***********************************************               **************************************************
 ****************************************************************************************************************/
void USART_Process_Data(void)
{
	float p,i,d;
	int sp,time;

	int z = sscanf(USARTbuffer,"kp%fki%fkd%ftime%dsp%d", &p, &i, &d, &time, &sp);

	if(z == 5)
	{
		PID.Kp = p;
		PID.Ki = i;
		PID.Kd = d;
		PID.SetPoint = sp;

		if(time == 555) Status.motor = 1;
		if(time == 444) Status.motor = 0;
		if(time > 60) time = 60;
		PID.time = time;

		Status.get_data=0;
		if(Status.Online)
		{
			HAL_UART_Transmit(&huart1,(uint8_t*) "Y", 1, 20);
			HAL_Delay(20);
			HAL_UART_Transmit(&huart1,(uint8_t*) "Y", 1, 20);
			HAL_Delay(20);
			HAL_UART_Transmit(&huart1,(uint8_t*) "Y", 1, 20);
			HAL_Delay(20);
			HAL_UART_Transmit(&huart1,(uint8_t*) "Y", 1, 20);
		}
	}
	else
	{
		HAL_UART_Receive_IT(&huart1,&NewDataInt,1);
	}

	UsartNewDataLineCount=0;
}
/****************************************************************************************************************
 ***********************************************               **************************************************
 ***********************************************      WiFi     **************************************************
 ***********************************************               **************************************************
 ****************************************************************************************************************/
void WiFi_Setting(void)
{
	char Num;

	lcd_clear();
	lcd_puts_XY(0,0,"Connect to WiFi:");
	lcd_puts_XY(1,1,">> AeroPendulum <<");
	lcd_puts_XY(0,2,"And Type in browser:");
	lcd_puts_XY(1,3,">> 192.168.4.1 <<");

	HAL_GPIO_WritePin(Trigger_GPIO_Port, 	Trigger_Pin, 	GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(Reset_GPIO_Port, 		Reset_Pin, 		GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(Reset_GPIO_Port, 		Reset_Pin, 		GPIO_PIN_RESET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(Trigger_GPIO_Port, 	Trigger_Pin, 	GPIO_PIN_RESET);

	while(Status.whiles)
	{

		Num = GET_KEY();
		if(Num == exit)	Status.whiles=0;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
