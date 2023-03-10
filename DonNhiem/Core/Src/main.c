/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "BH1750.h"
#include"fonts.h"
#include"test.h"
#include"ssd1306.h"
#include"bitmap.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t u8_RxBuff[20];
uint8_t u8_RxData;
uint8_t _rxIndex;
int Tx_Flag = 3;
uint32_t ticknow;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern volatile uint32_t uwTick;
char strtest[20];
char Uart_recv[10];
int st[3] = {6000,1000,3000};
int m1 = 0, m2 =0, m3 =0, m4 =0;
uint32_t uwtick1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int value=0;
float temp;
int warning_temp = 90;
int warning_lux = 50;
////////
void delay (uint16_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
/*********************************** DS18B20 FUNCTIONS ****************************************/

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_0

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay (400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set as output

		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the data pin LOW
		delay (1);  // wait for > 1us

		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (50);  // wait for 60 us
	}
	return value;
}
float DS18B20_Readvalue ()
{
	uint8_t  Temp_byte1, Temp_byte2;
	uint16_t  TEMP;

	float Temperature = 0;

	 DS18B20_Start ();
		  	  HAL_Delay (1);
		  	  DS18B20_Write (0xCC);  // skip ROM
		  	  DS18B20_Write (0x44);  // convert t
		  	  HAL_Delay (800);

		  	   DS18B20_Start ();
		        HAL_Delay(1);
		        DS18B20_Write (0xCC);  // skip ROM
		        DS18B20_Write (0xBE);  // Read Scratch-pad

		        Temp_byte1 = DS18B20_Read();
		  	  Temp_byte2 = DS18B20_Read();
		  	  TEMP = (Temp_byte2<<8)|Temp_byte1;
		  	  Temperature = (float)TEMP/16;
		  	  return Temperature;
}
void hien_thilux(char * str);
void hien_thitemp (char * TEMP);
void hien_thi(const unsigned char* str);
BH1750_device_t* test_dev;
void task1();
void task2();
void canh_bao ();
int __io_putchar (int ch){
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,100);
	return ch;
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  SSD1306_Init (); // initialize the display
  BH1750_init_i2c(&hi2c2);
  test_dev = BH1750_init_dev_struct(&hi2c2, "test device", true);
  BH1750_init_dev(test_dev);
  HAL_UART_Receive_IT(&huart1,(uint8_t*)Uart_recv, 8);
  SSD1306_UpdateScreen();
  uwTick=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if(Tx_Flag == 3)
	 {
		 st[0] = 6000; st[1] =1000; st[2]=3000;//1 la canh bao, 2 la temp, 3 la lux
		 if (uwTick % st[0] == 0)
		 {
			 uwtick1 = uwTick;
			 canh_bao();

		 }
		 if (uwTick % st[1] == 100)
		 {
			 uwtick1 = uwTick;
			 task1();
		 }
		 if (uwTick % st[2] == 950)
		 {
		 	uwtick1 = uwTick;
		 	task2();
		 }
	 }
	 if(Tx_Flag == 1)
	 {
		 if (uwTick % st[0] == 0)
		 {
			 uwtick1 = uwTick;
			 canh_bao();
		 }
		 if (uwTick % st[1] == 100)
		 {
		 	uwtick1 = uwTick;
		 	task1();
		 }
	 }
	 if(Tx_Flag ==2)
	 {
		 if (uwTick % st[0] == 0)
		 {
			uwtick1 = uwTick;
		 	canh_bao();
		 }
		 if (uwTick % st[2] == 950)
		 {
			 uwtick1 = uwTick;
		 	task2();
		 }
	 }
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 50-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance)
	{
		if(strstr(Uart_recv,"t1:")!=NULL)
		{
			sscanf(Uart_recv+3,"%d",&st[1]);
		}
		if(strstr(Uart_recv,"t2:")!=NULL)
		{
			sscanf(Uart_recv+3,"%d",&st[2]);
		}
		if(strstr(Uart_recv,"chedo:")!=NULL)
		{
			sscanf(Uart_recv+7,"%d",&Tx_Flag);
		}
		if(strstr(Uart_recv,"bao1:")!=NULL)
		{
			sscanf(Uart_recv+6,"%d",&warning_temp);
		}
		if(strstr(Uart_recv,"bao2:")!=NULL)
		{
			sscanf(Uart_recv+6,"%d",&warning_lux);
		}
		HAL_UART_Receive_IT(&huart1,(uint8_t*)Uart_recv,8);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_11) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	st[0]= st[0] + 500;
    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
    }
    if(GPIO_Pin == GPIO_PIN_12) // If The INT Source Is EXTI Line9 (A9 Pin)
        {
        	st[1]= st[1] + 500;
        	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
     }
}
void task1 ()
{
	char Temp[15] ;
	ticknow= HAL_GetTick();
			  		temp=DS18B20_Readvalue ();
//			  		if(temp >= warning_temp)
//			  		{
//			  			printf("CHAY ROI::Temp: %.1f %d %d\n",temp,(int)ticknow, (int)(uwTick-uwtick1));
//			  			hien_thitemp(Temp);
//			  			SSD1306_DrawBitmap(85, 0, nhietdo, 64, 64, 1);
//			  			SSD1306_UpdateScreen();
//			  		}
//			  		else

			  			printf("Temp: %.1f %d %d\n",temp,(int)ticknow, (int)(uwTick-uwtick1));
			  			sprintf(Temp, "Temp:%.1f", temp);

			  			hien_thitemp(Temp);

}
void task2()
{
	char light[15] ;
	ticknow= HAL_GetTick();
			  		BH1750_get_lumen(test_dev);
			  		value=test_dev->value;
//			  		if(value >= warning_lux)
//			  		{
//			  			printf("SANG ROI::%d %d %d\n",value,(int)ticknow, (int)(uwTick-uwtick1));
//			  			hien_thilux(light);
//			  			SSD1306_DrawBitmap(85, 0, anh_sang, 64, 64, 1);
//			  			SSD1306_UpdateScreen();
//			  		}
//			  		else

			  			sprintf(light, "%d", value);
			  			hien_thilux(light);
			  			printf("Lux:%d %d %d\n",value,(int)ticknow, (int)(uwTick-uwtick1));

}
void canh_bao (){
	ticknow= HAL_GetTick();
	if(temp < warning_temp && value < warning_lux)
	{
		m1++;m2=0;m3=0;m4=0;
	    if (m1==1) {
	    	SSD1306_Clear();
	    }
	}else if (temp > warning_temp && value < warning_lux) {
		printf("CHAY ROI::%d\n",(int)temp);
	    m1=0;m2++;m3=0;m4=0;
	    if (m2==1) {
		   SSD1306_Clear();
	    }
	    hien_thi(nhietdo);
	}else if (temp < warning_temp && value > warning_lux) {
		printf("SANG ROI::%d\n",value);
		m1=0;m3++;m2=0;m4=0;
		if (m3==1) {
			SSD1306_Clear();}
		hien_thi(lux);
	}else if (temp > warning_temp && value > warning_lux) {
		printf("TOANG ROI::%d %d\n",value, (int)temp);
		m1=0;m4++;m2=0;m3=0;
		if (m4==1) {
			SSD1306_Clear();}
		hien_thi(nguyhiem);
	}
	printf("Canh bao:%d %d\n",(int)ticknow, (int)(uwTick-uwtick1));
}
void hien_thi(const unsigned char* str)
{
	SSD1306_DrawBitmap(85, 0, str, 64, 64, 1);
	SSD1306_UpdateScreen();
}
void hien_thilux(char * str) {
	//SSD1306_Clear ();
	//SSD1306_GotoXY (10,10); // goto 10, 10
	SSD1306_GotoXY (0, 10);
		  		SSD1306_Puts ("Lux:     ", &Font_11x18, 1);
		  		SSD1306_UpdateScreen();
		  		if(value < 10) {
		  			SSD1306_GotoXY (63, 10);  // 1 DIGIT
		  		}
		  		else if (value < 100 ) {
		  			SSD1306_GotoXY (55, 10);  // 2 DIGITS
		  		}
		  		else if (value < 1000 ) {
		  			SSD1306_GotoXY (47, 10);  // 3 DIGITS
		  		}
		  		else {
		  			SSD1306_GotoXY (40, 10);  // 4 DIGIS
		  		}
	  SSD1306_Puts (str, &Font_11x18, 1);
	  SSD1306_UpdateScreen();

}
void hien_thitemp (char * TEMP){
	SSD1306_GotoXY (0, 30);
	SSD1306_Puts (TEMP, &Font_11x18, 1);
		  SSD1306_UpdateScreen();
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
