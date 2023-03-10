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
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "stdio.h"
#include "BH1750.h"
#include"fonts.h"
#include"test.h"
#include"ssd1306.h"
#include"bitmap.h"
#include "string.h"
#include<stdlib.h>
#include "STM32_Keypad4x4.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;


/* USER CODE BEGIN PV */
xTaskHandle Task0;
xTaskHandle Task1;
xTaskHandle Task2;
xTaskHandle Task3;
xTaskHandle Task4;
EventGroupHandle_t xEventGroup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);


/* USER CODE BEGIN PFP */
void Warning_system(void * argument);
void TempRead(void * argument);
void LuxRead(void * argument);
void Buttons(void * argument);
void ReceiveData(void  * argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar (int ch){
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,100);
	return ch;
}
typedef struct {
    int ID;
    int light;
    float Temp;
} myStruct_t;
QueueHandle_t myQueue;
myStruct_t Tranfer, Receive;
static uint32_t preid_temp=3000;
static uint32_t preid_lux=500;
float war_temp=30;
int war_lux=800;
int value=0;
float temp=0;
char buffer[8];
int chedo=3;
char Key;
char anhsang[4];
char nhiet_do[4];
int m1=0;int m2=0;int m3=0;int m4=0;
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
#define BIT_temp	( 1 << 0 )
#define BIT_lux	( 1 << 1 )
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
{ uint8_t  Temp_byte1, Temp_byte2;
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
void chuki(char* str,char b){
	int i=0;
 while(1){
	 b=  KEYPAD_Read();
	 if (b!='\0') {
		*(str+i)=b;
		i++;
		if (i==4) {
			break;
		}
		while(b != '\0')
					{
		b = KEYPAD_Read();
					}
	}
}

}

void hien_thi(const unsigned char * str){
	SSD1306_DrawBitmap(85, 0,str, 64, 64, 1);
	     SSD1306_UpdateScreen();
}
void canh_bao (){
	if(Receive.Temp < war_temp && value < war_lux)
	            	    {     m1++;m2=0;m3=0;m4=0;
	            	            if (m1==1) {
									SSD1306_Clear();
								}
	                     }else if (Receive.Temp > war_temp && value < war_lux) {
	                    	 m1=0;m2++;m3=0;m4=0;
	                    	 if (m2==1) {SSD1306_Clear();}
	                    	 hien_thi(nhietdo);
						}else if (Receive.Temp < war_temp && value > war_lux) {
							m1=0;m3++;m2=0;m4=0;
							if (m3==1) {SSD1306_Clear();}
							hien_thi(lux);
						}else if (Receive.Temp > war_temp && value > war_lux) {
							m1=0;m4++;m2=0;m3=0;
							if (m4==1) {SSD1306_Clear();}
							hien_thi(nguyhiem);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1,(uint8_t *)buffer, sizeof(buffer));
  HAL_TIM_Base_Start(&htim1);
    SSD1306_Init (); // initialize the display

    xTaskCreate(Buttons,"Buttons",50,NULL,4,&Task0);
    xTaskCreate(TempRead,"TempRead",100,NULL,4,&Task1);
    xTaskCreate(LuxRead,"LuxRead",100,NULL,4,&Task2);
    xTaskCreate(Warning_system,"WarningSystem",75,NULL, 8,&Task4);
    xTaskCreate(ReceiveData,"ReceiveData",200,NULL,6,&Task3);

    myQueue = xQueueCreate(10, sizeof(myStruct_t));
    xEventGroup = xEventGroupCreate();
    vTaskStartScheduler();
  /* USER CODE END 2 */


  /* We should never get here as control is now taken by the scheduler */
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance) {
		if(strstr(buffer,"che do")!=NULL)
		 {
			 sscanf(buffer+7, "%d", &chedo);
		  }
		if(strstr(buffer,"T1: ")!=NULL)
			{
			sscanf(buffer+4, "%d", (int *)&preid_temp);
			}
		if(strstr(buffer,"T2: ")!=NULL)
			{
					sscanf(buffer+4, "%d", (int *)&preid_lux);
			}
		if(strstr(buffer,"Tem:")!=NULL)
										 {
				  sscanf(buffer+4, "%f", &war_temp);
		  }
		if(strstr(buffer,"Lux:")!=NULL)
		 {
				   sscanf(buffer+4, "%d", &war_lux);
		  }
	HAL_UART_Receive_IT(&huart1,(uint8_t *)buffer, sizeof(buffer));
	}


}

void TempRead(void * argument)
	  {
	    /* USER CODE BEGIN 5 */
	uint32_t ticktemp;
	    /* Infinite loop */
	  	while (1)
	    {
	  		ticktemp= xTaskGetTickCount();
   	  	  temp=DS18B20_Readvalue ();
	  	  Tranfer.ID=1;
	        Tranfer.Temp=temp;
	        xQueueSend(myQueue,&Tranfer,portMAX_DELAY);
	        vTaskDelayUntil(&ticktemp, preid_temp);
	    }
	    /* USER CODE END 5 */
	  }


 void LuxRead(void * argument)
	  {
	    /* USER CODE BEGIN LuxRead */
	        uint32_t ticklux;
	  		BH1750_init_i2c(&hi2c2);
	  		BH1750_device_t* test_dev = BH1750_init_dev_struct(&hi2c2, "test device", true);
	  	    BH1750_init_dev(test_dev);
	    /* Infinite loop */
	   while (1)
	    {//Key = KEYPAD_Read();
		   ticklux= xTaskGetTickCount();
		  	  BH1750_get_lumen(test_dev);
	  	 	  value=test_dev->value;
	  	 	  	 	Tranfer.ID=2;
	  	 	  	 	Tranfer.light=value;
	  	 	  	xQueueSend(myQueue,&Tranfer,portMAX_DELAY);
	  	 	 vTaskDelayUntil(&ticklux, preid_lux);
	    }
	    /* USER CODE END LuxRead */
	  }


 void ReceiveData(void  * argument)
	  {
	    /* USER CODE BEGIN ReceiveData */

	  	char Temp[15] ;
	  	char light[15] ;
	  	uint32_t ticknow;

	    /* Infinite loop */
	  	while (1)
	    {
	      xQueueReceive(myQueue,&(Receive), portMAX_DELAY);
	      if (Receive.ID==1)
	        {
	    	ticknow= xTaskGetTickCount();
	  	  sprintf(Temp, "Temp:%.1f", Receive.Temp);
               canh_bao();
	  	       hien_thitemp(Temp);
	  	        if (chedo==1 || chedo == 3) {
	  	        	printf("Temperature: %.1f   S:%d\n",Receive.Temp,(int)ticknow);
				}
	  	      if (Receive.Temp >= war_temp) {
	  	      	  	               	xEventGroupSetBits(xEventGroup,BIT_temp);/* The bits being set. */
	  	      	  	   			}
	        }else if (Receive.ID==2)
	           {
	            ticknow= xTaskGetTickCount();
	  	      sprintf(light, "%d", Receive.light);
	  	       hien_thilux(light);
	  	     if (chedo==2 || chedo == 3) {
	  	       printf("Light: %d   S:%d\n",Receive.light,(int)ticknow);

	  	     }

	  	   if (Receive.light >= war_lux) {
	  	               	xEventGroupSetBits(xEventGroup,BIT_lux);/* The bits being set. */
	  	   			}
	  	     }


	    }
	    /* USER CODE END ReceiveData */
	  }

 void Buttons(void *argument)
 {
   /* USER CODE BEGIN 5 */

 	char a;
 	    /* Infinite loop */
 	  	while (1)
 	    {
 	  		Key = KEYPAD_Read();
 	  		     if (Key != KEYPAD_NOT_PRESSED) {
 	  		    	 if (Key=='*' || Key=='#') {
 	  					a=Key;
 	  		        	while(Key != '\0')
 	  					{
 	  						Key = KEYPAD_Read();
 	  					}
 	  		       if (a=='*') {
 	  		    	   chuki(anhsang, Key);
 	  		    	preid_lux = atof(anhsang);
 	  		    	memset(anhsang,'\0', sizeof(anhsang));
				   }else if (a=='#') {
					   chuki(nhiet_do, Key);
					   preid_temp = atof(nhiet_do);
					   memset(nhiet_do,'\0', sizeof(nhiet_do));
				}
 	  		         }


 	  		     }
                     vTaskDelay(100);
 	    }
   /* USER CODE END 5 */
 }
 void Warning_system(void * argument)
 	  {
 	    /* USER CODE BEGIN 5 */

 	    /* Infinite loop */
 	  	while (1)
 	    {
 	  		EventBits_t uxBits=xEventGroupWaitBits(
 	            xEventGroup,   /* The event group being tested. */
 	            BIT_temp|BIT_lux , /* The bits within the event group to wait for. */
 	            pdTRUE,        /* BIT_0 & BIT_4 should be cleared before returning. */
 	            pdFALSE,       /* Don't wait for both bits, either bit will do. */
 	            portMAX_DELAY );/* Wait a maximum of 100ms for either bit to be set. */

 	  		if (uxBits & BIT_temp) {
 	  			printf("The temperature is too high\n\n");
			}
 	  		if (uxBits & BIT_lux) {
 	  		 	printf("The light is too high\n\n");
 	  					}
 	    }
 	    /* USER CODE END 5 */
 	  }
/* USER CODE END 4 */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
