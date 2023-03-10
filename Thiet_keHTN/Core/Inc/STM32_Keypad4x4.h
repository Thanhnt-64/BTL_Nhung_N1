/**
 **************************************************************************************
 * @author  Sharath N
 * @file    STM32_Keypad4x4.h
 * @brief   Header file for interfacing 4*4 Matrix Keypad with STM32F411 Discovery Kit
 **************************************************************************************
**/ 


#ifndef STM32_KEYPADDRIVER_H
#define STM32_KEYPADDRIVER_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

/*------------------------Define your GPIO pins here - START----------------------------*/

/** 
  Keypad	STM32	       	 Description         I/O
     L1			PA4				Row 1           Output
     L2			PA5				Row 2           Output
     L3			PA6				Row 3           Output
     L4			PA7				Row 4           Output
		 
     R1			PA0				Column 1        Input
     R2			PA1	 			Column 2        Input
     R3			PA2				Column 3        Input
     R4			PA3				Column 4        Input
**/
/* Rows  ~ OUTPUT */
/* Row 1 default */
#define KEYPAD_ROW_1_PORT			  GPIOA
#define KEYPAD_ROW_1_PIN			  GPIO_PIN_5
/* Row 2 default */
#define KEYPAD_ROW_2_PORT			  GPIOA
#define KEYPAD_ROW_2_PIN			  GPIO_PIN_6
/* Row 3 default */
#define KEYPAD_ROW_3_PORT			  GPIOA
#define KEYPAD_ROW_3_PIN			  GPIO_PIN_7
/* Row 4 default */
#define KEYPAD_ROW_4_PORT			  GPIOB
#define KEYPAD_ROW_4_PIN			  GPIO_PIN_0

/* Columns ~ INPUT */
/* Column 1 default */
#define KEYPAD_COLUMN_1_PORT		GPIOA
#define KEYPAD_COLUMN_1_PIN			GPIO_PIN_1
/* Column 2 default */
#define KEYPAD_COLUMN_2_PORT		GPIOA
#define KEYPAD_COLUMN_2_PIN			GPIO_PIN_2
/* Column 3 default */
#define KEYPAD_COLUMN_3_PORT		GPIOA
#define KEYPAD_COLUMN_3_PIN			GPIO_PIN_3
/* Column 4 default */
#define KEYPAD_COLUMN_4_PORT		GPIOA
#define KEYPAD_COLUMN_4_PIN			GPIO_PIN_4

/*------------------------Define your GPIO pins here - END----------------------------*/
/* Keypad NOT pressed */
#define KEYPAD_NOT_PRESSED			'\0'


/**
 * @brief  Reads keypad data
 * @param  None
 * @retval Button status. This parameter will be a value of KEYPAD_Button_t enumeration
 */
char KEYPAD_Read(void);

#endif


