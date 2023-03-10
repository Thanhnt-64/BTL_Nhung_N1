/**
 ****************************************************************************************
 * @author  Sharath N
 * @file    STM32_Keypad4x4.c
 * @brief   Source file for interfacing 4*4 Matrix Keypad with STM32F411 Discovery Kit
 ****************************************************************************************
**/

#include "STM32_Keypad4x4.h"

/* Row pins are output */
#define SET_ROW_1_HIGH              HAL_GPIO_WritePin(KEYPAD_ROW_1_PORT, KEYPAD_ROW_1_PIN, GPIO_PIN_SET)
#define SET_ROW_2_HIGH              HAL_GPIO_WritePin(KEYPAD_ROW_2_PORT, KEYPAD_ROW_2_PIN, GPIO_PIN_SET)
#define SET_ROW_3_HIGH              HAL_GPIO_WritePin(KEYPAD_ROW_3_PORT, KEYPAD_ROW_3_PIN, GPIO_PIN_SET)
#define SET_ROW_4_HIGH              HAL_GPIO_WritePin(KEYPAD_ROW_4_PORT, KEYPAD_ROW_4_PIN, GPIO_PIN_SET)
#define SET_ROW_1_LOW               HAL_GPIO_WritePin(KEYPAD_ROW_1_PORT, KEYPAD_ROW_1_PIN, GPIO_PIN_RESET)
#define SET_ROW_2_LOW               HAL_GPIO_WritePin(KEYPAD_ROW_2_PORT, KEYPAD_ROW_2_PIN, GPIO_PIN_RESET)
#define SET_ROW_3_LOW               HAL_GPIO_WritePin(KEYPAD_ROW_3_PORT, KEYPAD_ROW_3_PIN, GPIO_PIN_RESET)
#define SET_ROW_4_LOW               HAL_GPIO_WritePin(KEYPAD_ROW_4_PORT, KEYPAD_ROW_4_PIN, GPIO_PIN_RESET)

/*Column pin are input, so read the pins */
#define READ_COLUMN_1               HAL_GPIO_ReadPin(KEYPAD_COLUMN_1_PORT, KEYPAD_COLUMN_1_PIN)
#define READ_COLUMN_2               HAL_GPIO_ReadPin(KEYPAD_COLUMN_2_PORT, KEYPAD_COLUMN_2_PIN) 
#define READ_COLUMN_3               HAL_GPIO_ReadPin(KEYPAD_COLUMN_3_PORT, KEYPAD_COLUMN_3_PIN) 
#define READ_COLUMN_4               HAL_GPIO_ReadPin(KEYPAD_COLUMN_4_PORT, KEYPAD_COLUMN_4_PIN)        

/* Keypad Button Values*/
const char Keypad_Button_Values[4][4] =  {    {'1', '2', '3', 'A'},
											  {'4', '5', '6', 'B'},
											  {'7', '8', '9', 'C'},
											  {'*', '0', '#', 'D'},
                                         };


/*For specific row value sent , check all the cloumns*/
static char Check_Keypad_Column(uint8_t Row){
	if(!READ_COLUMN_1)                           //If COLUMN_1 is LOW
	{
		return Keypad_Button_Values[Row-1][0];
	}

	if(!READ_COLUMN_2)                           //If COLUMN_2 is LOW  
	{
		return Keypad_Button_Values[Row-1][1];
	}

    if(!READ_COLUMN_3)                           //If COLUMN_3 is LOW
	{
		return Keypad_Button_Values[Row-1][2];
	}

	if(!READ_COLUMN_4)                           //If COLUMN_3 is LOW
	{
		return Keypad_Button_Values[Row-1][3];
	}

	return KEYPAD_NOT_PRESSED;                   //If NO Key is pressed
}


/* Read the keypad */
char KEYPAD_Read(void)
{
    char check;
    /*Set ROW_1 LOW and scan all the columns*/
	SET_ROW_2_HIGH;
	SET_ROW_3_HIGH;
	SET_ROW_4_HIGH;
    SET_ROW_1_LOW;
    check = Check_Keypad_Column(1);
    if(check)
    	return check;

    /*Set ROW_2 LOW and scan all the columns*/
    SET_ROW_1_HIGH;
	SET_ROW_3_HIGH;
	SET_ROW_4_HIGH;
    SET_ROW_2_LOW;
    check = Check_Keypad_Column(2);
    if(check)
    	return check;
    /*Set ROW_3 LOW and scan all the columns*/
    SET_ROW_1_HIGH;
	SET_ROW_2_HIGH;
	SET_ROW_4_HIGH;
    SET_ROW_3_LOW;
    check = Check_Keypad_Column(3);
    if(check)
    	return check;
    /*Set ROW_4 LOW and scan all the columns*/
    SET_ROW_1_HIGH;
	SET_ROW_2_HIGH;
	SET_ROW_3_HIGH;
    SET_ROW_4_LOW;
    check = Check_Keypad_Column(4);
    if(check)
    	return check;
    /*Key not pressed */
    return KEYPAD_NOT_PRESSED;
}
	
	

																		 





