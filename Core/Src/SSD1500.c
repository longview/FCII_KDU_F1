/*
 * SSD1500.c
 *
 *  Created on: May 25, 2024
 *      Author: Robin
 */

#include "SSD1500.h"

extern void delay_us(uint16_t au16_us);

void LCDSetByte(uint8_t byte)
{
	GPIOC->BSRR = byte | (byte^0xff)<<16;
}

void LCDWrite(uint8_t data)
{
	GPIOC->BSRR = LCD_WR_L_Pin << 16;
	LCDSetByte(data);
	delay_us(1);
	GPIOC->BSRR = LCD_WR_L_Pin;
}

uint8_t LCDReadStatus()
{
	GPIOC->BSRR = LCD_DATA_COMMAND_Pin << 16;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = DB0_Pin | DB1_Pin | DB2_Pin | DB3_Pin | DB4_Pin
			| DB5_Pin | DB6_Pin | DB7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	// swap level shifter direction
	HAL_GPIO_WritePin(LCD_DIR_GPIO_Port, LCD_DIR_Pin, DISABLE);

	GPIOC->BSRR = LCD_RD_L_Pin << 16;

	delay_us(1);

	uint8_t retval = GPIOC->IDR & 0xff;

	GPIOC->BSRR = LCD_RD_L_Pin;

	HAL_GPIO_WritePin(LCD_DIR_GPIO_Port, LCD_DIR_Pin, ENABLE);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIOC->BSRR = LCD_DATA_COMMAND_Pin;

	return retval;
}

void LCDWaitBUSY()
{
	while((LCDReadStatus() & 128) > 0)
		delay_us(1);
}

void InitLCD(int controllernum) {
	// LCD uses 8080 mode
	// set direction to LCD
	HAL_GPIO_WritePin(LCD_DIR_GPIO_Port, LCD_DIR_Pin, ENABLE);
	// enable bus output
	HAL_GPIO_WritePin(LCD_OE_L_GPIO_Port, LCD_OE_L_Pin, DISABLE);
	// set data bytes to output
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = DB0_Pin | DB1_Pin | DB2_Pin | DB3_Pin | DB4_Pin
			| DB5_Pin | DB6_Pin | DB7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	// set command mode
	HAL_GPIO_WritePin(LCD_DATA_COMMAND_GPIO_Port, LCD_DATA_COMMAND_Pin,
			DISABLE);
	// set chip select
	if (controllernum == 0)
		HAL_GPIO_WritePin(LCD_CS1_L_GPIO_Port, LCD_CS1_L_Pin, DISABLE);
	else
		HAL_GPIO_WritePin(LCD_CS2_L_GPIO_Port, LCD_CS2_L_Pin, DISABLE);

	LCDWrite(LCD_RESET);
	osDelay(50);
	LCDWrite(LCD_RMW_END);
	delay_us(10);
	LCDWrite(LCD_ON);
	delay_us(10);
	LCDWrite(LCD_DISP_START);
	delay_us(10);
	LCDWrite(LCD_STATICDRIVE_OFF);
	delay_us(10);
	LCDWrite(LCD_DUTY_32);
	delay_us(10);

	HAL_GPIO_WritePin(LCD_CS1_L_GPIO_Port, LCD_CS1_L_Pin, ENABLE);
	HAL_GPIO_WritePin(LCD_CS2_L_GPIO_Port, LCD_CS2_L_Pin, ENABLE);

	// set data mode
	HAL_GPIO_WritePin(LCD_DATA_COMMAND_GPIO_Port, LCD_DATA_COMMAND_Pin,
			ENABLE);
}

void LCDSetPage(uint8_t num)
{
	GPIOC->BSRR = LCD_DATA_COMMAND_Pin << 16;
	LCDWrite(LCD_SET_PAGE + num);
	GPIOC->BSRR = LCD_DATA_COMMAND_Pin;
	LCDWaitBUSY();
}

void LCDSetAddr(uint8_t num)
{
	GPIOC->BSRR = LCD_DATA_COMMAND_Pin << 16;
	LCDWrite(LCD_SET_ADD + num);
	GPIOC->BSRR = LCD_DATA_COMMAND_Pin;
	LCDWaitBUSY();
}



void FillLCD(uint8_t data)
{
	for (int i = 0; i < 4; i++)
	{
		LCDSetPage(i);
		LCDSetAddr(0);
		for (int j = 0; j < 61; j++)
		{
			LCDWrite(data);
			//LCDWaitBUSY();
		}
	}
}

void LCDWriteLineBitmap(int page, int addr, uint8_t *displaymemory)
{
	HAL_GPIO_WritePin(LCD_CS1_L_GPIO_Port, LCD_CS1_L_Pin, DISABLE);
	HAL_GPIO_WritePin(LCD_CS2_L_GPIO_Port, LCD_CS2_L_Pin, DISABLE);
	LCDSetPage(page);
	LCDSetAddr(addr);

	HAL_GPIO_WritePin(LCD_CS1_L_GPIO_Port, LCD_CS1_L_Pin, ENABLE);
	HAL_GPIO_WritePin(LCD_CS2_L_GPIO_Port, LCD_CS2_L_Pin, DISABLE);

	for (int i = 0; i < 61; i++)
	{
		LCDWrite(*(displaymemory+i));
		//LCDWaitBUSY();
	}


	HAL_GPIO_WritePin(LCD_CS1_L_GPIO_Port, LCD_CS1_L_Pin, DISABLE);
	HAL_GPIO_WritePin(LCD_CS2_L_GPIO_Port, LCD_CS2_L_Pin, ENABLE);

	for (int i = 0; i < 61; i++)
	{
		LCDWrite(*(displaymemory+i+61));
		//LCDWaitBUSY();
	}
	HAL_GPIO_WritePin(LCD_CS1_L_GPIO_Port, LCD_CS1_L_Pin, ENABLE);
	HAL_GPIO_WritePin(LCD_CS2_L_GPIO_Port, LCD_CS2_L_Pin, ENABLE);
}

void LCDWriteBitmap(uint8_t *displaymemory)
{
	LCDWriteLineBitmap(0, 0, displaymemory);
	LCDWriteLineBitmap(1, 0, displaymemory+122);
	LCDWriteLineBitmap(2, 0, displaymemory+244);
	LCDWriteLineBitmap(3, 0, displaymemory+366);
}
