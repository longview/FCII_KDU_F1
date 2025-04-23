/*
 * SSD1500.h
 *
 *  Created on: May 25, 2024
 *      Author: Robin
 */

#ifndef INC_SSD1500_H_
#define INC_SSD1500_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "main.h"

#define LCD_ON				0xAF
#define LCD_OFF            	0xAE
#define LCD_DISP_START		0xC0	// set display START line
#define LCD_SET_ADD        	0x00	// set column (Segment) address
#define LCD_SET_PAGE		0xB8
#define LCD_RESET			0xE2	// Reset is a command not a signal
#define LCD_ADC_RIGHTWARD	0xA0
#define LCD_ADC_LEFTWARD	0xA1
#define LCD_STATICDRIVE_OFF	0xA4	// Normal operation
#define LCD_STATICDRIVE_ON	0xA5	// static drive all segments lit (power save)
#define LCD_DUTY_16			0xA8	// 1/16 duty factor for driving LCD cells
#define LCD_DUTY_32			0xA9	// 1/32 duty factor for driving LCD cells
#define LCD_RMW				0xE0	// start RMW mode
#define LCD_RMW_END			0xEE	// end RMW mode

#define LCD_BUSY_FLAG		0x80
#define LCD_BUSY_BIT		7
#define LCD_ADC_FLAG		0x40	// ADC status 1 = nomral /forward 0=leftward/reverse
#define LCD_ADC_BIT			6		// ADC status 1 = nomral /forward 0=leftward/reverse

#define LCD_RESET_BIT		4
#define LCD_RESET_FLAG		0x10

void LCDWriteBitmap(uint8_t *displaymemory);
void LCDWriteLineBitmap(int page, int addr, uint8_t *displaymemory);
void FillLCD(uint8_t data);
void LCDSetAddr(uint8_t num);
void LCDSetPage(uint8_t num);
void InitLCD(int controllernum);
void LCDWaitBUSY();
uint8_t LCDReadStatus();
void LCDWrite(uint8_t data);
void LCDSetByte(uint8_t byte);

#endif /* INC_SSD1500_H_ */
