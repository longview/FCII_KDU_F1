/*
 * KDU.h
 *
 *  Created on: May 25, 2024
 *      Author: Robin
 */

#ifndef INC_KDU_H_
#define INC_KDU_H_

#include "stdint.h"

typedef struct
{
	char TopLine[32]; // top/bottom lines can be inverted using 8th bit
	char BottomLine[32];
	char Frequency1Line[32];
	char Frequency2Line[32];
	int8_t TextMode; // 0 = standard, 1 = 4-line 5x7, 2 = 4-line 4x6, 3 = 4-line 4x6 with bars
	int8_t Arrow1; // 1 = arrow, 0 = nothing, -1 = inverted arrow
	int8_t Arrow2;
	int8_t TopBarGraph; // 0 ... 100
	int8_t TopBarGraph2; // 0 ... 100
	int8_t BottomBarGraph;
	int8_t BottomBarGraph2;
	int8_t Line1BarGraph; // 0 ... 100
	int8_t Line1BarGraph2; // 0 ... 100
	int8_t Line2BarGraph;
	int8_t Line2BarGraph2;
	int16_t LCD_Contrast;
	int8_t LED; // 0 = none, 1 = RED, 2 = GREEN
	int16_t Backlight; // 0-5
	int16_t Backlight_Minimum; // 0-32000
	int8_t WriteFRAM;
} DisplayData_t;

typedef union {
	struct {
		uint32_t Zero: 1;
		uint32_t One: 1;
		uint32_t Two: 1;
		uint32_t Three: 1;
		uint32_t LeftArrow: 1;
		uint32_t RightArrow: 1;
		uint32_t Vol_Plus: 1;
		uint32_t Four: 1;

		uint32_t Five: 1;
		uint32_t Six: 1;
		uint32_t CLR: 1;
		uint32_t Pre_Plus: 1;
		uint32_t Vol_Minus: 1;
		uint32_t Seven: 1;
		uint32_t Eight: 1;
		uint32_t Nine: 1;

		uint32_t ENT: 1;
		uint32_t Pre_Minus: 1;
		uint32_t RFU: 14;
	} bits;
	uint32_t regval;
} KDU_KeyPress_t;

typedef struct
{
	int8_t ASCIIKey; // currently pressed ASCII character for getc type processing
	uint8_t SequenceCounter; // increment by 1 per message sent
	uint8_t ResetReason;
	uint8_t SerialNo;

	KDU_KeyPress_t KeyMask;

	uint32_t TickCount; // on-time in ms
	uint16_t TicksSinceRX;

	uint16_t Backlight;
	uint16_t Contrast;

	uint16_t ProgrammedContrast;
	uint8_t FRAMStatus;
	char BuildDate[12];
	char BuildTime[10];
} KDU_KeyData_t;

int KDUKeyPressToString(KDU_KeyPress_t source, char* out, int maxlen);
char KDUKeyPressToASCIIByte(KDU_KeyPress_t source);

#endif /* INC_KDU_H_ */
