/*
 * GLCD.h
 *
 *  Created on: May 25, 2024
 *      Author: Robin
 */

#ifndef INC_GLCD_H_
#define INC_GLCD_H_

#include <font4x6.h>
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

#ifndef LCD_WIDTH
#define LCD_WIDTH 122
#endif

#ifndef LCD_HEIGHT
#define LCD_HEIGHT 4
#endif

void DisplayDrawBarGraph(uint8_t *displaymemory, uint8_t *maxpos, uint8_t width,
		int8_t value, int8_t value2);
void DisplayWriteCharacter4x6(uint8_t *displaymemory, uint8_t* maxpos, char character);
void DisplayWriteCharacter5x7(uint8_t *displaymemory, uint8_t* maxpos, char* font, char character, bool invert);

#endif /* INC_GLCD_H_ */
