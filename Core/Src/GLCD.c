/*
 * GLCD.c
 *
 *  Created on: May 25, 2024
 *      Author: Robin
 */

#include "GLCD.h"

extern const uint8_t font_3x5_ext[];
extern const uint8_t font_3x5[];

void DisplayWriteCharacter5x7(uint8_t *displaymemory, uint8_t *maxpos,
		char *font, char character, bool invert) {
	if ((displaymemory + 5) > maxpos)
		return;

	memset((displaymemory), 0, 5);

	memcpy((displaymemory), &font[character * 5], 5);

	if (invert) {
		for (int i = 0; i < 4; i++)
			*(displaymemory + i) ^= 0xff;
	}

}

// use high bit to signal inverse character for these lines
void DisplayWriteCharacter4x6(uint8_t *displaymemory, uint8_t *maxpos,
		char character) {
	// bounds check
	if ((displaymemory + 3) > maxpos)
		return;

	memset((displaymemory), 0, 3);
	bool invert = false;
	if ((character & 0x80) > 0)
		invert = true;
	character &= 0x7F;

	int index = -1;
	const uint8_t *fontptr;

	// to upper case
	if ((character > 96) && (character < 123))
		character ^= 0x20;

	// the only valid characters are A-Z (upper case only)
	// and whatever special characters are handled below
	if (character >= 'A' && character <= 'Z')
		index = F_3x5_CHAR_LETTER_A + ((character - 'A') * 3);
	else if (character >= '0' && character <= '9')
		index = F_3x5_CHAR_NUMBER_0 + ((character - '0') * 3);
	else if (character == ' ')
		fontptr = &font_3x5_ext[0];
	else if (character == '-')
		fontptr = &font_3x5_ext[3];
	else if (character == '+')
		fontptr = &font_3x5_ext[6];
	else if (character == ':')
		fontptr = &font_3x5_ext[9];
	else if (character == '>')
		fontptr = &font_3x5_ext[12];
	else if (character == '<')
		fontptr = &font_3x5_ext[15];
	else
		return;

	if (index >= 0)
		fontptr = &font_3x5[index];

	memcpy((displaymemory), fontptr, 3);

	for (int i = 0; i < 3; i++)
		*(displaymemory + i) = *(displaymemory + i) << 1;

	if (invert) {
		for (int i = 0; i < 3; i++)
			*(displaymemory + i) ^= 0x7f;
		*(displaymemory - 1) |= 0x7f;
		*(displaymemory + 3) |= 0x7f;
	}

}

void DisplayDrawBarGraph(uint8_t *displaymemory, uint8_t *maxpos, uint8_t width,
		int8_t value, int8_t value2) {
	if (value < 0)
		return;

	float _val = (float)value/100.0f;
	_val *= (float)width;
	value = roundf(_val);

	_val = (float)value2/100.0f;
	_val *= (float)width;
	value2 = roundf(_val);

	if ((displaymemory + width) > maxpos)
		return;

	bool twoline = value2 != -1;

	*displaymemory = 0x7F;
	*(displaymemory + width + 1) = 0x7F;
	for (int i = 0; i < width; i++) {
		if ((displaymemory + i + 1) > maxpos)
			return;
		if (twoline)
		{
			if (i < value && i < value2)
				*(displaymemory + i + 1) = 0x77;
			else if (i < value)
				*(displaymemory + i + 1) = 0x47;
			else if (i < value2)
				*(displaymemory + i + 1) = 0x71;
			else
				*(displaymemory + i + 1) = 0x41;
		}
		else
		{
			if (i < value)
				*(displaymemory + i + 1) = 0x7f;
			else
				*(displaymemory + i + 1) = 0x41;
		}
	}
}

