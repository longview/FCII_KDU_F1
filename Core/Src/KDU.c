/*
 * KDU.c
 *
 *  Created on: May 26, 2024
 *      Author: Robin
 */


#include "KDU.h"
#include "stdio.h"

int KDUKeyPressToString(KDU_KeyPress_t source, char* out, int maxlen)
{
	KDU_KeyPress_t shift;
	int retval = 0;
	for (int i = 0; i < 32; i++)
	{
		shift.regval = source.regval;
		shift.regval &= 1<<i;
		char character = KDUKeyPressToASCIIByte(shift);
		int len = 0;
		if (character != -1)
			len = snprintf(out, maxlen, "%c ", character);
		out+=len;
		retval += len;
	}

	return retval;
}

char KDUKeyPressToASCIIByte(KDU_KeyPress_t source)
{
	if (source.bits.Zero == 1)
		return '0';
	if (source.bits.One == 1)
			return '1';
	if (source.bits.Two == 1)
			return '2';
	if (source.bits.Three == 1)
			return '3';
	if (source.bits.Four == 1)
			return '4';
	if (source.bits.Five == 1)
			return '5';
	if (source.bits.Six == 1)
			return '6';
	if (source.bits.Seven == 1)
			return '7';
	if (source.bits.Eight == 1)
			return '8';
	if (source.bits.Nine == 1)
			return '9';
	if (source.bits.ENT == 1)
			return 'e';
	if (source.bits.CLR == 1)
			return 'd'; // ASCII DEL
	if (source.bits.RightArrow == 1)
			return '>';
	if (source.bits.LeftArrow == 1)
			return '<';
	if (source.bits.Pre_Plus == 1)
			return 'h';
	if (source.bits.Pre_Minus == 1)
			return 'j';
	if (source.bits.Vol_Plus == 1)
			return 'k';
	if (source.bits.Vol_Minus == 1)
			return 'l';
	return -1;
}
