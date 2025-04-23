/*
 * crc32.h
 *
 *  Created on: May 4, 2024
 *      Author: Robin
 */

#ifndef INC_CRC32_H_
#define INC_CRC32_H_

#include "stdint.h"

uint32_t xcrc32 (const uint8_t *buf, int len, unsigned int init);
//static const unsigned int crc32_table[];

#endif /* INC_CRC32_H_ */
