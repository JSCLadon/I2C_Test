/*
*****************************************************************************
* Copyright (C) 2024 Infineon Technologies AG. All rights reserved.
*
* Infineon Technologies AG (INFINEON) is supplying this file for use
* exclusively with Infineon's products. This file can be freely
* distributed within development tools and software supporting such microcontroller
* products.
*
* THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
* INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR DIRECT, INDIRECT, INCIDENTAL,
* ASPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*
******************************************************************************
*/

#include <stdio.h>
#include "debug.h"
#include "src/TLx493D/interface.h"


void dbg_print_frame_raw16_to_UART(TLx493D_data_frame_t *frame)
{
	char buff[150];
	int i;

//	sprintf(buff, "x=%u %u y=%u %u z=%u %u\r\n",
//			(uint8_t)(frame->x >> 8) & 0xff,
//			(uint8_t)(frame->x & 0xff),
//			(uint8_t)(frame->y >> 8) & 0xff,
//			(uint8_t)(frame->y & 0xff),
//			(uint8_t)(frame->z >> 8) & 0xff,
//			(uint8_t)(frame->z & 0xff)
//	);

	sprintf(buff, "x=%d y=%d z=%d \r\n",
			(frame->x),
			(frame->y),
			(frame->z)
	);

	for (i = 0; buff[i] && i < 149; i++)
		;
	buff[149] = '\0';

	_LOG_STR(buff, i-1);
}
