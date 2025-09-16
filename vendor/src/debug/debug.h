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


/*! \file debug.h
 *  \brief Debug and logging.
 *
 *  The functions in this file are used for debugging and logging over UART.
 * */

#ifndef SRC_DEBUG_DEBUG_H_
#define SRC_DEBUG_DEBUG_H_

#include <stdint.h>
#include "../TLx493D/TLx493D.h"




/*! \brief Enable/Disable (1/0) logging on UART */
#define _ENABLE_LOGGING_ 0



#if _ENABLE_LOGGING_ == 1
	//! \brief Pass string and string length to be written to UART
	#define dbg_log(x) _LOG_STR((uint8_t*)x, sizeof(x)-1)
	#warning LOGGING ENABLED
#else
	#define dbg_log(x) ;
#endif



/*! \brief Print the data frame to UART in ASCII form */
void dbg_print_frame_raw16_to_UART(TLx493D_data_frame_t *frame);


#endif /* SRC_DEBUG_DEBUG_H_ */
