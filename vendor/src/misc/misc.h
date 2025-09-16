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


/*! \file misc.h
 * \brief Miscellaneous functions
 * */

#ifndef SRC_MISC_MISC_H_
#define SRC_MISC_MISC_H_

#include <stdint.h>
#include <stddef.h>





/*! \brief Copy a number of byted from source to destination
 * \param dest Destination of the copy.
 * \param src Source of the copy.
 * \param n Number of bytes to be copied. */
void MISC_memcpy(uint8_t *dest, const uint8_t *src, size_t n);


#endif /* SRC_SYSTEM_SYSTEM_H_ */
