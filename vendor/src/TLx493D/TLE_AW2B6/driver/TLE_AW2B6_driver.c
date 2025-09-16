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


/*! \file TLE_AW2B6_driver.c */

#include "src/TLx493D/interface.h"
#include "TLE_AW2B6_defines.h"
#include "TLE_AW2B6_driver.h"
#include "../../TLx493D.h"
#include "src/misc/misc.h"


#ifndef NULL
#define NULL ((void*)0)
#endif


int32_t TLE493D_AW2B6_read_regs(uint8_t i2c_addr, TLE493D_regmap_t *regmap, uint8_t upto)
{
	int32_t error;

	// sanity checks
	if (NULL == regmap) {
		return TLx493D_INVALID_ARGUMENT;
	}
	if (upto > (TLE493D_AW2B6_REGS_COUNT - 1)) {
		return TLx493D_INVALID_ARGUMENT;
	}
	// read all registers
	error = _I2C_read(i2c_addr, (uint8_t*)regmap, upto + 1);

	return error;
}

int32_t TLE493D_AW2B6_write_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data)
{
	uint8_t buf[2];
	int32_t error;

	buf[0] = reg_addr;
	buf[1] = data;
	// send register address and new value
	error = _I2C_write(i2c_addr, buf, 2U);

	return error;
}

int32_t TLE493D_AW2B6_write_reg_multi(uint8_t i2c_addr, uint8_t reg_addr_start, uint8_t *data, uint8_t count)
{
	uint8_t buf[TLE493D_AW2B6_REGS_COUNT + 1];
	int32_t error;


	// copy data to buffer
	buf[0] = reg_addr_start;
	MISC_memcpy(buf + 1, data, count);

	error = _I2C_write(i2c_addr, buf, count + 1);

	return error;
}
