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



/*! \file TLV_A1B6_driver.c */
#include "TLV_A1B6_driver.h"
#include "../../TLx493D.h"

#ifndef NULL
#define NULL ((void*) 0)
#endif

int32_t TLV493D_A1B6_read_regs(uint8_t addr, TLV493D_regmap_read_t *regmap, uint8_t upto)
{
	int32_t error;

	// read the registers from the sensor
	error = _I2C_read(addr, (uint8_t*)regmap, upto + 1);

	return error;
}

int32_t TLV493D_A1B6_write_regs(uint8_t addr, TLV493D_regmap_write_t *regmap, const TLV493D_regmap_read_t *regmap_check)
{
	int32_t error;
	uint8_t parity;

	// sanity check
	if (NULL == regmap) {
		return TLx493D_INVALID_ARGUMENT;
	}
	// correct reserved data
	if (NULL != regmap_check) {
		// Reg should be 0
		((uint8_t *)regmap)[TLV493D_A1B6_Res_REG] = 0U;
		// MOD1 Reserved data should contain bits 4:3 or register
		((uint8_t *)regmap)[TLV493D_A1B6_MOD1_REG]
					= (((uint8_t *)regmap)[TLV493D_A1B6_MOD1_REG] & ~TLV493D_A1B6_MOD1_Reserved_MSK)
					| (((uint8_t *)regmap_check)[TLV493D_A1B6_FactSet1_REG] & TLV493D_A1B6_MOD1_Reserved_MSK);
		((uint8_t *)regmap)[TLV493D_A1B6_Res2_REG] = ((uint8_t *)regmap_check)[TLV493D_A1B6_FactSet2_REG];
		((uint8_t *)regmap)[TLV493D_A1B6_MOD2_REG] = (((uint8_t *)regmap)[TLV493D_A1B6_MOD2_REG] & ~TLV493D_A1B6_MOD2_Reserved_MSK)
							   | (((uint8_t *)regmap_check)[TLV493D_A1B6_FactSet3_REG] & TLV493D_A1B6_MOD2_Reserved_MSK);
	}
	// reset parity bit
	((uint8_t *)regmap)[TLV493D_A1B6_MOD1_REG] &= ~ TLV493D_A1B6_MOD1_P_MSK;
	// compute EVEN parity bit
	parity = MISC_get_parity(((uint8_t *)regmap)[TLV493D_A1B6_Res_REG]);
	parity ^= MISC_get_parity(((uint8_t *)regmap)[TLV493D_A1B6_MOD1_REG]);
	parity ^= MISC_get_parity(((uint8_t *)regmap)[TLV493D_A1B6_Res2_REG]);
	parity ^= MISC_get_parity(((uint8_t *)regmap)[TLV493D_A1B6_MOD2_REG]);
	// compute ODD parity
	parity ^= 1;
	// add parity to register
	((uint8_t *)regmap)[TLV493D_A1B6_MOD1_REG] |= (parity << TLV493D_A1B6_MOD1_P_POS) & TLV493D_A1B6_MOD1_P_MSK;
	// write data to sensor
	error = _I2C_write(addr, (uint8_t*)regmap, TLV493D_A1B6_WRITE_REGS_COUNT);

	return error;
}

