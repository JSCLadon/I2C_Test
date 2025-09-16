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



/*! \file TLV_A1B6.c */

#include <stdint.h>
#include "driver/TLV_A1B6_defines.h"
#include "driver/TLV_A1B6_driver.h"

#include "TLV_A1B6.h"
#include "../TLx493D.h"

#ifndef NULL
#define NULL ((void*) 0)
#endif

///////////////////////////////////////////////////////////////////
//							LOCAL DATA
///////////////////////////////////////////////////////////////////

static struct {
	TLV493D_data_t data;
} _local = {
	.data = {0},
};

///////////////////////////////////////////////////////////////////
//							     PUBLIC
///////////////////////////////////////////////////////////////////

int32_t TLV493D_A1B6_init(TLV493D_data_t *data, bool addr_high, TLV493D_address_t addr_type)
{
	int32_t error;
	uint8_t addr_temp = 0;


	// use local data if no data struct provided
	if (NULL == data) {
		data = &(_local.data);
	}
	// determine the sensor inital address
	if (addr_high) {
		data->IIC_addr = TLV493D_A1B6_I2C_DEFAULT_ADDR_HIGH;
	} else {
		data->IIC_addr = TLV493D_A1B6_I2C_DEFAULT_ADDR_LOW;
	}
	data->ADDR_high = addr_high;
	data->frame_count = 10;
	// copy state of read registers in local memory (used for write registers)
	error = TLV493D_A1B6_read_regs(	data->IIC_addr,
					&(data->regmap_read),
					TLV493D_A1B6_READ_REGS_COUNT-1
	);
	if (!error) {
		/* prepare register values to be written
		 * initial configuration for POWER DOWN mode, INT disabled
		 * */
		data->regmap_write.MOD1 &= ~(TLV493D_A1B6_MOD1_IICAddr_MSK
						| TLV493D_A1B6_MOD1_INT_MSK
						| TLV493D_A1B6_MOD1_FAST_MSK
						| TLV493D_A1B6_MOD1_LOW_MSK);
		data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_INT_DISABLE
					   | TLV493D_A1B6_MOD1_FAST_DISABLE
					   | TLV493D_A1B6_MOD1_LOW_DISABLE;
		// add address bits
		data->addr_type = addr_type;
		switch (addr_type) {
		case TLV493D_A1B6_ADDR_3E_BC:
			data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_IICAddr_3E_BC;
			addr_temp = (!addr_high ? 0x3EU : 0xBCU);
			break;

		case TLV493D_A1B6_ADDR_36_B4:
			data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_IICAddr_36_B4;
			addr_temp = (!addr_high ? 0x36U : 0xB4U);
			break;

		case TLV493D_A1B6_ADDR_1E_9C:
			data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_IICAddr_1E_9C;
			addr_temp = (!addr_high ? 0x1EU : 0x9CU);
			break;

		case TLV493D_A1B6_ADDR_16_94:
			data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_IICAddr_16_94;
			addr_temp = (!addr_high ? 0x16U : 0x94U);
			break;
		default:
			return TLx493D_INVALID_ARGUMENT;
		}

		/* temperature measurement enabled
		 * low power mode instead of ultra-low power mode
		 * Parity test disabled for future writes
		 * */
		data->regmap_write.MOD2 &= ~(TLV493D_A1B6_MOD2_T_MSK
					 | TLV493D_A1B6_MOD2_LP_MSK
					 | TLV493D_A1B6_MOD2_PT_MSK);
		data->regmap_write.MOD2 |= TLV493D_A1B6_MOD2_T_ENABLE
					 | TLV493D_A1B6_MOD2_LP_LOW_POWER
					 | TLV493D_A1B6_MOD2_PT_DISABLE;
		// write changes to sensor
		error = TLV493D_A1B6_write_regs(data->IIC_addr,
						&(data->regmap_write),
						&(data->regmap_read)
		);
	}
	// mark initialization as completed successfully
	if (!error) {
		data->IIC_addr = addr_temp;
	}

	return error;
}

int32_t TLV493D_A1B6_set_operation_mode(TLV493D_data_t *data, TLV493D_op_mode_t mode)
{
	int32_t error;


	error = 0;
	// use local data if no data struct provided
	if (NULL == data) {
		data = &(_local.data);
	}
	// clear the mode bits
	data->regmap_write.MOD1 &= ~(TLV493D_A1B6_MOD1_INT_MSK
					| TLV493D_A1B6_MOD1_FAST_MSK
					| TLV493D_A1B6_MOD1_LOW_MSK);
	// set bits and enable/disable interrupt handing
	switch (mode) {
		case TLx493D_OP_MODE_MCM:
			data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_INT_DISABLE
						  | TLV493D_A1B6_MOD1_FAST_ENABLE
						  | TLV493D_A1B6_MOD1_LOW_ENABLE;
			break;
		case TLx493D_OP_MODE_ULTRA_LOW_POWER:
			data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_INT_ENABLE
						  | TLV493D_A1B6_MOD1_FAST_DISABLE
						  | TLV493D_A1B6_MOD1_LOW_ENABLE;
			data->regmap_write.MOD2 &= ~(TLV493D_A1B6_MOD2_LP_MSK);
			data->regmap_write.MOD2 |= TLV493D_A1B6_MOD2_LP_ULTRA_LOW_POWER;
			break;
		case TLx493D_OP_MODE_LOW_POWER:
			data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_INT_ENABLE
						  | TLV493D_A1B6_MOD1_FAST_DISABLE
						  | TLV493D_A1B6_MOD1_LOW_ENABLE;
			data->regmap_write.MOD2 &= ~(TLV493D_A1B6_MOD2_LP_MSK);
			data->regmap_write.MOD2 |= TLV493D_A1B6_MOD2_LP_LOW_POWER;
			break;
		case TLx493D_OP_MODE_FAST:
			data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_INT_ENABLE
						  | TLV493D_A1B6_MOD1_FAST_ENABLE
						  | TLV493D_A1B6_MOD1_LOW_DISABLE;
			break;
		case TLx493D_OP_MODE_POWER_DOWN:
			data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_INT_DISABLE
						  | TLV493D_A1B6_MOD1_FAST_DISABLE
						  | TLV493D_A1B6_MOD1_LOW_DISABLE;
			break;
		default:
			// execute the error sequence
			goto error;
	}
	/* write registers (do not correct reserved data,
	 * already corrected on initialization) */
	error = TLV493D_A1B6_write_regs(data->IIC_addr,
			&(data->regmap_write),
			NULL
	);
	if (error) {
		goto error;
	}
	// EXIT function - success
	return 0;

error:
	return error;
}

void TLV493D_A1B6_hard_reset_reconfigure(TLV493D_data_t *data)
{
	// use local data if no data struct provided
	if (NULL == data) {
		data = &(_local.data);
	}
	// power-cycle the sensor on kit
	_POWER_DISABLE();
	_POWER_ENABLE();
	// set desired logic level on ADDR pin and wait at least 200us
	_SET_ADDR_AND_WAIT(data->ADDR_high);
	// reinitialize sensor
	TLV493D_A1B6_init(data, data->ADDR_high, data->addr_type);
}

int32_t TLV493D_A1B6_read_frame(TLV493D_data_t *data, TLx493D_data_frame_t *frame)
{
	//uint8_t new_frame_count;
	int16_t xtemp, ytemp, ztemp, temperature;
	uint8_t* regs, new_frame_count;
	bool fast_mode;
	int32_t error;


	// use local data if no data struct provided
	if (NULL == data) {
		data = &(_local.data);
	}
	// is fast mode?
	fast_mode = (TLx493D_OP_MODE_FAST == TLx493D_get_operation_mode());
	// read registers
	error = TLV493D_A1B6_read_regs(data->IIC_addr,
					&(data->regmap_read),
					TLV493D_A1B6_Temp2_REG
	);
	if (error) {
		return error;
	}
	// cast register map to uint8_t array
	regs = (uint8_t *)&(data->regmap_read);
	// extract data
	//new_frame_count = (regs[3] & 0xC) >> 2;
	xtemp = ((uint16_t)regs[0] << 4);
	ytemp = ((uint16_t)regs[1] << 4);
	ztemp = ((uint16_t)regs[2] << 4);
	// if not in fast mode, include supplementary data
	if (!fast_mode) {
		xtemp |= (regs[4] & 0xF0) >> 4;
		ytemp |= regs[4] & 0x0F;
		ztemp |= regs[5] & 0x0F;
		temperature = ((uint16_t)(regs[3] & 0xF0) << 4) + regs[6];
	}
	/* Method for detecting an lock of the IC according to Early Problem Notification */
	//if framecounter has not incremented -> reset sensor
	new_frame_count = (regs[TLV493D_A1B6_Temp_REG] & TLV493D_A1B6_Temp_FRM_MSK) >> TLV493D_A1B6_Temp_FRM_POS;
	if (new_frame_count == data->frame_count) {
		// reset and reconfigure the sensor
		TLV493D_A1B6_hard_reset_reconfigure(data);
		data->frame_count = 10;
		// invalidate frame
		return TLx493D_INVALID_FRAME;
	} else {
		// save new frame counter
		data->frame_count = new_frame_count;
	}
	/* Extend Bits for Signed Values  */
	if (xtemp & 0x800) {
		xtemp |= 0xF000;
	}
	if (ytemp & 0x800) {
		ytemp |= 0xF000;
	}
	if (ztemp & 0x800) {
		ztemp |= 0xF000;
	}
	// copy data to frame
	frame->x = xtemp;
	frame->y = ytemp;
	frame->z = ztemp;
	frame->temp = temperature;

	// change with a define NO_ERROR
	return 0;
}

int32_t TLV493D_A1B6_set_temp_measure(TLV493D_data_t *data, bool enabled)
{
	int32_t error;


	// use local data if no data struct provided
	if (NULL == data) {
		data = &(_local.data);
	}
	// clear bit
	data->regmap_write.MOD2 &= ~TLV493D_A1B6_MOD2_T_MSK;
	// set bit value
	if (enabled) {
		data->regmap_write.MOD2 |= TLV493D_A1B6_MOD2_T_ENABLE;
	} else {
		data->regmap_write.MOD2 |= TLV493D_A1B6_MOD2_T_DISABLE;
	}
	// write changes to the sensor
	error = TLV493D_A1B6_write_regs(data->IIC_addr,
			&(data->regmap_write),
			NULL
	);

	return error;
}

int32_t TLV493D_A1B6_set_parity_test(TLV493D_data_t *data, bool enabled)
{
	int32_t error;


	// use local data if no data struct provided
	if (NULL == data) {
		data = &(_local.data);
	}
	// clear bit
	data->regmap_write.MOD2 &= ~TLV493D_A1B6_MOD2_T_MSK;
	// set bit value
	if (enabled) {
		data->regmap_write.MOD2 |= TLV493D_A1B6_MOD2_PT_ENABLE;
	} else {
		data->regmap_write.MOD2 |= TLV493D_A1B6_MOD2_PT_DISABLE;
	}
	// write changes to the sensor
	error = TLV493D_A1B6_write_regs(data->IIC_addr,
			&(data->regmap_write),
			NULL
	);

	return error;
}

int32_t TLV493D_A1B6_set_IIC_address(TLV493D_data_t *data, TLV493D_address_t new_addr_type)
{
	uint8_t addr_temp;
	int32_t error;


	// use local data if no data structure provided
	if (NULL == data) {
		data = &(_local.data);
	}
	data->regmap_write.MOD1 &= ~TLV493D_A1B6_MOD1_IICAddr_MSK;
	// add address bits
	switch (new_addr_type) {
	case TLV493D_A1B6_ADDR_3E_BC:
		data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_IICAddr_3E_BC;
		addr_temp = (!data->ADDR_high ? 0x3EU : 0xBCU);
		break;

	case TLV493D_A1B6_ADDR_36_B4:
		data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_IICAddr_36_B4;
		addr_temp = (!data->ADDR_high ? 0x36U : 0xB4U);
		break;

	case TLV493D_A1B6_ADDR_1E_9C:
		data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_IICAddr_1E_9C;
		addr_temp = (!data->ADDR_high ? 0x1EU : 0x9CU);
		break;

	case TLV493D_A1B6_ADDR_16_94:
		data->regmap_write.MOD1 |= TLV493D_A1B6_MOD1_IICAddr_16_94;
		addr_temp = (!data->ADDR_high ? 0x16U : 0x94U);
		break;
	default:
		return TLx493D_INVALID_ARGUMENT;
	}

	// write changes to sensor
	error = TLV493D_A1B6_write_regs(data->IIC_addr,
					&(data->regmap_write),
					NULL
	);
	// mark update process as completed successfully
	if (!error) {
		data->IIC_addr = addr_temp;
	}

	return error;
}


void TLV493D_A1B6_get_data(TLV493D_data_t *dest)
{
	// sanity check
	if (NULL == dest) {
		return;
	}

	// get data
	*dest = _local.data;
}

int32_t TLV493D_A1B6_set_data(TLV493D_data_t *src)
{
	int32_t error;

	// sanity check
	if (NULL == src) {
		return TLx493D_INVALID_ARGUMENT;
	}

	// write to sensor
	error = TLV493D_A1B6_write_regs(_local.data.IIC_addr,
					&(src->regmap_write),
					&(src->regmap_read)
	);
	if (0 != error) {
		return error;
	}
	// set data
	_local.data = *src;
	_local.data.frame_count = 10;

	return TLx493D_OK;
}
