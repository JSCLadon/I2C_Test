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



/*! \file TLx493D.c */

#include "TLx493D.h"
#include <stdbool.h>
#include "src/misc/misc.h"
#include "TLV_A1B6/TLV_A1B6.h"
#include "TLE_AW2B6/TLE_AW2B6.h"
#include "TLE_AW2B6/driver/TLE_AW2B6_defines.h"
#include "src/debug/debug.h"


/* Local data */
static struct {
	TLV493D_sensor_type_t sensor_type;
	TLV493D_op_mode_t op_mode;
} _local;



///////////////////////////////////////////////////////
//					PUBLIC METHODS
///////////////////////////////////////////////////////
int32_t TLx493D_init(void)
{
	int32_t error;


	// initial state
	_local.sensor_type = TLx493D_TYPE_UNKNOWN;
	_local.op_mode = TLx493D_OP_MODE_NOT_INITIALIZED;
	// power up the sensor
	_POWER_DISABLE();
	_POWER_ENABLE();
	// attempt to init TLE493D-A2B6 or W2B6
	error = TLE493D_AW2B6_init(NULL, TLE493D_AW2B6_I2C_A0_ADDR);
	// A/W2B6 communication was successful
	if (!error) {
		_local.op_mode = TLx493D_OP_MODE_MCM;
		// the HW version is either A2B6 or W2B6, detect which one
		dbg_log("SENSOR TYPE: TLE493D\r\n");
		_local.sensor_type = TLE493D_get_hw_version(NULL);

		// { check if TLI actually }
		// Set to Fast mode to receive interrupt
		TLx493D_set_operation_mode(TLx493D_OP_MODE_FAST);

		// TODO: In order to support detection of TLI devices,
		// interrupts on the dedicated interrupt line of
		// TLI must be detectable at this point.
		// Enable detection of interrupts on TLI dedicated
		// Interrupt pin

		// TODO: wait 10ms for interrupt

		// TODO: if interrupt detected on TLI dedicated pin
		// this device is a TLI
		// set version to TLI if detected
		if (false) {
			_local.sensor_type = TLx493D_TYPE_TLI_W2BW;
		}
	} else {
		dbg_log("SENSOR TYPE: TLV493D\r\n");
		_local.op_mode = TLx493D_OP_MODE_POWER_DOWN;

		// power-cycle the sensor on kit
		_POWER_DISABLE();
		_POWER_ENABLE();

		// set HIGH logic level on ADDR pin and wait at least 200us
		_SET_ADDR_AND_WAIT(true);

		// attempt to init the TLV493D-A1B6 sensor
		error = TLV493D_A1B6_init(NULL, true, TLV493D_A1B6_ADDR_3E_BC);
		if (!error) {
			dbg_log("HW: A1B6\r\n");
			_local.sensor_type = TLx493D_TYPE_TLV_A1B6;
			_local.op_mode = TLx493D_OP_MODE_POWER_DOWN;
		} else {
			dbg_log("HW: Unknown\r\n");
		}
	}

	return error;
}

TLV493D_sensor_type_t TLx493D_get_sensor_type(void)
{
	return _local.sensor_type;
}

int32_t TLx493D_set_operation_mode(TLV493D_op_mode_t mode)
{
	int32_t error;

	if (TLx493D_TYPE_TLV_A1B6 == _local.sensor_type) {
		error = TLV493D_A1B6_set_operation_mode(NULL, mode);
	} else {
		error = TLE493D_AW2B6_set_operation_mode(NULL, mode);
	}
	if (!error) {
		_local.op_mode = mode;
	}

	return error;
}

TLV493D_op_mode_t TLx493D_get_operation_mode()
{
	return _local.op_mode;
}

int32_t TLx493D_read_frame(TLx493D_data_frame_t *frame)
{
	if(TLx493D_TYPE_TLV_A1B6 == TLx493D_get_sensor_type()) {
		return TLV493D_A1B6_read_frame(NULL, frame);
	} else {
		return TLE493D_AW2B6_read_frame(NULL, frame);
	}
}

uint8_t MISC_get_parity(uint8_t data)
{
	data ^= data >> 4;
	data ^= data >> 2;
	data ^= data >> 1;
	return data & 1U;
}
