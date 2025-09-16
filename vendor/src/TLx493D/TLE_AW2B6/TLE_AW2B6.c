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


/*! \file TLE_AW2B6.c */

#include "../TLx493D.h"
#include "TLE_AW2B6.h"
#include "driver/TLE_AW2B6_defines.h"
#include "driver/TLE_AW2B6_driver.h"
#include "src/misc/misc.h"
#include "src/TLx493D/interface.h"
#include "src/debug/debug.h"


static struct {
	TLE493D_data_t data;
} _local;



///////////////////////////////////////////////////////
//					INTERNALS    //
///////////////////////////////////////////////////////

uint8_t TLE493D_AW2B6_get_FP_bit(TLE493D_data_t *data)
{
	uint8_t parity;

	// compute parity of MOD1 register
	parity = MISC_get_parity(data->regmap.MOD1 & ~(TLE493D_AW2B6_MOD1_FP_MSK));
	// add parity of MOD2:PRD register bits
	parity ^= MISC_get_parity(data->regmap.MOD2 & (TLE493D_AW2B6_MOD2_PRD_MSK));
	// compute ODD parity
	parity ^= 1;

	return parity & 1;
}

uint8_t TLE493D_AW2B6_get_CP_bit(TLE493D_data_t *data)
{
	uint8_t *regs, parity, i;


	regs = (uint8_t *)&(data->regmap);
	parity = 0;
	// even parity for registers XL to ZH
	for (i = TLE493D_AW2B6_XL_REG; i <= TLE493D_AW2B6_ZH_REG; i++) {
		parity ^= regs[i];
	}
	// include WU
	parity ^= regs[TLE493D_AW2B6_WU_REG] & ~(TLE493D_AW2B6_WU_WA_MSK);
	// include TMode
	parity ^= regs[TLE493D_AW2B6_TMode_REG] & ~(TLE493D_AW2B6_TMode_TST_MSK);
	// include TPhase
	parity ^= regs[TLE493D_AW2B6_TPhase_REG] & ~(TLE493D_AW2B6_TPhase_PH_MSK);
	// include Config
	parity ^= regs[TLE493D_AW2B6_Config_REG] & ~(TLE493D_AW2B6_Config_CP_MSK);
	// compute parity bit
	parity = MISC_get_parity(parity);
	// convert to ODD parity
	parity ^= 1U;

	return parity & 1U;
}

static int32_t _set_operation_mode_MCM(TLE493D_data_t *data)
{
	int32_t error;


	// clear bits
	data->regmap.MOD1 &= ~(TLE493D_AW2B6_MOD1_CA_MSK
				| TLE493D_AW2B6_MOD1_INT_MSK
				| TLE493D_AW2B6_MOD1_MODE_MSK
				| TLE493D_AW2B6_MOD1_FP_MSK
	);
	// set protocol and parity bits
	data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_CA_ENABLE
				| TLE493D_AW2B6_MOD1_INT_DISABLE
				| TLE493D_AW2B6_MOD1_MODE_MCM;
	data->regmap.MOD1 |= TLE493D_AW2B6_get_FP_bit(data) << TLE493D_AW2B6_MOD1_FP_POS;
	// write register
	error = TLE493D_AW2B6_write_reg(data->IIC_addr,
					TLE493D_AW2B6_MOD1_REG,
					data->regmap.MOD1
	);
	if (!error) {
		// clear and set trigger bits
		data->regmap.Config &= ~TLE493D_AW2B6_Config_TRIG_MSK;
		data->regmap.Config |= TLE493D_AW2B6_Config_TRIG_R6;
		error = TLE493D_AW2B6_write_reg(data->IIC_addr,
						TLE493D_AW2B6_Config_REG,
						data->regmap.Config
		);
	}

	return error;
}

static int32_t _set_operation_mode_low_power(TLE493D_data_t *data)
{
	int32_t error;


	// clear bits
	data->regmap.MOD1 &= ~(TLE493D_AW2B6_MOD1_CA_MSK
				| TLE493D_AW2B6_MOD1_INT_MSK
				| TLE493D_AW2B6_MOD1_MODE_MSK
				| TLE493D_AW2B6_MOD1_FP_MSK
	);
	// set protocol and parity bits
	data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_CA_ENABLE
				| TLE493D_AW2B6_MOD1_INT_ENABLE
				| TLE493D_AW2B6_MOD1_MODE_LOW_POWER;
	data->regmap.MOD1 |= TLE493D_AW2B6_get_FP_bit(data) << TLE493D_AW2B6_MOD1_FP_POS;
	// write register
	error = TLE493D_AW2B6_write_reg(data->IIC_addr,
					TLE493D_AW2B6_MOD1_REG,
					data->regmap.MOD1
	);

	// disable ADC trigger
	if (!error) {
		data->regmap.Config &= ~TLE493D_AW2B6_Config_TRIG_MSK;
		data->regmap.Config |= TLE493D_AW2B6_Config_TRIG_NONE;
		error = TLE493D_AW2B6_write_reg(
				data->IIC_addr,
				TLE493D_AW2B6_Config_REG,
				data->regmap.Config
		);
	}

	return error;
}

static int32_t _set_operation_mode_fast(TLE493D_data_t *data)
{
	int32_t error;


	// clear bits
	data->regmap.MOD1 &=
				~(TLE493D_AW2B6_MOD1_CA_MSK
				| TLE493D_AW2B6_MOD1_FP_MSK
				| TLE493D_AW2B6_MOD1_INT_MSK
				| TLE493D_AW2B6_MOD1_MODE_MSK
	);
	// set protocol and parity bits
	data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_CA_ENABLE
				| TLE493D_AW2B6_MOD1_INT_ENABLE
				| TLE493D_AW2B6_MOD1_MODE_FAST_MODE;
	data->regmap.MOD1 |= TLE493D_AW2B6_get_FP_bit(data) << TLE493D_AW2B6_MOD1_FP_POS;
	// write register
	error = TLE493D_AW2B6_write_reg(data->IIC_addr,
					TLE493D_AW2B6_MOD1_REG,
					data->regmap.MOD1
	);

	// disable ADC trigger
	if (!error) {
		data->regmap.Config &= ~TLE493D_AW2B6_Config_TRIG_MSK;
		data->regmap.Config |= TLE493D_AW2B6_Config_TRIG_NONE;
		error = TLE493D_AW2B6_write_reg(
				data->IIC_addr,
				TLE493D_AW2B6_Config_REG,
				data->regmap.Config
		);
	}

	return error;
}


///////////////////////////////////////////////////////
//					PUBLIC							 //
///////////////////////////////////////////////////////


int32_t TLE493D_AW2B6_init(TLE493D_data_t *data, TLE493D_address_t i2c_addr)
{
	int32_t error = 0;


	if (NULL == data) {
		data = &(_local.data);
	}
	data->IIC_addr = i2c_addr;
	// prepare register
	data->regmap.MOD1 = 0;
	// needed to compute the FP bit
	data->regmap.MOD2 = 0;
	// set address bits to maintain current address
	switch (i2c_addr) {
		case TLE493D_AW2B6_I2C_A0_ADDR:
			data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_IICadr_A0;
			break;
		case TLE493D_AW2B6_I2C_A1_ADDR:
			data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_IICadr_A1;
			break;
		case TLE493D_AW2B6_I2C_A2_ADDR:
			data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_IICadr_A2;
			break;
		case TLE493D_AW2B6_I2C_A3_ADDR:
			data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_IICadr_A3;
			break;
	}
	/* Initial settings:
	 * 1-Byte read protocol
	 * Collision avoidance
	 * Interrupt disabled
	 * Master Control Mode
	 * */
	data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_PR_1BYTE
					 | TLE493D_AW2B6_MOD1_CA_ENABLE
					 | TLE493D_AW2B6_MOD1_INT_DISABLE
					 | TLE493D_AW2B6_MOD1_MODE_MCM;
	data->regmap.MOD1 |= (TLE493D_AW2B6_get_FP_bit(data) << TLE493D_AW2B6_MOD1_FP_POS);
	// write registers
	error = TLE493D_AW2B6_write_reg(data->IIC_addr,
					TLE493D_AW2B6_MOD1_REG,
					data->regmap.MOD1
	);
	if (!error) {
		// write config reg, set trigger
		data->regmap.Config = TLE493D_AW2B6_Config_TRIG_R6;
		error = TLE493D_AW2B6_write_reg(
			data->IIC_addr,
			TLE493D_AW2B6_Config_REG,
			data->regmap.Config
		);
	}

	if (!error) {
		// read entire register map
		error = TLE493D_AW2B6_read_regs(
			data->IIC_addr,
			&(data->regmap),
			(TLE493D_AW2B6_REGS_COUNT - 1)
		);
	}

	return error;
}

int32_t TLE493D_AW2B6_set_operation_mode(TLE493D_data_t *data, TLV493D_op_mode_t mode)
{
	int32_t error;

	if (NULL == data) {
		data = &(_local.data);
	}
	switch (mode) {
		case TLx493D_OP_MODE_MCM:
			error = _set_operation_mode_MCM(data);
			break;
		case TLx493D_OP_MODE_LOW_POWER:
			error = _set_operation_mode_low_power(data);
			break;
		case TLx493D_OP_MODE_FAST:
			error = _set_operation_mode_fast(data);
			break;
		default:
			error = -1;
			break;
	}

	return error;
}

int32_t TLE493D_AW2B6_read_frame(TLE493D_data_t *data, TLx493D_data_frame_t *frame)
{
	int16_t xtemp, ytemp, ztemp, ttemp;
	bool fastmode;
	int32_t error;
	uint8_t *regs;


	if (NULL == data) {
		data = &(_local.data);
	}
	// check if in fast mode
	fastmode = (TLx493D_OP_MODE_FAST == TLx493D_get_operation_mode());
	// Read registers to local registers copy using the driver
	if (fastmode) {
		error = TLE493D_AW2B6_read_regs(data->IIC_addr,
						&(data->regmap),
						TLE493D_AW2B6_Temp_REG
		);
	} else {
		error = TLE493D_AW2B6_read_regs(data->IIC_addr,
						&(data->regmap),
						TLE493D_AW2B6_XL_REG
		);
	}
	// convert data to frame if no error encountered
	if (!error) {
		regs = (uint8_t *)&(data->regmap);
		// extract data
		xtemp = (uint16_t)regs[0] << 4;
		ytemp = (uint16_t)regs[1] << 4;
		ztemp = (uint16_t)regs[2] << 4;
		ttemp = 0;
		// also append the least significant bits
		if (!fastmode) {
			xtemp |= (regs[4] & 0xF0) >> 4;
			ytemp |= (regs[4] & 0x0F);
			ztemp |= (regs[5] & 0x0F);
			ttemp = (uint16_t)((uint16_t)regs[3] << 4) | ((regs[5] & (0x3U << 6)) >> 4);
		}
		// Extend Bits for Signed Values
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
		frame->temp = ttemp;
	}

	return error;
}

int32_t TLE493D_AW2B6_WU_enable(TLE493D_data_t *data, uint16_t wu_xl, uint16_t wu_xh, uint16_t wu_yl, uint16_t wu_yh, uint16_t wu_zl, uint16_t wu_zh)
{
	int32_t err_rd, err_wr;


	// use local data if none provided
	if (NULL == data) {
		data = &(_local.data);
	}

	// prepare registers
	data->regmap.XL = wu_xl >> 4;
	data->regmap.XH = wu_xh >> 4;
	data->regmap.YL = wu_yl >> 4;
	data->regmap.YH = wu_yh >> 4;
	data->regmap.ZL = wu_zl >> 4;
	data->regmap.ZH = wu_zh >> 4;
	// set values and enable WU
	data->regmap.WU = ((wu_xh << 2) & TLE493D_AW2B6_WU_XH_MSK)
			| ((wu_xl >> 1)  & TLE493D_AW2B6_WU_XL_MSK)
			| TLE493D_AW2B6_WU_WU_ENABLE;
	data->regmap.TMode  = ((wu_yh << 2) & TLE493D_AW2B6_TMode_YH_MSK)
			    | ((wu_yl >> 1)  & TLE493D_AW2B6_TMode_YL_MSK);
	data->regmap.TPhase = ((wu_zh << 2) & TLE493D_AW2B6_TPhase_ZH_MSK)
			    | ((wu_zl >> 1)  & TLE493D_AW2B6_TPhase_ZL_MSK);
	data->regmap.Config &= ~TLE493D_AW2B6_Config_CP_MSK;
	data->regmap.Config |= TLE493D_AW2B6_get_CP_bit(data);

	// write registers XL to Config from data->regmap
	err_wr = TLE493D_AW2B6_write_reg_multi(data->IIC_addr,
						TLE493D_AW2B6_XL_REG,
						(uint8_t*)&(data->regmap) + TLE493D_AW2B6_XL_REG,
						TLE493D_AW2B6_Config_REG - TLE493D_AW2B6_XL_REG + 1
	);
	if (err_wr) {
		return err_wr;
	}

	err_rd = _I2C_read(
			data->IIC_addr,
			(uint8_t*)&(data->regmap),
			TLE493D_AW2B6_Config_REG + 1
	);

//	err_rd = TLE493D_AW2B6_read_regs(data->IIC_addr,
//					&(data->regmap),
//					TLE493D_AW2B6_Config_REG
//	);

	if (err_rd) {
		return err_rd;
	}
	if (!(data->regmap.WU & TLE493D_AW2B6_WU_WA_MSK)) {
		return TLx493D_WU_ENABLE_FAIL;
	}

	return TLx493D_OK;
}

int32_t TLE493D_AW2B6_WU_disable(TLE493D_data_t *data)
{
	// use local data if none provided
	if (NULL == data) {
		data = &(_local.data);
	}
	// disable WU
	data->regmap.WU &= ~TLE493D_AW2B6_WU_WU_MSK;
	data->regmap.WU |= TLE493D_AW2B6_WU_WU_DISABLE;
	// set parity bit
	data->regmap.Config &= TLE493D_AW2B6_Config_CP_MSK;
	data->regmap.Config |= TLE493D_AW2B6_get_CP_bit(data);

	// write registers WU and Config
	TLE493D_AW2B6_write_reg_multi(data->IIC_addr,
					TLE493D_AW2B6_WU_REG,
					(uint8_t*)&(data->regmap) + TLE493D_AW2B6_WU_REG,
					TLE493D_AW2B6_Config_REG - TLE493D_AW2B6_WU_REG + 1
	);
	// optimized reading
	// error =
	_I2C_read(
			data->IIC_addr,
			(uint8_t*)&(data->regmap),
			TLE493D_AW2B6_Config_REG + 1
	);

	// read all registers up to TPhase
//	TLE493D_AW2B6_read_regs(data->IIC_addr,
//				&(data->regmap),
//				TLE493D_AW2B6_Config_REG
//	);

	return TLx493D_OK;
}

int32_t TLE493D_AW2B6_set_IIC_address(TLE493D_data_t *data, TLE493D_address_t i2c_addr)
{
	int32_t error;


	if (NULL == data) {
		data = &(_local.data);
	}
	// clear I2C addr bits
	data->regmap.MOD1 &= ~TLE493D_AW2B6_MOD1_IICadr_MSK;
	// set new bit values
	switch (i2c_addr) {
	case TLE493D_AW2B6_I2C_A0_ADDR:
		data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_IICadr_A0;
		break;
	case TLE493D_AW2B6_I2C_A1_ADDR:
		data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_IICadr_A1;
		break;
	case TLE493D_AW2B6_I2C_A2_ADDR:
		data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_IICadr_A2;
		break;
	case TLE493D_AW2B6_I2C_A3_ADDR:
		data->regmap.MOD1 |= TLE493D_AW2B6_MOD1_IICadr_A3;
		break;
	}
	// get parity bit
	data->regmap.MOD1 &= ~TLE493D_AW2B6_MOD1_FP_MSK;
	data->regmap.MOD1 |= (TLE493D_AW2B6_get_FP_bit(data)) << TLE493D_AW2B6_MOD1_FP_POS;
	// write changes
	error = TLE493D_AW2B6_write_reg(data->IIC_addr,
					TLE493D_AW2B6_MOD1_REG,
					data->regmap.MOD1
	);
	data->IIC_addr = i2c_addr;

	return error;
}

int32_t TLE493D_AW2B6_magnetic_tmp_comp(TLE493D_data_t *data, TLE493D_magnetic_comp_t sens)
{
	int32_t error;

	// no sensor data provided, use local data
	if (NULL == data) {
		data = &(_local.data);
	}
	// clear bits
	data->regmap.Config &= ~(TLE493D_AW2B6_Config_TL_mag_MSK | TLE493D_AW2B6_Config_CP_MSK);
	// set new value
	data->regmap.Config |= (uint8_t)sens;
	// set parity
	data->regmap.Config |= TLE493D_AW2B6_get_CP_bit(data);
	// write to sensor
	error = TLE493D_AW2B6_write_reg(data->IIC_addr,
					TLE493D_AW2B6_Config_REG,
					data->regmap.Config
	);

//	{ TEST CODE }
//	error = TLE493D_AW2B6_read_regs(data->IIC_addr,
//				&(data->regmap),
//				TLE493D_AW2B6_Ver_REG
//	);
//	if (((data->regmap.Config & TLE493D_AW2B6_Config_TL_mag_MSK)) == sens) {
//		dbg_log("-- bits OK --\r\n");
//	} else {
//		dbg_log("-- bits FAIL --\r\n");
//	}
//	if (((data->regmap.Diag & TLE493D_AW2B6_Diag_CF_MSK)
//		>> TLE493D_AW2B6_Diag_CF_POS) == 1U) {
//		dbg_log("-- parity OK --\r\n");
//	} else {
//		dbg_log("-- parity FAIL --\r\n");
//	}

	return error;
}

int32_t TLE493D_AW2B6_set_high_sensitivity(TLE493D_data_t *data, bool on)
{
	int32_t error;

	// no sensor data provided, use local data
	if (NULL == data) {
		data = &(_local.data);
	}
	// clear bits
	data->regmap.Config &= ~(TLE493D_AW2B6_Config_X2_MSK | TLE493D_AW2B6_Config_CP_MSK);
	// set new value
	data->regmap.Config |= (uint8_t)(on
			? TLE493D_AW2B6_Config_X2_DOUBLE
			: TLE493D_AW2B6_Config_X2_SIMPLE);
	// set parity
	data->regmap.Config |= TLE493D_AW2B6_get_CP_bit(data);
	// write to sensor
	error = TLE493D_AW2B6_write_reg(data->IIC_addr,
					TLE493D_AW2B6_Config_REG,
					data->regmap.Config
	);

//	{ TEST CODE }
//	error = TLE493D_AW2B6_read_regs(data->IIC_addr,
//					&(data->regmap),
//					TLE493D_AW2B6_Ver_REG
//	);
//	if (((data->regmap.Config & TLE493D_AW2B6_Config_X2_MSK)) == newval) {
//		dbg_log("-- bits OK --\r\n");
//	} else {
//		dbg_log("-- bits FAIL --\r\n");
//	}
//	if (((data->regmap.Diag & TLE493D_AW2B6_Diag_CF_MSK)
//		>> TLE493D_AW2B6_Diag_CF_POS) == 1U) {
//		dbg_log("-- parity OK --\r\n");
//	} else {
//		dbg_log("-- parity FAIL --\r\n");
//	}

	return error;
}

int32_t TLE493D_AW2B6_set_angle_mode(TLE493D_data_t *data, bool on)
{
	int32_t error;


	// no sensor data provided, use local data
	if (NULL == data) {
		data = &(_local.data);
	}
	// check if the DT bit is set
	if (0U == (data->regmap.Config & TLE493D_AW2B6_Config_DT_MSK)) {
		// DT bit not set (Temperature measurement not disabled)
		return TLx493D_INVALID_SENSOR_STATE;
	}
	// clear bits
	data->regmap.Config &= ~(TLE493D_AW2B6_Config_AM_MSK | TLE493D_AW2B6_Config_CP_MSK);
	// set new value
	data->regmap.Config |= (uint8_t)(on
					? TLE493D_AW2B6_Config_AM_DISABLE_BZ_MEASURE
					: TLE493D_AW2B6_Config_AM_ENABLE_BZ_MEASURE);
	// set parity
	data->regmap.Config |= TLE493D_AW2B6_get_CP_bit(data);
	// write to sensor
	error = TLE493D_AW2B6_write_reg(data->IIC_addr,
					TLE493D_AW2B6_Config_REG,
					data->regmap.Config
	);
	if (!error) {
		error = TLE493D_AW2B6_read_regs(data->IIC_addr,
						&(data->regmap),
						TLE493D_AW2B6_Config_REG
		);
	}



//	if (((data->regmap.Config & TLE493D_AW2B6_Config_AM_MSK)) == newval) {
//		dbg_log("-- bits OK --\r\n");
//	} else {
//		dbg_log("-- bits FAIL --\r\n");
//	}
//	if (((data->regmap.Diag & TLE493D_AW2B6_Diag_CF_MSK)
//		>> TLE493D_AW2B6_Diag_CF_POS) == 1U) {
//		dbg_log("-- parity OK --\r\n");
//	} else {
//		dbg_log("-- parity FAIL --\r\n");
//	}

	return error;
}

int32_t TLE493D_AW2B6_set_temp_measure(TLE493D_data_t *data, bool on)
{
	int32_t error;

	// no sensor data provided, use local data
	if (NULL == data) {
		data = &(_local.data);
	}
	// clear bits
	data->regmap.Config &= ~(TLE493D_AW2B6_Config_DT_MSK | TLE493D_AW2B6_Config_CP_MSK);
	// set new value
	data->regmap.Config |= (uint8_t)(on
					? TLE493D_AW2B6_Config_DT_ENABLE
					: TLE493D_AW2B6_Config_DT_DISABLE);
	// set parity
	data->regmap.Config |= TLE493D_AW2B6_get_CP_bit(data);
	// write to sensor
	error = TLE493D_AW2B6_write_reg(data->IIC_addr,
					TLE493D_AW2B6_Config_REG,
					data->regmap.Config
	);
	if (!error) {
		error = TLE493D_AW2B6_read_regs(data->IIC_addr,
						&(data->regmap),
						TLE493D_AW2B6_Config_REG
		);
	}

	return error;
}

int32_t TLV493D_A1B6_set_lowpower_update_frequency(TLE493D_data_t *data, TLE493D_lp_update_freq_t freq)
{

	int32_t error;

	// no sensor data provided, use local data
	if (NULL == data) {
		data = &(_local.data);
	}
	// clear parity bit
	data->regmap.MOD1 &= ~(TLE493D_AW2B6_MOD1_FP_MSK);
	// clear frequency bits
	data->regmap.MOD2 &= ~(TLE493D_AW2B6_MOD2_PRD_MSK);
	// set new value
	data->regmap.MOD2 |= freq & TLE493D_AW2B6_MOD2_PRD_MSK;
	// set parity bit
	data->regmap.MOD1 |= TLE493D_AW2B6_get_FP_bit(data) << TLE493D_AW2B6_MOD1_FP_POS;
	// write to sensor
	error = TLE493D_AW2B6_write_reg_multi(data->IIC_addr,
						TLE493D_AW2B6_MOD1_REG,
						(uint8_t*)&(data->regmap.MOD1),
						TLE493D_AW2B6_MOD2_REG - TLE493D_AW2B6_MOD1_REG + 1
	);
	// update local register copy
	if (!error) {
		error = TLE493D_AW2B6_read_regs(data->IIC_addr,
						&(data->regmap),
						TLE493D_AW2B6_MOD2_REG
		);
	}

	return error;
}

int32_t TLV493D_A1B6_set_trigger_mode(TLE493D_data_t *data, TLE493D_Config_trigger_mode_t mode)
{

	int32_t error;

	// no sensor data provided, use local data
	if (NULL == data) {
		data = &(_local.data);
	}
	// clear bits
	data->regmap.Config &= ~(TLE493D_AW2B6_Config_TRIG_MSK | TLE493D_AW2B6_Config_CP_MSK);
	// set new value
	data->regmap.Config |= (uint8_t)mode;
	// set parity
	data->regmap.Config |= TLE493D_AW2B6_get_CP_bit(data);
	// write to sensor
	error = TLE493D_AW2B6_write_reg(data->IIC_addr,
					TLE493D_AW2B6_Config_REG,
					data->regmap.Config
	);
	if (!error) {
		error = TLE493D_AW2B6_read_regs(data->IIC_addr,
						&(data->regmap),
						TLE493D_AW2B6_Config_REG
		);
	}


	if (((data->regmap.Config & TLE493D_AW2B6_Config_TRIG_MSK)) == mode) {
		dbg_log("-- bits OK --\r\n");
	} else {
		dbg_log("-- bits FAIL --\r\n");
	}
	if (((data->regmap.Diag & TLE493D_AW2B6_Diag_CF_MSK)
		>> TLE493D_AW2B6_Diag_CF_POS) == 1U) {
		dbg_log("-- parity OK --\r\n");
	} else {
		dbg_log("-- parity FAIL --\r\n");
	}


	return error;
}

TLV493D_sensor_type_t TLE493D_get_hw_version(TLE493D_data_t *data)
{
	if (NULL == data) {
		data = &(_local.data);
	}

	// identify HW version
	if ((data->regmap.Ver & TLE493D_AW2B6_Ver_HWV_MSK)
			== TLE493D_AW2B6_Ver_HWV_B21) {
		if((data->regmap.Ver & TLE493D_AW2B6_Ver_TYPE_MSK) == 0) {
			dbg_log("HW: W2B6\r\n");
			return TLx493D_TYPE_TLE_W2B6;
		} else {
			dbg_log("HW: A2B6\r\n");
			return TLx493D_TYPE_TLE_A2B6;
		}
	}

	dbg_log("HW: UNKNOWN\r\n");
	return TLx493D_TYPE_UNKNOWN;
}

void TLE493D_AW2B6_get_data(TLE493D_data_t *dest)
{
	// sanity check
	if (NULL == dest) {
		return;
	}

	// get data
	*dest = _local.data;
}

int32_t TLE493D_AW2B6_set_data(TLE493D_data_t *src)
{
	int32_t error;

	// sanity check
	if (NULL == src) {
		return TLx493D_INVALID_ARGUMENT;
	}

	// write to sensor
	error = TLE493D_AW2B6_write_reg_multi(	_local.data.IIC_addr,
						TLE493D_AW2B6_XL_REG,
						(uint8_t*)&(src->regmap.XL),
						TLE493D_AW2B6_MOD2_REG - TLE493D_AW2B6_XL_REG + 1
		);
	if (0 != error) {
		return error;
	}
	// set data
	_local.data = *src;

	// no error
	return TLx493D_OK;
}
