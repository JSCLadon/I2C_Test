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



#ifndef _TLV493D__H_
#define _TLV493D__H_

/*!
 * \file TLx493D.h
 * \brief TLx 3D Hall Sensor Family Abstraction
 *
 * This file presents and abstraction for the sensors of the TLx493D
 * family, offering basic functionality like changing the operation
 * mode of the sensor, or reading a data frame.
 * */


#include <stdbool.h>
#include <stdint.h>

/*!
 * \brief Generic data frame, common to all supported hardware version
 * */
typedef struct {
	//! Magnetic field intensity raw value on the X axis
	int16_t x;
	//! Magnetic field intensity raw value on the Y axis
	int16_t y;
	//! Magnetic field intensity raw value on the Z axis
	int16_t z;
	//! Raw Temperature value
	int16_t temp;
} TLx493D_data_frame_t;

/*! \brief Error codes returned by the TLx493D library */
enum {
	//! No error encountered
	TLx493D_OK = 0,
	//! Function called with invalid argument
	TLx493D_INVALID_ARGUMENT = -1,
	//! The returned frame is invalid and should be discarded
	TLx493D_INVALID_FRAME = -2,
	//! The called method has not been implemented yet
	TLx493D_NOT_IMPLEMENETED = -3,
	//! One or mode sensor registers are set incorrectly
	TLx493D_INVALID_SENSOR_STATE = -4,
	//!The WU feature failed to activate; unknown error
	TLx493D_WU_ENABLE_FAIL 	= -5,
};


/*! \brief Type of sensor on board */
typedef enum {
	TLx493D_TYPE_UNKNOWN,
	TLx493D_TYPE_TLV_A1B6,
	TLx493D_TYPE_TLE_A2B6,
	TLx493D_TYPE_TLE_W2B6,
	TLx493D_TYPE_TLI_W2BW
} TLV493D_sensor_type_t;

/*! \brief Operating Mode*/
typedef enum {
	TLx493D_OP_MODE_NOT_INITIALIZED,
	TLx493D_OP_MODE_POWER_DOWN,
	TLx493D_OP_MODE_MCM,
	TLx493D_OP_MODE_FAST,
	TLx493D_OP_MODE_LOW_POWER,
	TLx493D_OP_MODE_ULTRA_LOW_POWER,
} TLV493D_op_mode_t;



/*! \brief Detect and initialize the connected sensor.
 *
 *  Automatically detect the sensor hardware version and call the appropriate
 * initialization sequences. Must be called prior to any other call to
 * a TLx493D_* method.
 * \return Error code.*/
int32_t TLx493D_init(void);

/*! \brief Return the type of sensor present on the board. */
TLV493D_sensor_type_t TLx493D_get_sensor_type(void);

/*! \brief Set the operation mode of the sensors, if supported.
 * \return Error code.*/
int32_t TLx493D_set_operation_mode(TLV493D_op_mode_t mode);

/*! \brief Get the operation mode of the sensors */
TLV493D_op_mode_t TLx493D_get_operation_mode();

/*! \brief Read a data frame from the sensor
 * \return Error code. */
int32_t TLx493D_read_frame(TLx493D_data_frame_t *frame);


/*! \brief Compute the EVEN parity of a byte of data
 *
 *  \return Even parity of the data, either the value 0 or 1. */
uint8_t MISC_get_parity(uint8_t data);

#endif //_TLV493D__H_
