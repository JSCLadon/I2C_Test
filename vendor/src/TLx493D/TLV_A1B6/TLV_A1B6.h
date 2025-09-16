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


/*! \file TLV_A1B6.h
 * \brief TLV493D-A1B6 abstraction.
 *
 * Abstracts the basic functions of the TLV493D-A1B6 and offers
 * a way to store the internal state of the sensor registers.
 * */

#ifndef SRC_TLX493D_A1B6_H_
#define SRC_TLX493D_A1B6_H_

#include "../TLx493D.h"
#include "driver/TLV_A1B6_driver.h"

/*! \brief I2C addresses supported by the TLV493D-A1B6
 * sensor.
 *
 * The left side addresses from the define
 * names (3E, 36, 1E, 16) are relevant when the sensor
 * is powered up with the ADDR pin LOW. The right side
 * addresses can be used when the sensor is powered up
 * with the ADDR pin HIGH.
 * All values are in hexadecimal representation. */
typedef enum {
	TLV493D_A1B6_ADDR_3E_BC,
	TLV493D_A1B6_ADDR_36_B4,
	TLV493D_A1B6_ADDR_1E_9C,
	TLV493D_A1B6_ADDR_16_94
} TLV493D_address_t;


/*! \brief Data structure containing information about the internal
 * state of a sensor. Also used to identify a sensor on a bus. */
typedef struct {
	//! I2C address to be written on the bus for sensor addressing
	uint8_t IIC_addr;
	//! Last frame value from the sensor ADC used to detect a stuck ADC
	uint8_t frame_count;
	//! Type of I2C address (addr bit unspecified)
	TLV493D_address_t addr_type;
	//! Address bit, representing state of ADDR line at power up
	bool ADDR_high;
	//! Last known state of the Read registers
	TLV493D_regmap_read_t regmap_read;
	//! Last known state of the Write registers
	TLV493D_regmap_write_t regmap_write;
} TLV493D_data_t;


/*! \brief Initialize the sensor.
 *
 *
 * \param data parameter is optional (can be replaced with NULL) and specifies a
 * 	data structure that should store the state of the sensor. If no data structure is
 * 	sepcifiec, an internal data structure will be used. This parameter should be used in
 * 	a bus configuration to easily identify sensors and also to manually inspect the
 * 	internal state of the sensor.
 * \param ADDR_high indicates the level of ADDR at the time the sensor was powered up.
 * 	ADDR_high=true indicates that the sensor was powered up with ADDR=HIGH
 * 	ADDR_high=false indicates that the sensor was powered up with ADDR=LOW
 * \param addr_type indicates the desired address after initialization while keeping in mind the
 * value of ADDR_high and the ADDR pin logic value at startup.
 *
 *
 * 	\code{.c}
 * 	// ========================= Example =============================
 * 	// NULL -> store sensor information internally inside the library
 * 	//	This mode supports only one sensor at a time
 * 	// true -> At startup the ADDR pin was HIGH so the internal sensor
 * 	// address pin is set to 1
 * 	// TLV493D_A1B6_ADDR_1E_9C -> the desired sensor address is either
 * 	// 1E or 9C. Since the ADDR pin was HIGH at startup, the address will
 * 	// always be 9C
 * 	TLV493D_A1B6_init(NULL, true, TLV493D_A1B6_ADDR_1E_9C);
 * 	\endcode
 * 	*/
int32_t TLV493D_A1B6_init(TLV493D_data_t *data, bool ADDR_high, TLV493D_address_t addr_type);

/*! \brief Hard reset the sensor by executing a power cycle and reinitialize using
 * the settings from the data structure. Will only set the address.
 * \param data Sensor data structure. By passing NULL, local data will be used.
 * */
void TLV493D_A1B6_hard_reset_reconfigure(TLV493D_data_t *data);

/*! \brief Read the registers of the TLx493D sensor and create a data frame */
int32_t TLV493D_A1B6_read_frame(TLV493D_data_t *data, TLx493D_data_frame_t *frame);

/*! \brief Change the operation mode of the sensor */
int32_t TLV493D_A1B6_set_operation_mode(TLV493D_data_t *data, TLV493D_op_mode_t mode);

/*! \brief Enable or disable the temperature measurement */
int32_t TLV493D_A1B6_set_temp_measure(TLV493D_data_t *data, bool enabled);

/*! \brief Enable or disable the parity test */
int32_t TLV493D_A1B6_set_parity_test(TLV493D_data_t *data, bool enabled);

/*! \brief Set a new I2C address for the sensor, considering the ADDR pin level at startup */
int32_t TLV493D_A1B6_set_IIC_address(TLV493D_data_t *data, TLV493D_address_t new_addr_type);

/*! \brief Copy the data stored in the library to the dest structure */
void TLV493D_A1B6_get_data(TLV493D_data_t *dest);

/*! \brief Copy the data from src to the library */
int32_t TLV493D_A1B6_set_data(TLV493D_data_t *src);

#endif /* SRC_TLX493D_A1B6_H_ */
