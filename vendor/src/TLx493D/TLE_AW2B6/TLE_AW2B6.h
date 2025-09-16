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


/*! \file TLE_AW2B6.h
 * \brief TLE493D-A2B6/-W2B6 abstraction.
 *
 * Abstracts the basic functions of the TLE493D-A1B6/-W2B6 and offers
 * a way to store the internal state of the sensor registers.
 * */

#ifndef SRC_TLX493D_AW2B6_H_
#define SRC_TLX493D_AW2B6_H_

#include <stdint.h>
#include <stdbool.h>
#include "../TLx493D.h"
#include "driver/TLE_AW2B6_defines.h"

/*! \brief Data structure containing information about the internal
 * state of a sensor. Also used to identify a sensor on a bus. */
typedef struct {
	//! I2C address to be written on the bus
	TLE493D_address_t IIC_addr;
	//! Last known state of the internal sensor registers
	TLE493D_regmap_t regmap;
} TLE493D_data_t;


/*! \brief Initialize the sensor having the specified I2C address by reading the
 * internal registers and disabling periodic interrupt pulses.
 *
 * \param data Structure to copy the values of the internal registers to.
 * 	If data is NULL, the internal library data structure will be used
 * 	instead. This approach support only one sensor, and for a bus of
 * 	several sensors, a different data structure should be used for each
 * 	one of them.
 * \param i2c_addr The initial address of the sensor. Sensors may
 * have different fused default addresses.
 * */
int32_t TLE493D_AW2B6_init(TLE493D_data_t *data, uint8_t i2c_addr);


/*! \Brief Compute the value of the FP bit using the internal register state of the sensor */
uint8_t TLE493D_AW2B6_get_FP_bit(TLE493D_data_t *data);

/*! \Brief Compute the value of the CP bit using the internal register state of the sensor */
uint8_t TLE493D_AW2B6_get_CP_bit(TLE493D_data_t *data);


/*! \brief Set the operation mode of the sensor */
int32_t TLE493D_AW2B6_set_operation_mode(TLE493D_data_t *data, TLV493D_op_mode_t mode);


/*! \brief Read a data frame from the sensor. An ADC sampling must
 * be completed before calling this method */
int32_t TLE493D_AW2B6_read_frame(TLE493D_data_t *data, TLx493D_data_frame_t *frame);

/*! \brief Enable the Wake Up mode (available only on the -W2B6 hardware version) with the
 * provided upper and lower limits */
int32_t TLE493D_AW2B6_WU_enable(TLE493D_data_t *data, uint16_t wu_xl, uint16_t wu_xh, uint16_t wu_yl, uint16_t wu_yh, uint16_t wu_zl, uint16_t wu_zh);

/*! \brief Disable the Wake Up mode */
int32_t TLE493D_AW2B6_WU_disable(TLE493D_data_t *data);

/*! \brief Set a new I2C address for the sensor */
int32_t TLE493D_AW2B6_set_IIC_address(TLE493D_data_t *data, TLE493D_address_t i2c_addr);

/*! \brief Set the magnetic temperature compensation mode */
int32_t TLE493D_AW2B6_magnetic_tmp_comp(TLE493D_data_t *data, TLE493D_magnetic_comp_t sens);

/*! \brief Double the measurement sensitivity(when on=true). This will decrease the ADC integration speed */
int32_t TLE493D_AW2B6_set_high_sensitivity(TLE493D_data_t *data, bool on);

/*! \brief Enable/Disable angle mode. In order to enable angle mode, the temperature
 * measurement must be disabled */
int32_t TLE493D_AW2B6_set_angle_mode(TLE493D_data_t *data, bool on);

/*! \brief Enable/Disable temperature measurement */
int32_t TLE493D_AW2B6_set_temp_measure(TLE493D_data_t *data, bool on);


/*! \brief Set the update frequency while in LOW POWER Mode */
int32_t TLV493D_A1B6_set_lowpower_update_frequency(TLE493D_data_t *data, TLE493D_lp_update_freq_t freq);


/*! \brief Set trigger mode.
 * Note that the TLE493D_AW2B6_Config_TRIG_R0 mode is momentarily not
 * safe to use in this software implementation */
int32_t TLV493D_A1B6_set_trigger_mode(TLE493D_data_t *data, TLE493D_Config_trigger_mode_t mode);

/*! \brief Return hardware version of the TLE493D */
TLV493D_sensor_type_t TLE493D_get_hw_version(TLE493D_data_t *data);

/*! \brief Copy the data stored in the library to the dest structure */
void TLE493D_AW2B6_get_data(TLE493D_data_t *dest);

/*! \brief Copy the data from src to the library and the sensor */
int32_t TLE493D_AW2B6_set_data(TLE493D_data_t *src);

#endif /* SRC_TLX493D_AW2B6_H_ */
