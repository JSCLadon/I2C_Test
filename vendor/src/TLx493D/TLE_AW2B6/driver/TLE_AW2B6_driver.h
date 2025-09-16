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


/*! \file TLE_AW2B6_driver.h
 * \brief
 *
 * \warning
 * IMPORTANT: The TLE493D driver assumes that the 1-Byte read mode is always activated
 * before any read operation. The 2-Byte read mode is NOT supported!*/

#ifndef SRC_TLX493D_TLE_AW2B6_DRIVER_TLE_AW2B6_DRIVER_H_
#define SRC_TLX493D_TLE_AW2B6_DRIVER_TLE_AW2B6_DRIVER_H_

#include <stdint.h>
#include "TLE_AW2B6_defines.h"




/*! \brief Read register values from the sensor, starting with the register
 * at address 0 up to register <b>upto</b>
 *
 * \param addr the I2C address of the sensor;
 * \param regmap Register map structure used to store the read
 * 	registers of the sensor.
 * \param upto The reading process will start with register 0 and will continue
 * 	incrementally up to the register <b>upto</b>.
 * */
int32_t TLE493D_AW2B6_read_regs(uint8_t addr, TLE493D_regmap_t *regmap, uint8_t upto);


/*! \brief Write the <b>data</b> value to the <b>reg_addr</b> register on the sensor with the
 * I2C address <b>i2c_addr</b>.
 *
 * \param i2c_addr I2C address of the sensor.
 * \param reg_addr Address of the register that is to be written.
 * \param data Data to be written to the register.
 * */
int32_t TLE493D_AW2B6_write_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);


/*! \brief Write <b>count</b> bytes from the <b>data</b> array to the sensor with
 * the I2C address <b>addr</b>, starting with the register <b>addr_reg_start</b>.
 *
 * \param addr I2C sensor address
 * \param addr_reg_start Address of the first register to be written
 * \param data Data to be written to the registers
 * \param count Number of bytes to be written
 *  */
int32_t TLE493D_AW2B6_write_reg_multi(uint8_t addr, uint8_t addr_reg_start, uint8_t *data, uint8_t count);

#endif /* SRC_TLX493D_TLE_AW2B6_DRIVER_TLE_AW2B6_DRIVER_H_ */
