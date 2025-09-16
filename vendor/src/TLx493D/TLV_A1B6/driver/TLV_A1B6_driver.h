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


/*! \file TLV_A1B6_driver.h
 * \brief Low level driver for the TLV493D-A1B6.
 *
 * It simplifies the read
 * and write operations when working with the internal
 * registers of the sensor and also help prevent changes to
 * the reserved data registers.
 * */

#ifndef SRC_TLX493D_TLV_A1B6_DRIVER_TLV_A1B6_DRIVER_H_
#define SRC_TLX493D_TLV_A1B6_DRIVER_TLV_A1B6_DRIVER_H_

#include <stdint.h>
#include "TLV_A1B6_defines.h"
#include "src/TLx493D/interface.h"

/*! \brief Data structure describing the TLV493D read registers. */
typedef struct {
	uint8_t Bx, By, Bz;
	uint8_t Temp;
	uint8_t Bx2, Bz2;
	uint8_t Temp2;
	uint8_t FactSet1, FactSet2, FactSet3;
} TLV493D_regmap_read_t;

/*! \brief Data structure describing the TLV493D write registers. */
typedef struct {
	uint8_t Res;
	uint8_t MOD1;
	uint8_t Res2;
	uint8_t MOD2;
} TLV493D_regmap_write_t;


/*! \brief Read register values from the sensor, starting with the register
 * at address 0 up to register <b>upto</b>.
 *
 * \param addr the I2C address of the sensor;
 * \param regmap register map read structure used to store the read
 * 	registers of the sensor.
 * \param upto The reading process will start with register 0 and will continue
 * incrementally up to the register <b>upto</b>
 *
 * \return Error code.
 * */
int32_t TLV493D_A1B6_read_regs(uint8_t addr, TLV493D_regmap_read_t *regmap, uint8_t upto);

/*! \brief Write the register data from regmap to the sensor registers.
 *
 * If the <b>regmap_check</b> pointer points to a valid structure (is not NULL),
 * the reserved registers data from the <b>regmap</b> will be overwritten with the
 * reserved data from <b>regmap_check</b>. This ensures that the reserved data read
 * from the sensor is properly written back to the sensor. This overwrite will need
 * to happend only once, as the corrections  will be stored inside <b>regmap</b>.
 *
 * \return Error code.
 * */
int32_t TLV493D_A1B6_write_regs(uint8_t addr, TLV493D_regmap_write_t *regmap, const TLV493D_regmap_read_t *regmap_check);

#endif /* SRC_TLX493D_TLV_A1B6_DRIVER_TLV_A1B6_DRIVER_H_ */
