
/*****************************************************************************
*
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
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*
******************************************************************************/


/*!\file interface.h
 * \brief Generic Library interface to the peripheral drivers.
 *
 * The purpose of this file is to connect microcontroller
 * dependent functions to the generic TLx493D library which is
 * microcontroller agnostic. The functions specified below
 * are needed for the normal functioning of the sensor.
 *
 * In order to interface the the TLx493D (this library) to some microcontroller,
 * several functions need to be implemented and then they should be specified in
 * this file (interface.h). For example, in order to implement reading from the
 * I2C bus, which is needed by the library, a function like the following is required:
 * \code {.c}
 * int32_t I2C_read_device_specific(uint8_t addr, uint8_t *data, uint8_t count) {
 * 	int32_t status;
 *
 * 	// Read from I2C and assign status variable
 *
 * 	return status;
 * }
 * \endcode
 * The this function should be specified below as such
 * \code {.c}
 * #define _I2C_read I2C_read_device_specific
 * \endcode
 * Please refer to the following #defines for information about the required implementation.
 *
 *
 * Note: ALL I2C functions should return a positive value of type int32_t (defined
 * in stdint.h) indicating some communication error or Zero(0) indicating
 * a successful communication. The positive return values themselves, aside from
 * indicating an error, are meaningless to the library and thus can be arbitrarily
 * chosen by the user as seen fit.
 * Negative return value are reserved and used internally by the generic library,
 * thus no function referred in this library should return a negative error as it
 * may collide with the library reserved return value! */


#ifndef SRC_TLX493D_INTERFACE_H_
#define SRC_TLX493D_INTERFACE_H_

#include <stddef.h>
#include <stdbool.h>
#include "tlx493d_platform.h"


// TODO: complete this file and remove the #error directive below
// #error "interface.h not completed"

/*! \brief Function Header: (uint8_t addr, uint8_t *data, uint8_t count)
 *
 * I2C read command must have a header precisely of type:
 * (uint8_t addr, uint8_t *data, uint8_t count):
 * \param addr The I2C address of the sensor
 * \param data The array that the function will read to
 * \param count The number of bytes the function will read
 *
 * \code{.c}
 * // =================== EXAMPLE ===================
 * // Read 10 bytes from the I2C device with address 0x63
 * // to the array data_ptr
 * // The error code will be written to error. On success
 * // it will be 0 (Zero).
 * uint8_t data_ptr[10];
 * error = _I2C_read(0x23, data_ptr, 10);
 * \endcode
 * */
#define _I2C_read tlx_port_i2c_read


/*! \brief Function Header: (uint8_t addr, const uint8_t* data, uint8_t count)
 *
 * I2C write command must have a header precisely of type:
 * (uint8_t addr, const uint8_t* data, uint8_t count), where:
 * \param addr Is the I2C address to read from data is the;
 * \param data is the array that the function will read to
 * \param count is the number of bytes the function will read
 *
 * \code{.c}
 * // =============== EXAMPLE ================
 * uint8_t payload[] = {5, 1, 2, 3};
 * // write 4 bytes from payload to the I2C device with address 0x23
 * error = _I2C_write(0x23, payload, 4);
 * \endcode
 * */
#define _I2C_write tlx_port_i2c_write


/*! \brief Function Header: (void)
 *
 * _I2C_recover should take no parameter. It will write the
 * recover address (FF) on the I2C bus
 * \code{.c}
 * // ============ EXAMPLE ==============
 * // Perform am I2C recovery command
 * _I2C_recover();
 * \endcode
 * */
#define _I2C_recover() tlx_port_i2c_recover()


/*! \brief Function Header: (void)
 *
 * _I2C_reset should take no parameter. It will write the
 * reset address (00) on the I2C bus
 * \code{.c}
 * // ============ EXAMPLE ==============
 * // Perform am I2C reset command
 * _I2C_reset();
 * \endcode
 * */
#define _I2C_reset() tlx_port_i2c_reset()


/*! \brief Function Header: (bool high)
 *
 * \param high A value of <b> true </b> will set the ADDR pin HIGH at sensor power up,
 * 	and a value of <b> false </b> will set the ADDR pin to LOW at sensor power up.
 *
 * Set the desired level on ADDR(SDA) pin and wait at least 200us.
 * Header should be of type (bool high) where:
 * _SET_ADDR_AND_WAIT(true) will set the ADDR(SDA) line to HIGH and then wait
 * for at least 200us.
 * _SET_ADDR_AND_WAIT(false) will set the ADDR(SDA) line to LOW and then wait for at least 200us.
 * Finally, the SDA line should be set back to HIGH.
 * \code{.c}
 * // =========== EXAMPLE ===========
 * // set voltage on ADDR pin to low and wait for sensor startup
 * _SET_ADDR_AND_WAIT(false);
 * \endcode
 * */
#define _SET_ADDR_AND_WAIT(high) tlx_port_set_addr_and_wait(high)


/*! \brief Function Header: (void)
 *
 * Set the pin responsible with supplying the sensor voltage to HIGH.
 * The function will be called with no arguments.
 * \code{.c}
 * // ============ EXAMPLE ==============
 * // Power Up the sensor
 * _POWER_ENABLE();
 * \endcode
 * */
#define _POWER_ENABLE() tlx_port_power_enable()


/*! \brief Function Header: (void)
 *
 * Set the pin responsible with supplying the sensor voltage to LOW.
 * The function will be called with no arguments.
 * \code{.c}
 * // ============ EXAMPLE ==============
 * // Power Down the sensor
 * _POWER_DISABLE();
 * \endcode
 * */
#define _POWER_DISABLE() tlx_port_power_disable()

/*! \brief Function Header: (void *data, uint32_t count)
 *
 * Offers a method to log a string. The header of the method
 * should be of type (void *data, uint32_t count)
 * \param data An array of unit8_t to be written
 * \param count The number of bytes to be written
 * \code{.c}
 * // ============ EXAMPLE ==============
 * // log the string 'Example'
 * _LOG_STR("Example", sizeof("Example") - 1);
 * \endcode
 * */
#define _LOG_STR(buff, len) tlx_port_log(buff, len)

#endif /* SRC_TLX493D_INTERFACE_H_ */
