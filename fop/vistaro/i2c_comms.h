/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *    i2c_comms.h
 *
 *  DESCRIPTION
 *    Header file for the I2C procedures
 *
 ******************************************************************************/
#ifndef _I2C_COMMS_H
#define _I2C_COMMS_H

/*=============================================================================
 *  SDK Header Files
 *============================================================================*/
#include <pio.h>
#include <types.h>
#include <i2c.h>

/*=============================================================================
 *  Local Header Files
 *============================================================================*/
#include "user_config.h"

/*=============================================================================
 *  Public function prototypes
 *============================================================================*/
/* This function acquires the I2C bus. */
extern bool I2CAcquire(void);

/* This function releases the I2C bus. */
extern void I2CRelease(void);

/* Initialise the I2C */
extern void I2CcommsInit(void);

/* Read the specified register (1-byte) from the specified device.
 * The baseAddress parameter here is the WRITE address for the device.
 */
extern bool I2CReadRegister(uint8 base_address, uint8 reg, 
                             uint8 *p_register_value);

extern void I2C_IO_Write(uint8 data);

/* Read a contiguous sequence of registers from the specified device. */
extern bool I2CReadRegisters(uint8 base_address, uint8 start_reg, 
                              uint8 num_bytes, uint8 *buffer);

/* Write one byte of data to the specified register on the specified device. */
extern bool I2CWriteRegister(uint8 base_address, uint8 reg, 
                              uint8 register_value);
#endif /* _I2C_COMMS_H */
