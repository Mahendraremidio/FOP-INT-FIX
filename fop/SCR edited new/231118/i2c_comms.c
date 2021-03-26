/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 *  FILE
 *    i2c_comms.c
 *
 *  DESCRIPTION
 *    This file defines different I2C procedures.
 *
 ******************************************************************************/

/*=============================================================================
 *  SDK Header Files
 *============================================================================*/
#include <pio.h>
#include <types.h>
#include <i2c.h>

/*=============================================================================
 *  Local Header Files
 *============================================================================*/
#include "i2c_comms.h"
#include "user_config.h"
#include <debug.h>

/*=============================================================================
 *  Private Data Type
 *============================================================================*/
/* The following enum will tell if the I2C bus is in Released state or in 
 * Acquired state.
 */
typedef enum
{
    i2c_bus_released = 0x00,
    i2c_bus_acquired = 0x01
} i2c_bus_status;

/*=============================================================================
 *  Private Data
 *============================================================================*/
/* Flag for the I2C bus status */
i2c_bus_status bus_I2C_status = i2c_bus_released;

bool i2c_initialised = FALSE;

/*=============================================================================
 *  Public function definitions
 *============================================================================*/
/*----------------------------------------------------------------------------*
 *  NAME
 *      I2CAcquire
 *
 *  DESCRIPTION
 *      This function acquires the I2C bus
 *
 *  RETURNS
 *      Bool- TRUE if successful.
 *
 *----------------------------------------------------------------------------*/
extern bool I2CAcquire(void)
{
    /* If I2C bus is in Released state, acquire it and return success */
    if(bus_I2C_status == i2c_bus_released)
    {
        bus_I2C_status = i2c_bus_acquired;
        return TRUE;
    }
    else /* Else return Failure */
    {
        return FALSE;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      I2CRelease
 *
 *  DESCRIPTION
 *      This function releases the I2C bus
 *
 *  RETURNS
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
extern void I2CRelease(void)
{
    bus_I2C_status = i2c_bus_released;
    I2cReset();
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      I2CcommsInit
 *
 *  DESCRIPTION
 *      This function initialises the I2C
 *
 *  RETURNS
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
extern void I2CcommsInit(void)
{
    /* Initialise I2C if it is not already initialised */
    if(!i2c_initialised)
    {
        i2c_initialised = TRUE;
        PioSetModes(    ((0x01L << I2C_SDA_PIO) | (0x01L << I2C_SCL_PIO)), 
                                                pio_mode_user);

        /* Configure the PIOs as Input */
        PioSetDirs(     ((0x01L << I2C_SDA_PIO) | (0x01L << I2C_SCL_PIO)), 
                                                FALSE);
        PioSetPullModes(((0x01L << I2C_SDA_PIO) | (0x01L << I2C_SCL_PIO)), 
                                                pio_mode_strong_pull_down); 
        PioSetEventMask(((0x01L << I2C_SDA_PIO) | (0x01L << I2C_SCL_PIO)), 
                                                pio_event_mode_disable);

        /* Configure the I2C controller */
        I2cInit(I2C_SDA_PIO,
                I2C_SCL_PIO,
                I2C_POWER_PIO_UNDEFINED,
                pio_i2c_pull_mode_strong_pull_up);

        /* Configure pull mode of the I2C pins */
       /* PioSetI2CPullMode(pio_i2c_pull_mode_strong_pull_up);*/

        /* Configure the I2C clock */
        I2cConfigClock(I2C_SCL_400KBPS_HIGH_PERIOD, I2C_SCL_400KBPS_LOW_PERIOD);

        /* Enable the I2C controller */
        I2cEnable(TRUE);
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      I2CReadRegister
 *
 *  DESCRIPTION
 *      This function reads the specified register from the specified device
 *
 *  RETURNS
 *      TRUE if successful
 *
 *----------------------------------------------------------------------------*/
extern bool I2CReadRegister(uint8 base_address, uint8 reg, 
                             uint8 *p_register_value)
{
    bool success = TRUE;
    /*uint8 ch;*/
    
   I2cRawCommand(i2c_cmd_send_start,TRUE,0);                      
   I2cRawWriteByte(base_address<<1);         
   I2cRawCommand(i2c_cmd_send_restart,TRUE,0);                        
   I2cRawWriteByte(base_address<<1);        
   I2cRawCommand(i2c_cmd_wait_ack,TRUE,1 * MILLISECOND);                   
   I2cRawWriteByte(reg);                      
   I2cRawCommand(i2c_cmd_wait_ack,TRUE,1 * MILLISECOND);                     
   I2cRawCommand(i2c_cmd_send_restart,TRUE,0);                         
   I2cRawWriteByte((base_address<<1)|0x01);   
   I2cRawCommand(i2c_cmd_wait_ack,TRUE,1* MILLISECOND); 
   I2cRawReadByte(p_register_value); 
   I2cRawCommand(i2c_cmd_send_nack,TRUE,1 * MILLISECOND);    
   I2cRawStop(TRUE);    
   
   I2cRawComplete(2 * MILLISECOND);
   I2cRawTerminate();
    
   /* ch = *p_register_value;
    DebugWriteChar(ch);*/
    
    
    return success;
}


extern void I2C_IO_Write(uint8 data)
{
    
    I2cRawCommand(i2c_cmd_send_start,TRUE,0);
    I2cRawWriteByte(0x42);
    I2cRawCommand(i2c_cmd_wait_ack,TRUE,1* MILLISECOND); 
    I2cRawWriteByte(data);
    I2cRawCommand(i2c_cmd_wait_ack,TRUE,1* MILLISECOND);
    I2cRawStop(TRUE); 
    I2cRawComplete(2 * MILLISECOND);
    I2cRawTerminate();
    
    
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      I2CReadRegisters
 *
 *  DESCRIPTION
 *      This function reads a contiguous sequence of registers from the 
 *      specified device
 *
 *  RETURNS
 *      TRUE if successful
 *
 *----------------------------------------------------------------------------*/
extern bool I2CReadRegisters(uint8 base_address, uint8 start_reg, 
                              uint8 num_bytes, uint8 *p_buffer)
{
    bool success = FALSE;
    uint8 *p_write_pos;

    /* We assume that the supplied buffer is big enough. */
    if( (I2cRawStart(TRUE)                     == sys_status_success) &&
        (I2cRawWriteByte(base_address)         == sys_status_success) &&
        (I2cRawWaitAck(TRUE)                   == sys_status_success) &&
        (I2cRawWriteByte(start_reg)            == sys_status_success) &&
        (I2cRawWaitAck(TRUE)                   == sys_status_success) &&
        (I2cRawRestart(TRUE)                   == sys_status_success) &&
        (I2cRawWriteByte((base_address | 0x1)) == sys_status_success) &&
        (I2cRawWaitAck(TRUE)                   == sys_status_success))
    {
        for(p_write_pos = p_buffer;
            num_bytes > 1;
            num_bytes--)
        {
            I2cRawReadByte(p_write_pos++); /* Assume this works - */
            I2cRawSendAck(TRUE);        /* everything up to this point has. */
        }
        /* This is the last byte */
        I2cRawReadByte(p_write_pos++);
        I2cRawSendNack(TRUE);
        
        success = (I2cRawStop(TRUE) == sys_status_success);
    }
    I2cRawComplete(1 * MILLISECOND);
    I2cRawTerminate();
    
    return success;
}


/*-----------------------------------------------------------------------------*
 *  NAME
 *      I2CWriteRegister
 *
 *  DESCRIPTION
 *      This function writes one byte of data to a specified register on the 
 *      specified device
 *
 *  RETURNS
 *      TRUE if successful
 *
 *----------------------------------------------------------------------------*/
extern bool I2CWriteRegister(uint8 base_address, uint8 reg, 
                              uint8 register_value)
{
    bool success;
    
    success = ( (I2cRawStart(TRUE)               == sys_status_success) &&
                (I2cRawWriteByte(base_address)   == sys_status_success) &&
                (I2cRawWaitAck(TRUE)             == sys_status_success) &&
                (I2cRawWriteByte(reg)            == sys_status_success) &&
                (I2cRawWaitAck(TRUE)             == sys_status_success) &&
                (I2cRawWriteByte(register_value) == sys_status_success) &&
                (I2cRawWaitAck(TRUE)             == sys_status_success) &&
                (I2cRawStop(TRUE)                == sys_status_success));

    I2cRawComplete(1 * MILLISECOND);
    I2cRawTerminate();
    
    return success;
}
