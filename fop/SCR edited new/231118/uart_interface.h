/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *     uart_interface.h
 *
 *  DESCRIPTION
 *      This file contains prototypes for accessing UART functionality.
 *
 ******************************************************************************/
#ifndef __UART_INTERFACE_H__
#define __UART_INTERFACE_H__

/*============================================================================*
 *  SDK Header includes
 *============================================================================*/

#include <types.h>
#include <sys_events.h>
#include <sleep.h>

/*============================================================================*
 *  Public definitions
 *============================================================================*/

/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function is called to initialise the UART.*/
extern void InitUart(void);

/* This function reads and process the next data from the byte queue.*/
extern void ProcessRxData(void);

/* Sends the data to UART. */
extern void SendDataToUart(uint8 *data, uint16 size);

/* Configures the UART either to high or low baud rate. */
extern void ConfigureUart(bool bHigh);

/* Function that updates the last sent BLE notification status, whether success
 * or failed.
 */
extern void SetLastNotificationStatus(bool bsuccess);

#endif /* __UART_INTERFACE_H__ */

