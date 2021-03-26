/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *     serial_service.h
 *
 *  DESCRIPTION
 *      Header definitions for the Serial Service
 *
 ******************************************************************************/
#ifndef __SERIAL_SERVICE_H__
#define __SERIAL_SERVICE_H__

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>
#include <gatt.h>


/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function is used to initialise serial service data structure */
extern void SerialInitChipReset(void);

/* This function handles write operation on serial service attributes
 * maintained by the application.and responds with the GATT_ACCESS_RSP
 * message.
 */
extern void SerialHandleAccessWrite(GATT_ACCESS_IND_T *p_ind);

/* This function is used to read serial service specific data stored in
 * NVM.
 */
extern void SerialReadDataFromNVM(bool bonded,uint16 *p_offset);

/* This function is used to check if the handle belongs to the serial
 * service.
 */
extern bool SerialCheckHandleRange(uint16 handle);

/* Sends the serial service notification. */
extern bool SerialSendNotification(uint8 *data, uint16 size);

/* Function that configures the UART port. */
extern void SerialConfigureUart(bool bhigh);

/* This function is used by application to notify bonding status to 
 * Serial service.
 */
extern void SerialBondingNotify(void);

/* This function handles read operation on serial service attributes
 * maintained by the application.and responds with the GATT_ACCESS_RSP
 * message.
 */
extern void SerialHandleAccessRead(GATT_ACCESS_IND_T *p_ind);

#ifdef NVM_TYPE_FLASH
/* This function writes HR service data in NVM */
extern void WriteSerialServiceDataInNvm(void);
#endif /* NVM_TYPE_FLASH */

#endif /* __SERIAL_SERVICE_H__ */
