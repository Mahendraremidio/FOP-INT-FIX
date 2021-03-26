/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *     nvm_access.h
 *
 *  DESCRIPTION
 *      Header definitions for NVM usage.
 *
 ******************************************************************************/
#ifndef __NVM_ACCESS_H__
#define __NVM_ACCESS_H__

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>

/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function is used to perform the actions necessary to save power on
 * NVM once read/write operations are done.
 */
extern void Nvm_Disable(void);

/* Function to read words from the NVM Store after preparing the NVM to be
 * readable.After the read operation, perform the actions necessary to save
 * power on NVM.
 */
extern void Nvm_Read(uint16 *buffer, uint16 length, uint16 offset);

/* Function to write words to the NVM Store after preparing the NVM to be
 * writable.After the write operation, perform the actions necessary to save
 * power on NVM.
 */
extern void Nvm_Write(uint16 *buffer, uint16 length, uint16 offset);

#ifdef NVM_TYPE_FLASH
/* Erases the NVM memory.*/
extern void Nvm_Erase(void);
#endif /* NVM_TYPE_FLASH */

#endif /* __NVM_ACCESS_H__ */
