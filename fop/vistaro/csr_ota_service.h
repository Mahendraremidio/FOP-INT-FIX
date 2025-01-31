/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 *  FILE
 *      csr_ota_service.h
 *
 *  DESCRIPTION
 *     Header definitions for CSR OTA Update Application Service.
 *
 *****************************************************************************/

#ifndef _CSR_OTA_SERVICE_H
#define _CSR_OTA_SERVICE_H

/*=============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "user_config.h"

#ifdef ENABLE_OTA
/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>
#include <bt_event_types.h>

/*=============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function is used to initialise OTA service data structure. */
extern void OtaDataInit(void);

/* Handler for a READ action from the Central */
extern void OtaHandleAccessRead(GATT_ACCESS_IND_T *p_ind);

/* Handler for a WRITE action from the Central */
extern void OtaHandleAccessWrite(GATT_ACCESS_IND_T *p_ind);

/* Determine whether a handle is within the range of the OTA-upgrade service. */
extern bool OtaCheckHandleRange(uint16 handle);

/* This function indicates whether the OTA module requires the 
 * device to reset on client disconnection.
 */
extern bool OtaResetRequired(void);

#endif /* ENABLE_OTA */

#endif /* _CSR_OTA_SERVICE_H */