/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 *  FILE
 *      gatt_service_uuids.h
 *
 *  DESCRIPTION
 *      UUID MACROs for GATT service
 *
 *****************************************************************************/

#ifndef __GATT_SERVICE_UUIDS_H__
#define __GATT_SERVICE_UUIDS_H__

/*=============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "user_config.h"

/*============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Brackets should not be used around the value of a macro. The parser 
 * which creates .c and .h files from .db file doesn't understand  brackets 
 * and will raise syntax errors.
 */

/* For UUID values, refer http://developer.bluetooth.org/gatt/services/
 * Pages/ServiceViewer.aspx?u=org.bluetooth.service.generic_attribute.xml
 */

#define UUID_GATT                                      0x1801

#define UUID_SERVICE_CHANGED                           0x2A05


#endif /* __GATT_SERVICE_UUIDS_H__ */
