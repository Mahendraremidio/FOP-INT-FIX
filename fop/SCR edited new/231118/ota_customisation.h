/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 *  FILE
 *      ota_customisation.h
 *
 *  DESCRIPTION
 *      Customisation requirements for the CSR OTAU functionality.
 *
 *****************************************************************************/

#ifndef __OTA_CUSTOMISATION_H__
#define __OTA_CUSTOMISATION_H__

/*=============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "user_config.h"

#ifdef ENABLE_OTA
/* ** CUSTOMISATION **
 * The following header file names may need altering to match your application.
 */

#include "serial_gatt.h"
#include "app_gatt_db.h"
#include "serial_server.h"

/*=============================================================================*
 *  Private Definitions
 *============================================================================*/

/* ** CUSTOMISATION **
 * Change these definitions to match your application.
 */
#define CONNECTION_CID      g_app_data.st_ucid
#define IS_BONDED           g_app_data.bonded
#define CONN_CENTRAL_ADDR   g_app_data.con_bd_addr
#define CONNECTION_IRK      g_app_data.irk
#define LINK_DIVERSIFIER    g_app_data.diversifier

/* Uncomment the following if this application is using a static random address.
 */
/*#define USE_STATIC_RANDOM_ADDRESS*/

/* Uncomment the following if this application is using a resolvable random 
 * address.
 */
/*#define USE_RESOLVABLE_RANDOM_ADDRESS*/

/* Do not enable both addressing types */
#if defined(USE_STATIC_RANDOM_ADDRESS) && defined(USE_RESOLVABLE_RANDOM_ADDRESS)
#error "Choose just one addressing type"
#endif /* USE_STATIC_RANDOM_ADDRESS && USE_RESOLVABLE_RANDOM_ADDRESS */

#endif /* ENABLE_OTA */

#endif /* __OTA_CUSTOMISATION_H__ */

