/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *     serial_service_uuids.h
 *
 *  DESCRIPTION
 *      UUID MACROs for custom serial service
 *
 ******************************************************************************/

#ifndef __SERIAL_SERVICE_UUIDS_H__
#define __SERIAL_SERVICE_UUIDS_H__

/*============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Brackets should not be used around the value of a macro. The parser
 * which creates .c and .h files from .db file doesn't understand  brackets
 * and will raise syntax errors.
 */

#define UUID_SERIAL_SERVICE            0x00006500D10211E19B2300025B00A5A5

/*#define UUID_SERIAL_DATA_TRANSFER      0x00005501D10211E19B2300025B00A5A5*/
  #define UUID_SERIAL_DATA_TRANSFER      0x00006501D10211E19B2300025B00A5A5

/*Split the 128bit UUID into 8-bit*/
#define UUID_SERIAL_SERVICE_1          0x00
#define UUID_SERIAL_SERVICE_2          0x00
#define UUID_SERIAL_SERVICE_3          0x65
#define UUID_SERIAL_SERVICE_4          0x00
#define UUID_SERIAL_SERVICE_5          0xd1
#define UUID_SERIAL_SERVICE_6          0x02
#define UUID_SERIAL_SERVICE_7          0x11
#define UUID_SERIAL_SERVICE_8          0xe1
#define UUID_SERIAL_SERVICE_9          0x9b
#define UUID_SERIAL_SERVICE_10         0x23
#define UUID_SERIAL_SERVICE_11         0x00
#define UUID_SERIAL_SERVICE_12         0x02
#define UUID_SERIAL_SERVICE_13         0x5b
#define UUID_SERIAL_SERVICE_14         0x00
#define UUID_SERIAL_SERVICE_15         0xa5
#define UUID_SERIAL_SERVICE_16         0xa5



#endif /* __SERIAL_SERVICE_UUIDS_H__ */

