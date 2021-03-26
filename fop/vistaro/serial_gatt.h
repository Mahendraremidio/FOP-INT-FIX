/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *     serial_gatt.h
 *
 *  DESCRIPTION
 *      Header definitions for common application attributes
 *
 ******************************************************************************/
#ifndef __SERIAL_ACCESS_H__
#define __SERIAL_ACCESS_H__

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>
#include <time.h>
#include <gatt.h>
#include <gap_types.h>

/*============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Timeout values for fast and slow advertisements */
/* Fast advertisment timeout */
#define FAST_CONNECTION_ADVERT_TIMEOUT_VALUE     \
                                        (30 * SECOND)

/* Slow advertisment timeout */
#define SLOW_CONNECTION_ADVERT_TIMEOUT_VALUE     \
                                        (1  * MINUTE)

/*============================================================================*
 *  Public Data Types
 *============================================================================*/

/* Application states */
typedef enum
{
    /* Application initial state */
    app_state_init = 0,

    /* Application is performing fast undirected advertisements */
    app_state_fast_advertising,

    /* Application is performing slow undirected advertisements */
    app_state_slow_advertising,

    /* Application is performing directed advertisements */
    app_state_directed_advertising,

    /* Connection has been established with the host */
    app_state_connected,

    /* Disconnection initiated by the application */
    app_state_disconnecting,

    /* Application is neither advertising nor connected to a host */
    app_state_idle

} app_state;


/* GATT Client Characteristic Configuration Value [Ref GATT spec, 3.3.3.3] */
typedef enum
{
    gatt_client_config_none            = 0x0000,
    gatt_client_config_notification    = 0x0001,
    gatt_client_config_indication      = 0x0002,
    gatt_client_config_reserved        = 0xFFF4

} gatt_client_config;

/*  Application defined panic codes */
typedef enum
{
    /* Failure while setting advertisement parameters */
    app_panic_set_advert_params,

    /* Failure while setting advertisement data */
    app_panic_set_advert_data,

    /* Failure while setting scan response data */
    app_panic_set_scan_rsp_data,

    /* Failure while registering GATT DB with firmware */
    app_panic_db_registration,

    /* Failure while reading NVM */
    app_panic_nvm_read,

    /* Failure while writing NVM */
    app_panic_nvm_write,

    /* Failure while reading Tx Power Level */
    app_panic_read_tx_pwr_level,

    /* Failure while deleting device from whitelist */
    app_panic_delete_whitelist,

    /* Failure while adding device to whitelist */
    app_panic_add_whitelist,

    /* Failure while triggering connection parameter update procedure */
    app_panic_con_param_update,

    /* Event received in an unexpected application state */
    app_panic_invalid_state,

    /* Unexpected beep type */
    app_panic_unexpected_beep_type,

    /* Failure while erasing NVM */
    app_panic_nvm_erase
} app_panic_code;

/*============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Invalid UCID indicating we are not currently connected */
#define GATT_INVALID_UCID               (0xFFFF)

/* Invalid Attribute Handle */
#define INVALID_ATT_HANDLE              (0x0000)

/* AD Type for Appearance */
#define AD_TYPE_APPEARANCE              (0x19)

/* Maximum Length of Device Name
 * Note: Do not increase device name length beyond (DEFAULT_ATT_MTU - 3 = 20)
 * octets as the GAP service doesn't support handling of Prepare write and
 * Execute write procedures.
 */
#define DEVICE_NAME_MAX_LENGTH          (20)

/* The following macro definition should be included only if a user wants the
 * application to have a static random address.
 */
/* #define USE_STATIC_RANDOM_ADDRESS */

/* Timer value for remote device to re-encrypt the link using old keys */
#define BONDING_CHANCE_TIMER            (30 *  SECOND)

/* Maximum length of the data to be sent to the peer device */
#define SERIAL_RX_DATA_LENGTH           (20)

#ifdef ENABLE_OTA

/* This error codes should be returned when a remote connected device writes a
 * configuration which the application does not support.
 */
#define gatt_status_desc_improper_config   \
                                        (STATUS_GROUP_GATT+ 0xFD)

/* Highest possible handle for ATT database. */
#define ATT_HIGHEST_POSSIBLE_HANDLE     (0xFFFF)

/* Extract 3rd byte (bits 16-23) of a uint24 variable */
#define THIRD_BYTE(x)                   \
                                       ((uint8)((((uint24)x) >> 16) & 0x0000ff))

/* Extract 2rd byte (bits 8-15) of a uint24 variable */
#define SECOND_BYTE(x)                  \
                                       ((uint8)((((uint24)x) >> 8) & 0x0000ff))

/* Extract least significant byte (bits 0-7) of a uint24 variable */
#define FIRST_BYTE(x)                  ((uint8)(((uint24)x) & 0x0000ff))

/* Convert a word-count to bytes */
#define WORDS_TO_BYTES(_w_)            (_w_ << 1)

/* Convert bytes to word-count*/
#define BYTES_TO_WORDS(_b_)            (((_b_) + 1) >> 1)

/* The Maximum Transmission Unit length supported by this device. */
#define ATT_MTU                         23

/* The maximum user data that can be carried in each radio packet.
 * MTU minus 3 bytes header
 */
#define MAX_DATA_LENGTH                (ATT_MTU-3)

#endif /* ENABLE_OTA */

/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* Handle read operations on attributes maintained by the application */
extern void HandleAccessRead(GATT_ACCESS_IND_T *p_ind);

/* Handle write operations on attributes maintained by the application. */
extern void HandleAccessWrite(GATT_ACCESS_IND_T *p_ind);

/* Start undirected advertisements and move to ADVERTISING state */
extern void GattStartAdverts(bool fast_connection,
                             gap_mode_connect connect_mode);

/* Stop on-going advertisements */
extern void GattStopAdverts(void);

/* Prepare the list of supported 128-bit service UUIDs to be added to
 * Advertisement data
 */
extern uint16 GetSupported128BitUUIDServiceList(uint8 *p_service_uuid_ad);

/* Check if the address is resolvable random or not */
extern bool GattIsAddressResolvableRandom(TYPED_BD_ADDR_T *p_addr);

/* Trigger fast advertisements */
extern void GattTriggerFastAdverts(TYPED_BD_ADDR_T *p_addr);

/* Initialise the application GATT data. */
extern void InitGattData(void);

#endif /* __SERIAL_ACCESS_H__ */
