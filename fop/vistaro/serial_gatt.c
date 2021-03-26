/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 *  FILE
 *      serial_gatt.c
 *
 *  DESCRIPTION
 *      GATT related routines implementation
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <ls_app_if.h>
#include <gap_app_if.h>
#include <ls_err.h>
#include <ls_types.h>
#include <panic.h>
#include <gatt.h>
#include <gatt_uuid.h>
#include <timer.h>
#include <mem.h>

/*============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "serial_server.h"
#include "app_gatt_db.h"
#include "serial_gatt.h"
#include "appearance.h"

#include "battery_service.h"
#include "battery_uuids.h"
#include "dev_info_uuids.h"
#include "dev_info_service.h"
#include "serial_service_uuids.h"
#include "serial_service.h"
#include "gatt_service.h"
#include "csr_ota_service.h"
#include "gap_conn_params.h"

/*============================================================================*
 *  Private Definitions
 *============================================================================*/

/* This constant defines an array that is large enough to hold the advertisement
 * data.
 */
#define MAX_ADV_DATA_LEN                                  (31)

/* Acceptable shortened device name length that can be sent in advertisement
 * data
 */
#define SHORTENED_DEV_NAME_LEN                            (8)

/* Length of Tx Power prefixed with 'Tx Power' AD Type */
#define TX_POWER_VALUE_LENGTH                             (2)

/*============================================================================*
 *  Private Data types
 *============================================================================*/

/* GATT data structure */
typedef struct _APP_GATT_DATA_T
{
    /* Value for which advertisement timer needs to be started. */
    uint32                     advert_timer_value;

} APP_GATT_DATA_T;

/*============================================================================*
 *  Private Data
 *============================================================================*/

/* Application GATT data instance */
static APP_GATT_DATA_T g_gatt_data;

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

/* Set advertisement parameters */
static void gattSetAdvertParams(bool fast_connection,
                                gap_mode_connect connect_mode);

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      gattSetAdvertParams
 *
 *  DESCRIPTION
 *      This function is used to set advertisement parameters.
 *
 *  PARAMETERS
 *      p_addr [in]             Bonded host address
 *      fast_connection [in]    TRUE:  Fast advertisements
 *                              FALSE: Slow advertisements
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void gattSetAdvertParams(bool fast_connection,
                                gap_mode_connect connect_mode)
{
    uint8 advert_data[MAX_ADV_DATA_LEN];/* Advertisement packet */
    uint16 length;                      /* Length of advertisement packet */
    /* Advertisement interval, microseconds */
    uint32 adv_interval_min = RP_ADVERTISING_INTERVAL_MIN;
    uint32 adv_interval_max = RP_ADVERTISING_INTERVAL_MAX;
    
    TYPED_BD_ADDR_T temp_addr;


    /* Device appearance */
    uint8 device_appearance[ATTR_LEN_DEVICE_APPEARANCE + 1] = {
                AD_TYPE_APPEARANCE,
                WORD_LSB(APPEARANCE_APPLICATION_VALUE),
                WORD_MSB(APPEARANCE_APPLICATION_VALUE)
                };

    /* A variable to keep track of the data added to advert_data. The limit is
     * MAX_ADV_DATA_LEN. GAP layer will add AD Flags to advert_data which is 3
     * bytes. Refer BT Spec 4.0, Vol 3, Part C, Sec 11.1.3:
     *
     * First byte is length
     * second byte is AD TYPE = 0x1
     * Third byte is Flags description
     */
    uint16 length_added_to_adv = 3;
    
    temp_addr.type = ls_addr_type_public;
    MemCopy(&temp_addr.addr,
            &g_app_data.bonded_bd_addr.addr,
            sizeof(BD_ADDR_T));

    if(fast_connection)
    {
        adv_interval_min = FC_ADVERTISING_INTERVAL_MIN;
        adv_interval_max = FC_ADVERTISING_INTERVAL_MAX;
    }
    
    if((GapSetMode(gap_role_peripheral, gap_mode_discover_general,
                        connect_mode,
                        gap_mode_bond_yes,
                        gap_mode_security_unauthenticate) != ls_err_none) ||
                   (connect_mode == gap_mode_connect_directed && 
                   GapSetAdvAddress(&temp_addr) 
                   != ls_err_none) ||
       (GapSetAdvInterval(adv_interval_min, adv_interval_max)
                        != ls_err_none))
    {
        ReportPanic(app_panic_set_advert_params);
    }

    /* Reset existing advertising data */
    if(LsStoreAdvScanData(0, NULL, ad_src_advertise) != ls_err_none)
    {
        ReportPanic(app_panic_set_advert_data);
    }

    /* Reset existing scan response data */
    if(LsStoreAdvScanData(0, NULL, ad_src_scan_rsp) != ls_err_none)
    {
        ReportPanic(app_panic_set_scan_rsp_data);
    }

    /* Setup ADVERTISEMENT DATA */
    /* Add to advData only if undirected advertisements are to be done. */
    if(connect_mode == gap_mode_connect_undirected)
    {
    
       /* Add UUID list of the services supported by the device */
        length = GetSupported128BitUUIDServiceList(advert_data);

        /* One added for Length field, which will be added to Adv Data by GAP
         * layer
         */
        length_added_to_adv += (length + 1);

        if (LsStoreAdvScanData(length, advert_data,
                            ad_src_advertise) != ls_err_none)
        {
            ReportPanic(app_panic_set_advert_data);
        }

        /* One added for Length field, which will be added to Adv Data by GAP
         * layer
         */
        length_added_to_adv += (sizeof(device_appearance) + 1);

        /* Add device appearance to the advertisements */
        if (LsStoreAdvScanData(ATTR_LEN_DEVICE_APPEARANCE + 1,
            device_appearance, ad_src_advertise) != ls_err_none)
        {
            ReportPanic(app_panic_set_advert_data);
        }
    }
}

/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      InitGattData
 *
 *  DESCRIPTION
 *      This function initialises the application GATT data.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void InitGattData(void)
{
    g_gatt_data.advert_timer_value = TIMER_INVALID;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandleAccessRead
 *
 *  DESCRIPTION
 *      This function handles read operations on attributes (as received in
 *      GATT_ACCESS_IND message) maintained by the application and responds
 *      with the GATT_ACCESS_RSP message.
 *
 *  PARAMETERS
 *      p_ind [in]              Data received in GATT_ACCESS_IND message.
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HandleAccessRead(GATT_ACCESS_IND_T *p_ind)
{
    /* For the received attribute handle, check all the services that support
     * attribute 'Read' operation handled by application.
     */
    /* More services may be added here to support their read operations */
    if(DeviceInfoCheckHandleRange(p_ind->handle))
    {
        /* Attribute handle belongs to the DEVICE INFORMATION service */
        DeviceInfoHandleAccessRead(p_ind);
    }
#ifdef ENABLE_OTA
    else if(GattCheckHandleRange(p_ind->handle))
    {
        /* Attribute handle belongs to Gatt service */
        GattHandleAccessRead(p_ind);
    }
    else if(OtaCheckHandleRange(p_ind->handle))
    {
        /* Attribute handle belongs to OTA service */
        OtaHandleAccessRead(p_ind);
    }
#endif /* ENABLE_OTA */
    else if(BatteryCheckHandleRange(p_ind->handle))
    {
        /* Attribute handle belongs to BATTERY service */
        BatteryHandleAccessRead(p_ind);
    }
    else if(SerialCheckHandleRange(p_ind->handle))
    {
        SerialHandleAccessRead(p_ind);
    }
    else
    {
        /* Application doesn't support 'Read' operation on received attribute
         * handle, so return 'gatt_status_read_not_permitted' status.
         */
        GattAccessRsp(p_ind->cid, p_ind->handle,
                      gatt_status_read_not_permitted,
                      0, NULL);
    }

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandleAccessWrite
 *
 *  DESCRIPTION
 *      This function handles write operations on attributes (as received in
 *      GATT_ACCESS_IND message) maintained by the application and responds
 *      with the GATT_ACCESS_RSP message.
 *
 *  PARAMETERS
 *      p_ind [in]              Data received in GATT_ACCESS_IND message.
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HandleAccessWrite(GATT_ACCESS_IND_T *p_ind)
{
    /* For the received attribute handle, check all the services that support
     * attribute 'Write' operation handled by application.
     */
    /* More services may be added here to support their write operations */
    if(BatteryCheckHandleRange(p_ind->handle))
    {
         /* Attribute handle belongs to BATTERY service */
        BatteryHandleAccessWrite(p_ind);
    }
#ifdef ENABLE_OTA
    else if(GattCheckHandleRange(p_ind->handle))
    {
        /* Attribute handle belongs to Gatt service */
        GattHandleAccessWrite(p_ind);
    }
    else if(OtaCheckHandleRange(p_ind->handle))
    {
        /* Attribute handle belongs to OTA service */
        OtaHandleAccessWrite(p_ind);
    }
#endif /* ENABLE_OTA */
    else if(SerialCheckHandleRange(p_ind->handle))
    {
        /* Attribute handle belongs to Serial service */
        SerialHandleAccessWrite(p_ind);
    }
    else
    {
        /* Application doesn't support 'Write' operation on received  attribute
         * handle, so return 'gatt_status_write_not_permitted' status
         */
        GattAccessRsp(p_ind->cid, p_ind->handle,
                      gatt_status_write_not_permitted,
                      0, NULL);
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GattStartAdverts
 *
 *  DESCRIPTION
 *      This function is used to start undirected advertisements. 
 *
 *  PARAMETERS
 *      p_addr [in]             Bonded host address
 *      fast_connection [in]    TRUE:  Fast advertisements
 *                              FALSE: Slow advertisements
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void GattStartAdverts(bool fast_connection,gap_mode_connect connect_mode)
{
    /* Variable 'connect_flags' needs to be updated to have peer address type
     * if Directed advertisements are supported as peer address type will
     * only be used in that case. 
     */
#ifdef USE_STATIC_RANDOM_ADDRESS
    uint16 connect_flags = L2CAP_CONNECTION_SLAVE_UNDIRECTED |
                           L2CAP_OWN_ADDR_TYPE_RANDOM;
#else
    uint16 connect_flags = L2CAP_CONNECTION_SLAVE_UNDIRECTED |
                           L2CAP_OWN_ADDR_TYPE_PUBLIC;
#endif /* USE_STATIC_RANDOM_ADDRESS */
    
    /* Set advertisement parameters */
    gattSetAdvertParams(fast_connection,connect_mode);

    if(g_app_data.bonded &&
       (!GattIsAddressResolvableRandom(&g_app_data.bonded_bd_addr)))
    {
        if(connect_mode == gap_mode_connect_directed)
        {
            g_gatt_data.advert_timer_value = TIMER_INVALID;
            
            /* Start with directed advertisements */
            connect_flags = L2CAP_CONNECTION_SLAVE_DIRECTED |
                            L2CAP_OWN_ADDR_TYPE_PUBLIC |
                            L2CAP_PEER_ADDR_TYPE_PUBLIC;
        }
        else
        {
            /* When the device is bonded, set the advertising filter policy to 
             * "process scan and connection requests only from devices in white 
             * list"
             */
            connect_flags = L2CAP_CONNECTION_SLAVE_WHITELIST | 
                            L2CAP_OWN_ADDR_TYPE_PUBLIC;
        }
    }

    /* Start GATT connection in Slave role */
    GattConnectReq(NULL, connect_flags);

     /* Start advertisement timer */
    if(g_gatt_data.advert_timer_value)
    {
        StartAdvertTimer(g_gatt_data.advert_timer_value);
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GetSupported128BitUUIDServiceList
 *
 *  DESCRIPTION
 *      This function prepares the list of supported 128-bit service UUIDs to be
 *      added to Advertisement data. It also adds the relevant AD Type to the
 *      start of AD array.
 *
 *  PARAMETERS
 *      p_service_uuid_ad [out] AD Service UUID list
 *
 *  RETURNS
 *      Size of AD Service UUID list
 *----------------------------------------------------------------------------*/
extern uint16 GetSupported128BitUUIDServiceList(uint8 *p_service_uuid_ad)
{
    uint8   size_data = 0;              /* Size of AD Service UUID list */

    /* Add 128-bit UUID for supported main service */
    p_service_uuid_ad[size_data++] = AD_TYPE_SERVICE_UUID_128BIT_LIST;

    /* Add serial service UUID*/
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_16;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_15;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_14;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_13;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_12;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_11;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_10;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_9;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_8;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_7;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_6;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_5;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_4;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_3;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_2;
    p_service_uuid_ad[size_data++] = UUID_SERIAL_SERVICE_1;

    /* Add all the supported UUIDs in this function*/

    /* Return the size of AD service data. */
    return ((uint16)size_data);

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GattIsAddressResolvableRandom
 *
 *  DESCRIPTION
 *      This function checks if the address is resolvable random or not.
 *
 *  PARAMETERS
 *      p_addr [in]             Address to check
 *
 *  RETURNS
 *      TRUE if supplied address is a resolvable private address
 *      FALSE if supplied address is non-resolvable private address
 *----------------------------------------------------------------------------*/
extern bool GattIsAddressResolvableRandom(TYPED_BD_ADDR_T *p_addr)
{
    if(p_addr->type != L2CA_RANDOM_ADDR_TYPE ||
      (p_addr->addr.nap & BD_ADDR_NAP_RANDOM_TYPE_MASK)
                                      != BD_ADDR_NAP_RANDOM_TYPE_RESOLVABLE)
    {
        /* This is not a resolvable private address  */
        return FALSE;
    }
    return TRUE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GattTriggerFastAdverts
 *
 *  DESCRIPTION
 *      This function is used to trigger fast advertisements.
 *
 *  PARAMETERS
 *      p_addr [in]             Bonded host address
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void GattTriggerFastAdverts(TYPED_BD_ADDR_T *p_addr)
{
    g_gatt_data.advert_timer_value = FAST_CONNECTION_ADVERT_TIMEOUT_VALUE;
  
    /* Trigger fast connections */
    GattStartAdverts(TRUE,gap_mode_connect_undirected);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GattStopAdverts
 *
 *  DESCRIPTION
 *      This function is used to stop advertisements.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void GattStopAdverts(void)
{
    switch(AppGetState())
    {
        case app_state_fast_advertising:
        {
               /* Slow advertisement timer for reduced power connections. */
                g_gatt_data.advert_timer_value =
                            SLOW_CONNECTION_ADVERT_TIMEOUT_VALUE;
            
               /* Stop on-going advertisements */
                GattCancelConnectReq();
         }
         break;

        case app_state_slow_advertising:
            /* Stop on-going advertisements */
            GattCancelConnectReq();
        break;

        default:
            /* Ignore timer in remaining states */
        break;
    }
}
