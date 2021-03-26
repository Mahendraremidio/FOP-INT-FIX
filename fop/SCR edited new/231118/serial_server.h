/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *     serial_server.h
 *
 *  DESCRIPTION
 *      Header file for a simple serial server application.
 *
 ******************************************************************************/
#ifndef __SERIAL_SERVER_H__
#define __SERIAL_SERVER_H__

/*============================================================================*
 *  SDK Header includes
 *============================================================================*/
#include <types.h>
#include <bluetooth.h>
#include <timer.h>

/*============================================================================*
 *  Local Header includes
 *============================================================================*/

#include "serial_gatt.h"
#include "user_config.h"

/*============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Maximum number of words in central device Identity Resolving Key (IRK) */
#define MAX_WORDS_IRK                       (8)

/*=============================================================================*
 *  Public Data types
 *============================================================================*/

/* Application data structure */
typedef struct _APP_DATA_T
{
    /* Current state of application */
    app_state                  state;

    /* TYPED_BD_ADDR_T of the host to which device is connected */
    TYPED_BD_ADDR_T            con_bd_addr;

    /* Track the Connection Identifier (UCID) as Clients connect and
     * disconnect
     */
    uint16                     st_ucid;

    /* Boolean flag to indicate whether the device is bonded */
    bool                       bonded;

    /* TYPED_BD_ADDR_T of the host to which device is bonded */
    TYPED_BD_ADDR_T            bonded_bd_addr;

    /* Diversifier associated with the Long Term Key (LTK) of the bonded
     * device
     */
    uint16                     diversifier;

    /* Central Private Address Resolution IRK. Will only be used when
     * central device used resolvable random address.
     */
    uint16                     irk[MAX_WORDS_IRK];


    /* Boolean flag to indicate pairing button press */
    bool                       pairing_button_pressed;

    /* Timer ID for 'UNDIRECTED ADVERTS' and activity on the sensor device like
     * measurements or user intervention in CONNECTED state.
     */
    timer_id                   app_tid;

    /* Boolean flag to indicate whether to set white list with the bonded
     * device. This flag is used in an interim basis while configuring
     * advertisements.
     */
    bool                       enable_white_list;


    /* Boolean flag to indicate whether encryption is enabled with the bonded
     * host
     */
    bool                       encrypt_enabled;

    /* This timer will be used if the application is already bonded to the
     * remote host address but the remote device wanted to rebond which we had
     * declined. In this scenario, we give ample time to the remote device to
     * encrypt the link using old keys. If remote device doesn't encrypt the
     * link, we will disconnect the link when this timer expires.
     */
    timer_id                   bonding_reattempt_tid;

    /* Timer to control LED */
    timer_id                   led_timer_tid;
} APP_DATA_T;

extern APP_DATA_T g_app_data;
/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* Call the firmware Panic() routine and provide a single point for debugging
 * any application level panics
 */
extern void ReportPanic(app_panic_code panic_code);

/* Change the current state of the application */
extern void AppSetState(app_state new_state);

/* Return the current state of the application.*/
extern app_state AppGetState(void);

/* Check if the whitelist is enabled or not. */
extern bool IsWhiteListEnabled(void);

/* Handles pairing removal - removes bonding and triggers advertisements */
extern void HandlePairingRemoval(void);

/* Start the advertisement timer. */
extern void StartAdvertTimer(uint32 interval);

/* Return whether the connected device is bonded or not */
extern bool IsDeviceBonded(void);

/* Return the unique connection ID (UCID) of the connection */
extern uint16 GetConnectionID(void);

/* Handle a short button press. If idle, the device moves to fast
 * advertising.
 */
extern void HandleShortButtonPress(void);

extern void HandleExtTrigger(void);
extern void white(void);

/* Handle a long button press. If connected, the device disconnects from the
 * host.
 */
extern void HandleLongButtonPress(void);

/* Function that returns whether radio events are configured or not */
extern bool IsAppWaitingForRadioEvent(void);


#ifdef NVM_TYPE_FLASH
/* This function writes the application data to NVM. This function should 
 * be called on getting nvm_status_needs_erase
 */
extern void WriteApplicationAndServiceDataToNVM(void);
#endif /* NVM_TYPE_FLASH */

#endif /* __SERIAL_SERVER_H__ */

