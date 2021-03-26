      /***************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 *  FILE
 *      serial_server.c
 *
 *  DESCRIPTION
 *      This file defines a simple implementation of a serial server.
 *
 ******************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <main.h>
#include <types.h>
#include <timer.h>
#include <mem.h>
#include <config_store.h>
#include <debug.h> 
#include <gatt.h>
#include <ls_app_if.h>
#include <gap_app_if.h>
#include <buf_utils.h>
#include <security.h>
#include <panic.h>
#include <nvm.h>
#include <random.h>
#include <pio.h>
#include <pio_ctrlr.h>
#include <sleep.h>
#include <csr_ota.h>
#include <aio.h>
#include <i2c.h>
#include "i2c_comms.h"








/*============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "serial_gatt.h"
#include "app_gatt_db.h"
#include "nvm_access.h"
#include "serial_server.h"
#include "hw_access.h"
#include "battery_service.h"
#include "serial_service.h"
#include "uart_interface.h"
#include "byte_queue.h"
#include "user_config.h"
#include "gatt_service.h"
#include "csr_ota_service.h"
#include "vf_pwm.h"

/*============================================================================*
 *  Private Definitions
 *============================================================================*/

/* Maximum number of timers.required by this application
 *
 *  This file:        app_tid
 *  This file:        bonding_reattempt_tid 
 *  hw_access.c:      button_press_tid
 */
#define MAX_APP_TIMERS                 (8)

/* Number of Identity Resolving Keys (IRKs) that application can store */
#define MAX_NUMBER_IRK_STORED          (1)

/* Magic value to check the sanity of Non-Volatile Memory (NVM) region used by
 * the application. This value is unique for each application.
 */
#define NVM_SANITY_MAGIC               (0xAB19)

/* NVM offset for NVM sanity word */
#define NVM_OFFSET_SANITY_WORD         (0)

/* NVM offset for bonded flag */
#define NVM_OFFSET_BONDED_FLAG         (NVM_OFFSET_SANITY_WORD + 1)

/* NVM offset for bonded device Bluetooth address */
#define NVM_OFFSET_BONDED_ADDR         (NVM_OFFSET_BONDED_FLAG + \
                                        sizeof(g_app_data.bonded))

/* NVM offset for diversifier */
#define NVM_OFFSET_SM_DIV              (NVM_OFFSET_BONDED_ADDR + \
                                        sizeof(g_app_data.bonded_bd_addr))

/* NVM offset for IRK */
#define NVM_OFFSET_SM_IRK              (NVM_OFFSET_SM_DIV + \
                                        sizeof(g_app_data.diversifier))

/* Number of words of NVM used by application. Memory used by supported
 * services is not taken into consideration here.
 */
#define NVM_MAX_APP_MEMORY_WORDS       (NVM_OFFSET_SM_IRK + \
                                        MAX_WORDS_IRK)

/* Macro definitions for LED on/off time duration */
#define TIMER_ADV_ON_OFF_DURATION      (500 * MILLISECOND)
#define TIMER_CONNECTED_ON_DURATION    (100 * MILLISECOND)
#define TIMER_CONNECTED_OFF_DURATION   (5   * SECOND)

/*Application data variable */

#define MODE1   0x31     /*Mode-1 definition */
#define MODE2   0x32    /*Mode-2 definition */

#define WHITE_INT   0x57 /* White LED Command*/

#define IR_INT      0x52 /* IR LED Commnd*/

#define TRIGGER1     0x00 /*Trigger press event*/
#define TRIGGER2     0x01 /*Trigger release event*/


#define TRIGGER3     0x03 /*Trigger press event*/
#define TRIGGER4     0x02 /*Trigger release event*/



/* PIO numbers for PWM Ports */
#define PWM0_PORT  8
#define PWM1_PORT  9
#define PWM2_PORT  10
/*#define PWM3_PORT  11*/
#define PWM4_PORT  12
#define PWM5_PORT  13
#define PWM6_PORT  14
#define PWM7_PORT  15

#define BUFFER_SIZE 11          /* Buffer size required to hold maximum value */



uint8 mode,w_int=0x80,ir_int=0x80,io_led;
uint8 ar[2] = {'\0','\0'};

uint8 cnt=0;

uint8 cnt1=0;

static uint8 reg = 0;                                                                               
/* Macro definition for LED on/off time duration */

uint8 device_battery = 0;
/*============================================================================*
 *  Private Data
 *============================================================================*/

/* Declare space for application timers */
static uint16 app_timers[SIZEOF_APP_TIMER * MAX_APP_TIMERS];

/* Application data instance */
APP_DATA_T g_app_data;

/* Variable to toggle LED on/off */
bool g_turn_on_led_advert = TRUE;
bool g_turn_on_led_connect = TRUE;

bool g_radio_event_configured = FALSE;

/* variable to track the connection counts upon a link loss*/
uint16 g_adv_count = 0;

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

/* Initialise application data structure */
static void appDataInit(void);

/* Initialise and read NVM data */
static void readPersistentStore(void);

/* Enable whitelist based advertising */
static void enableWhiteList(void);

/* Handle Idle timer expiry in connected states */
static void appIdleTimerHandler(timer_id tid);

/* Reset the time for which the application was idle */
static void resetIdleTimer(void);

/* Handle the expiry of the bonding chance timer */
static void handleBondingChanceTimerExpiry(timer_id tid);

/* Exit the advertising states */
static void appExitAdvertising(void);

/* Exit the initialisation state */
static void appInitExit(void);

/* Handle advertising timer expiry */
static void appAdvertTimerHandler(timer_id tid);

/* GATT_ADD_DB_CFM signal handler */
static void handleSignalGattAddDbCfm(GATT_ADD_DB_CFM_T *p_event_data);

/* GATT_CANCEL_CONNECT_CFM signal handler */
static void handleSignalGattCancelConnectCfm(void);

/* GATT_CONNECT_CFM signal handler */
static void handleSignalGattConnectCfm(GATT_CONNECT_CFM_T *p_event_data);

/* SM_KEYS_IND signal handler */
static void handleSignalSmKeysInd(SM_KEYS_IND_T *p_event_data);

/* SM_PAIRING_AUTH_IND signal handler */
static void handleSignalSmPairingAuthInd(
                    SM_PAIRING_AUTH_IND_T *p_event_data);

/* LM_EV_ENCRYPTION_CHANGE signal handler */
static void handleSignalLMEncryptionChange(
                    HCI_EV_DATA_ENCRYPTION_CHANGE_T *p_event_data);


/* SM_SIMPLE_PAIRING_COMPLETE_IND signal handler */
static void handleSignalSmSimplePairingCompleteInd(
                    SM_SIMPLE_PAIRING_COMPLETE_IND_T *p_event_data);

/* SM_DIV_APPROVE_IND signal handler */
static void handleSignalSmDivApproveInd(SM_DIV_APPROVE_IND_T *p_event_data);

/* GATT_ACCESS_IND signal handler */
static void handleSignalGattAccessInd(GATT_ACCESS_IND_T *p_event_data);

/* LM_EV_DISCONNECT_COMPLETE signal handler */
static void handleSignalLmDisconnectComplete(
                    HCI_EV_DATA_DISCONNECT_COMPLETE_T *p_event_data);

/* LS_RADIO_EVENT_IND signal handler */
static void handleSignalLsRadioEventInd(LS_RADIO_EVENT_IND_T *p_event_data);

/* Function that plays LED on/off */
static void handleLED(timer_id tid);

/* Start advertisements */
static void appStartAdvert(void);

/* Record whether the last BLE notification sent was success or failure.This is 
 * to ensure flow control at the application level.
 */
static void setLastNotificationResult(bool bsuccess);

/* Empty the UART buffers */
static void emptySendRecvBuffers(void);

/* Configures a PWM port. */
/*void PioFastPwmConfig(uint32 pio_mask);*/

/* Enable the PWM. */
/*void PioFastPwmEnable(bool enable);*/

/* Included externally in PIO controller code.*/
/*void pio_ctrlr_code(void);*/

/* Sets the bright/dull widths for a PWM port. */
/*bool PioFastPwmSetWidth(uint8 pwm_port, uint8 bright_width, uint8 dull_width,
                        bool inverted);*/

/* Sets the PWM Periods. */
/*void PioFastPwmSetPeriods(uint16 bright, uint16 dull);*/

/* White LED PWM*/

/*void WLED_PWM(uint8 wled);*/



/* IR LED PWM*/

/*void IR_PWM(uint8 ir);*/

static void handler(timer_id const id);

static void Led_handler(timer_id const id);

static uint8 writeASCIICodedNumber(uint32 value);

static void Handler_blk(timer_id const id);
timer_id tim_tid;

timer_id tim_tid1;
timer_id tim_tidf;

bool Flag0 =0, Flag1=0, Flag2=0, Flag3=0,Flag4=0,Flag5=0,Flag6=0,Flag7=0,Adp,Chg;

bool F1=0,F2=0,F3=0,F4=0,blk=0;

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      appDataInit
 *
 *  DESCRIPTION
 *      This function is called to initialise the application data structure.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void Handler_blk(timer_id const id)
{
    if(blk==0)
    {
      blk=1;
      if(I2CAcquire())
          {
              I2CcommsInit();
              I2C_IO_Write(00);
              I2CRelease(); 
              TimeDelayUSec(2* MILLISECOND);
           }
   }
    else if(blk==1)
    {  
        blk=0;
      if(I2CAcquire())
          {
              I2CcommsInit();
              I2C_IO_Write(reg);
              I2CRelease(); 
              TimeDelayUSec(2* MILLISECOND);
           }
  }
    tim_tidf = TimerCreate(250*MILLISECOND,TRUE,Handler_blk);
}



static void appDataInit(void)
{
    /* Initialise general application timer */
    if (g_app_data.app_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_data.app_tid);
        g_app_data.app_tid = TIMER_INVALID;
    }

    /* Reset the pairing button press flag */
    g_app_data.pairing_button_pressed = FALSE;

    /* Initialise the connected client ID */
    g_app_data.st_ucid = GATT_INVALID_UCID;

    /* Initialise white list flag */
    g_app_data.enable_white_list = FALSE;


    /* Initialise link encryption flag */
    g_app_data.encrypt_enabled = FALSE;

    /* Initialise the bonding reattempt timer */
    if (g_app_data.bonding_reattempt_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_data.bonding_reattempt_tid);
        g_app_data.bonding_reattempt_tid = TIMER_INVALID;
    }

    /* Initialise the application GATT data */
    InitGattData();

    /* Battery Service data initialisation */
    BatteryDataInit();

#ifdef ENABLE_OTA
    /* Initialise GATT Data structure */
    GattDataInit();    
    /* OTA Service data initialisation */
    OtaDataInit();
#endif /* ENABLE_OTA */

    /* Start LED */
    if (g_app_data.led_timer_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_data.led_timer_tid);
        g_app_data.led_timer_tid = TIMER_INVALID;
    }

    g_app_data.led_timer_tid = TimerCreate(TIMER_ADV_ON_OFF_DURATION,
                                               TRUE,
                                               handleLED);
    /* Call the required service data initialisation APIs from here */
    
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      setLastNotificationResult
 *
 *  DESCRIPTION
 *      Records the last notification was sent successfully or not
 *
 *  PARAMETERS
 *      bvalue             TRUE success and FALSE for failure.
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void setLastNotificationResult(bool bsuccess)
{
    SetLastNotificationStatus(bsuccess);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      emptySendRecvBuffers
 *
 *  DESCRIPTION
 *      This function is used to clear the UART buffers
 *      
 *
 *  PARAMETERS
 *      Nothing
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/ 
static void emptySendRecvBuffers(void)
{
    BQClearBuffer(0);
    BQClearBuffer(1);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      readPersistentStore
 *
 *  DESCRIPTION
 *      This function is used to initialise and read NVM data.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void readPersistentStore(void)
{
    /* NVM offset for supported services */
    uint16 nvm_offset = NVM_MAX_APP_MEMORY_WORDS;
    uint16 nvm_sanity = 0xffff;
    bool nvm_start_fresh = FALSE;

    /* Read persistent storage to find if the device was last bonded to another
     * device. If the device was bonded, trigger fast undirected advertisements
     * by setting the white list for bonded host. If the device was not bonded,
     * trigger undirected advertisements for any host to connect.
     */

    Nvm_Read(&nvm_sanity,
             sizeof(nvm_sanity),
             NVM_OFFSET_SANITY_WORD);

    if(nvm_sanity == NVM_SANITY_MAGIC)
    {

        /* Read Bonded Flag from NVM */
        Nvm_Read((uint16*)&g_app_data.bonded,
                  sizeof(g_app_data.bonded),
                  NVM_OFFSET_BONDED_FLAG);

        if(g_app_data.bonded)
        {
            /* Bonded Host Typed BD Address will only be stored if bonded flag
             * is set to TRUE. Read last bonded device address.
             */
            Nvm_Read((uint16*)&g_app_data.bonded_bd_addr,
                       sizeof(TYPED_BD_ADDR_T),
                       NVM_OFFSET_BONDED_ADDR);

            /* If device is bonded and bonded address is resolvable then read
             * the bonded device's IRK
             */
            if(GattIsAddressResolvableRandom(&g_app_data.bonded_bd_addr))
            {
                Nvm_Read(g_app_data.irk,
                         MAX_WORDS_IRK,
                         NVM_OFFSET_SM_IRK);
            }

        }
        else /* Case when we have only written NVM_SANITY_MAGIC to NVM but
              * didn't get bonded to any host in the last powered session
              */
        {
            /* Any initialisation can be done here for non-bonded devices */

        }

        /* Read the diversifier associated with the presently bonded/last
         * bonded device.
         */
        Nvm_Read(&g_app_data.diversifier,
                 sizeof(g_app_data.diversifier),
                 NVM_OFFSET_SM_DIV);

        /* Read Serial client CCD fron NVM */
        SerialReadDataFromNVM(TRUE,&nvm_offset);

    }
    else /* NVM Sanity check failed means either the device is being brought up
          * for the first time or memory has got corrupted in which case
          * discard the data and start fresh.
          */
    {

        nvm_start_fresh = TRUE;

        nvm_sanity = NVM_SANITY_MAGIC;

        /* Write NVM Sanity word to the NVM */
        Nvm_Write(&nvm_sanity,
                  sizeof(nvm_sanity),
                  NVM_OFFSET_SANITY_WORD);

        /* The device will not be bonded as it is coming up for the first
         * time
         */
        g_app_data.bonded = FALSE;

        /* Write bonded status to NVM */
        Nvm_Write((uint16*)&g_app_data.bonded,
                   sizeof(g_app_data.bonded),
                  NVM_OFFSET_BONDED_FLAG);

        /* When the application is coming up for the first time after flashing
         * the image to it, it will not have bonded to any device. So, no LTK
         * will be associated with it. Hence, set the diversifier to 0.
         */
        g_app_data.diversifier = 0;

        /* Write the same to NVM. */
        Nvm_Write(&g_app_data.diversifier,
                  sizeof(g_app_data.diversifier),
                  NVM_OFFSET_SM_DIV);

        /* Read Serial client CCD fron NVM */
        SerialReadDataFromNVM(FALSE,&nvm_offset);

    }

    /* Read Battery service data from NVM if the devices are bonded and
     * update the offset with the number of words of NVM required by
     * this service
     */
    BatteryReadDataFromNVM(&nvm_offset);

    /* Add the 'read Service data from NVM' API call here, to initialise the
     * service data, if the device is already bonded. One must take care of the
     * offset being used for storing the data.
     */

    /* Read GATT service data from NVM */    
#ifdef ENABLE_OTA
    GattReadDataFromNVM(&nvm_offset);
#endif /* ENABLE_OTA */

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      enableWhiteList
 *
 *  DESCRIPTION
 *      This function enables white list based advertising.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void enableWhiteList(void)
{
    if(IsDeviceBonded())
    {
        if(!GattIsAddressResolvableRandom(&g_app_data.bonded_bd_addr))
        {
            /* Enable white list if the device is bonded and the bonded host
             * is not using resolvable random address.
             */
            g_app_data.enable_white_list = TRUE;
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appIdleTimerHandler
 *
 *  DESCRIPTION
 *      This function is used to handle Idle timer expiry in connected states.
 *      At the expiry of this timer, application shall disconnect with the
 *      host and shall move to 'APP_DISCONNECTING' state.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appIdleTimerHandler(timer_id tid)
{
    if(tid == g_app_data.app_tid)
    {
        /* Timer has just expired, so mark it as invalid */
        g_app_data.app_tid = TIMER_INVALID;

        /* Handle signal as per current state */
        switch(g_app_data.state)
        {
            case app_state_connected:
            {
                /* Trigger Disconnect and move to app_state_disconnecting
                 * state
                 */
                AppSetState(app_state_disconnecting);
            }
            break;

            default:
                /* Ignore timer in any other state */
            break;
        }

    } /* Else ignore the timer */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleBondingChanceTimerExpiry
 *
 *  DESCRIPTION
 *      This function is handle the expiry of bonding chance timer.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleBondingChanceTimerExpiry(timer_id tid)
{
    if(g_app_data.bonding_reattempt_tid == tid)
    {
        /* The timer has just expired, so mark it as invalid */
        g_app_data.bonding_reattempt_tid = TIMER_INVALID;
        /* The bonding chance timer has expired. This means the remote has not
         * encrypted the link using old keys. Disconnect the link.
         */
        AppSetState(app_state_disconnecting);
    }/* Else it may be due to some race condition. Ignore it. */
}


/*-----------------------------------------------------------------------------*
 *  NAME
 *      appStartAdvert
 *
 *  DESCRIPTION
 *      This function is used to start directed advertisements if a valid
 *      reconnection address has been written by the remote device. Otherwise,
 *      it starts fast undirected advertisements.
 *
 *  RETURNS/MODIFIES
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void appStartAdvert(void)
{
    app_state advert_type;

    /* Check if the app is bonded */
    if(g_app_data.bonded)
    {
        /* Start directed advertisement */
        advert_type = app_state_directed_advertising;
    }
    else 
    {
        /* Start with fast advertisements */
        advert_type = app_state_fast_advertising;
    }

    /* Set the application state */
    AppSetState(advert_type);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appExitAdvertising
 *
 *  DESCRIPTION
 *      This function is called while exiting app_state_fast_advertising and
 *      app_state_slow_advertising states.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appExitAdvertising(void)
{
    /* Cancel advertisement timer. Must be valid because timer is active
     * during app_state_fast_advertising and app_state_slow_advertising states.
     */
    TimerDelete(g_app_data.app_tid);
    g_app_data.app_tid = TIMER_INVALID;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appAdvertTimerHandler
 *
 *  DESCRIPTION
 *      This function is used to handle Advertisement timer expiry.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appAdvertTimerHandler(timer_id tid)
{
    /* Based upon the timer id, stop on-going advertisements */
    if(g_app_data.app_tid == tid)
    {
        /* Timer has just expired so mark it as invalid */
        g_app_data.app_tid = TIMER_INVALID;

        GattStopAdverts();
    }/* Else ignore timer expiry, could be because of
      * some race condition */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appInitExit
 *
 *  DESCRIPTION
 *      This function is called upon exiting from app_state_init state. The
 *      application starts advertising after exiting this state.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appInitExit(void)
{
    if(g_app_data.bonded &&
       !GattIsAddressResolvableRandom(&g_app_data.bonded_bd_addr))
    {
        /* If the device is bonded and the bonded device address is not
         * resolvable random, configure the white list with the bonded
         * host address.
         */
        if(LsAddWhiteListDevice(&g_app_data.bonded_bd_addr) != ls_err_none)
        {
            ReportPanic(app_panic_add_whitelist);
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      resetIdleTimer
 *
 *  DESCRIPTION
 *      This function is used to reset the time for which the application was
 *      idle during the connected state.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void resetIdleTimer(void)
{
    /* Delete the Idle timer, if already running */
    if (g_app_data.app_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_data.app_tid);
        g_app_data.app_tid = TIMER_INVALID;
    }

    /* Start the Idle timer again.*/
    g_app_data.app_tid  = TimerCreate(CONNECTED_IDLE_TIMEOUT_VALUE,
                                    TRUE, appIdleTimerHandler);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattAddDbCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_ADD_DB_CFM.
 *
 *  PARAMETERS

 *      p_event_data [in]       Data supplied by GATT_ADD_DB_CFM signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalGattAddDbCfm(GATT_ADD_DB_CFM_T *p_event_data)
{
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_init:
        {
            if(p_event_data->result == sys_status_success)
            {
                /* Start advertising. */
                appStartAdvert();
            }
            else
            {
                /* This should never happen */
                ReportPanic(app_panic_db_registration);
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}
/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattCancelConnectCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CANCEL_CONNECT_CFM.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalGattCancelConnectCfm(void)
{
    if(g_app_data.pairing_button_pressed)
    {
        /* Pairing removal has been initiated by the user */
        g_app_data.pairing_button_pressed = FALSE;

        /* Disable white list */
        g_app_data.enable_white_list = FALSE;

        /* Reset and clear the white list */
        LsResetWhiteList();

        /* Trigger fast advertisements */
        if(g_app_data.state == app_state_fast_advertising)
        {
            GattTriggerFastAdverts(&g_app_data.bonded_bd_addr);
        }
        else
        {
            AppSetState(app_state_fast_advertising);
        }
    }
    else
    {
        /* Handle signal as per current state.
         *
         * The application follows this sequence in advertising state:
         *
         * 1. Fast advertising for FAST_CONNECTION_ADVERT_TIMEOUT_VALUE seconds:
         *
         * 2. Slow advertising for SLOW_CONNECTION_ADVERT_TIMEOUT_VALUE seconds
         */
        switch(g_app_data.state)
        {
            case app_state_fast_advertising:
            {
                 AppSetState(app_state_slow_advertising);

            }
            break;

            case app_state_slow_advertising:
                /* If the application was doing slow advertisements, stop
                 * advertising and move to idle state.
                 */
                AppSetState(app_state_idle);
            break;

            default:
                /* Control should never come here */
                ReportPanic(app_panic_invalid_state);
            break;
        }

    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattConnectCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CONNECT_CFM.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by GATT_CONNECT_CFM signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalGattConnectCfm(GATT_CONNECT_CFM_T *p_event_data)
{
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_directed_advertising:
        case app_state_fast_advertising:   /* FALLTHROUGH */
        case app_state_slow_advertising:
        {
            if(p_event_data->result == sys_status_success)
            {

                /* Store received UCID */
                g_app_data.st_ucid = p_event_data->cid;

                /* Store connected BD Address */
                g_app_data.con_bd_addr = p_event_data->bd_addr;

                if(g_app_data.bonded &&
                   GattIsAddressResolvableRandom(&g_app_data.bonded_bd_addr) &&
                   (SMPrivacyMatchAddress(&p_event_data->bd_addr,
                                          g_app_data.irk,
                                          MAX_NUMBER_IRK_STORED,
                                          MAX_WORDS_IRK) < 0))
                {
                    /* Application was bonded to a remote device using
                     * resolvable random address and application has failed
                     * to resolve the remote device address to which we just
                     * connected, so disconnect and start advertising again
                     */
                    AppSetState(app_state_disconnecting);
                }
                else
                {
                    /* Enter connected state
                     * - If the device is not bonded OR
                     * - If the device is bonded and the connected host doesn't
                     *   support Resolvable Random address OR
                     * - If the device is bonded and connected host supports
                     *   Resolvable Random address and the address gets resolved
                     *   using the stored IRK key
                     */
                    AppSetState(app_state_connected);
                    
#ifdef ENABLE_OTA
                /* If we are bonded to this host, then it may be appropriate
                 * to indicate that the database is not now what it had
                 * previously.
                 */
                if(g_app_data.bonded)
                {
                    GattOnConnection(p_event_data->cid);
                }
#endif /* ENABLE_OTA */
                     
                /* Reset the radio events configured flag */
                g_radio_event_configured = FALSE;
               }
            }
            else
            {
                /* Connect Failed. Increment the advertising attempt count
                 * and enter fast advertising state
                 */
                if(g_adv_count<MAX_RETRY_ADVERT_ATTEMPT)
                {
                    if((p_event_data)->result == 
                        HCI_ERROR_DIRECTED_ADVERTISING_TIMEOUT)
                    {                   
                          AppSetState(app_state_fast_advertising);
                    }
                }
                else
                {
                   AppSetState(app_state_idle);
                }
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalSmKeysInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_KEYS_IND and copies the IRK from it.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by SM_KEYS_IND signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalSmKeysInd(SM_KEYS_IND_T *p_event_data)
{
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_connected:
        {
          /* If keys are present, save them */
          if((p_event_data->keys)->keys_present & (1 << SM_KEY_TYPE_DIV))
          { 
            /* Store the diversifier which will be used for accepting/rejecting
             * the encryption requests.
             */
            g_app_data.diversifier = (p_event_data->keys)->div;

            /* Write the new diversifier to NVM */
            Nvm_Write(&g_app_data.diversifier,
                      sizeof(g_app_data.diversifier),
                      NVM_OFFSET_SM_DIV);
          }

            /* Store IRK if the connected host is using random resolvable
             * address. IRK is used afterwards to validate the identity of
             * connected host
             */
            if(GattIsAddressResolvableRandom(&g_app_data.con_bd_addr)&&
               ((p_event_data->keys)->keys_present & (1 << SM_KEY_TYPE_ID)))
            {
                MemCopy(g_app_data.irk,
                        (p_event_data->keys)->irk,
                        MAX_WORDS_IRK);

                /* If bonded device address is resolvable random
                 * then store IRK to NVM
                 */
                Nvm_Write(g_app_data.irk,
                          MAX_WORDS_IRK,
                          NVM_OFFSET_SM_IRK);
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattNotificationCfm
 *
 *  DESCRIPTION
 *      This function handles the Notification Confirm message.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by GATT_CHAR_VAL_IND_CFM_T signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalGattNotificationCfm(
                                         GATT_CHAR_VAL_IND_CFM_T *p_event_data)
{
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_connected:
        {
            if(p_event_data->result == sys_status_success)
            {
                /* remove the last sent message from the queue */
                setLastNotificationResult(TRUE);
                
                /* reset idle timer */
                resetIdleTimer();
                
                /* Process Data */
                ProcessRxData();
            }
            else if(p_event_data->result == gatt_status_busy)
            {
                 g_radio_event_configured = TRUE;
                 /* Enable radio events on Tx data. */
                 LsRadioEventNotification(p_event_data->cid, 
                                             radio_event_tx_data);
                 
                 /* Last notification failed. set the notification status */
                 setLastNotificationResult(FALSE);
                 
                 /* reset idle timer */
                resetIdleTimer();
            }
        }
        break;
        
        default:
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalSmPairingAuthInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_PAIRING_AUTH_IND. This message will
 *      only be received when the peer device is initiating 'Just Works'
 *      pairing.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by SM_PAIRING_AUTH_IND signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalSmPairingAuthInd(SM_PAIRING_AUTH_IND_T *p_event_data)
{
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_connected:
        {
            /* Authorise the pairing request if the application is NOT bonded */
            if(!g_app_data.bonded)
            {
                SMPairingAuthRsp(p_event_data->data, TRUE);
            }
            else /* Otherwise Reject the pairing request */
            {
                SMPairingAuthRsp(p_event_data->data, FALSE);
            }
        }
        break;

        default:
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLMEncryptionChange
 *
 *  DESCRIPTION
 *      This function handles the signal LM_EV_ENCRYPTION_CHANGE.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by LM_EV_ENCRYPTION_CHANGE signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalLMEncryptionChange(
                    HCI_EV_DATA_ENCRYPTION_CHANGE_T *p_event_data)
{
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_connected:
        {
            if(p_event_data->status == sys_status_success)
            {
                if(g_adv_count != 0)
                {
                   /* A link loss reconnect had succeeded.
                    * Reset retry attempts.
                    */
                   g_adv_count = 0;
                   
                   /* Try to send data, if any pending in queue */
                   ProcessRxData();
                }
                else
                {
                    /* Fresh Connection.Clear UART buffers */
                    emptySendRecvBuffers();
                } 
                
                g_app_data.encrypt_enabled = p_event_data->enc_enable;

                if(g_app_data.encrypt_enabled)
                {
                    /* Delete the bonding chance timer, if running */
                    if (g_app_data.bonding_reattempt_tid != TIMER_INVALID)
                    {
                        TimerDelete(g_app_data.bonding_reattempt_tid);
                        g_app_data.bonding_reattempt_tid = TIMER_INVALID;
                    }

                    /* Update battery status at every connection instance. It
                     * may not be worth updating timer this often, but this will
                     * depend upon application requirements.
                     */
                    BatteryUpdateLevel(g_app_data.st_ucid);
                }
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalSmSimplePairingCompleteInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_SIMPLE_PAIRING_COMPLETE_IND.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by SM_SIMPLE_PAIRING_COMPLETE_IND
 *                              signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalSmSimplePairingCompleteInd(
                                 SM_SIMPLE_PAIRING_COMPLETE_IND_T *p_event_data)
{

    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_connected:
        {
            if(p_event_data->status == sys_status_success)
            {
                /* Store bonded host information to NVM. This includes
                 * application and service specific information.
                 */
                g_app_data.bonded = TRUE;
                g_app_data.bonded_bd_addr = p_event_data->bd_addr;

                /* Store bonded host typed bd address to NVM */

                /* Write one word bonded flag */
                Nvm_Write((uint16*)&g_app_data.bonded,
                          sizeof(g_app_data.bonded),
                          NVM_OFFSET_BONDED_FLAG);

                /* Write typed bd address of bonded host */
                Nvm_Write((uint16*)&g_app_data.bonded_bd_addr,
                          sizeof(TYPED_BD_ADDR_T),
                          NVM_OFFSET_BONDED_ADDR);

                /* Configure white list with the Bonded host address only
                 * if the connected host doesn't support random resolvable
                 * addresses
                 */
                if(!GattIsAddressResolvableRandom(&g_app_data.bonded_bd_addr))
                {
                    /* It is important to note that this application does not
                     * support Reconnection Address. In future, if the
                     * application is enhanced to support Reconnection Address,
                     * make sure that we don't add Reconnection Address to the
                     * white list
                     */
                    if(LsAddWhiteListDevice(&g_app_data.bonded_bd_addr) !=
                        ls_err_none)
                    {
                        ReportPanic(app_panic_add_whitelist);
                    }

                }

                /* If the devices are bonded then send notification to all
                 * registered services for the same so that they can store
                 * required data to NVM.
                 */

#ifdef ENABLE_OTA
                /* Notify the Gatt service about the pairing */
                GattBondingNotify();
#endif /* ENABLE_OTA */            

                BatteryBondingNotify();
                
                 /* Add the Service Bonding Notify API here */
                SerialBondingNotify();
            }
            else
            {
                /* Pairing has failed.
                 * 1. If pairing has failed due to repeated attempts, the
                 *    application should immediately disconnect the link.
                 * 2. If the application was bonded and pairing has failed, then
                 *    since the application was using a white list the remote
                 *    device has the same address as our bonded device address.
                 *    The remote connected device may be a genuine one but
                 *    instead of using old keys, wanted to use new keys. We do
                 *    not allow bonding again if we are already bonded but we
                 *    will give the connected device some time to encrypt the
                 *    link using the old keys. If the remote device fails to
                 *    encrypt the link in that time we will disconnect the link.
                 */
                 if(p_event_data->status == sm_status_repeated_attempts)
                 {
                    AppSetState(app_state_disconnecting);
                 }
                 else if(g_app_data.bonded)
                 {
                    g_app_data.encrypt_enabled = FALSE;
                    g_app_data.bonding_reattempt_tid =
                                          TimerCreate(
                                               BONDING_CHANCE_TIMER,
                                               TRUE,
                                               handleBondingChanceTimerExpiry);
                 }
            }
        }
        break;

        default:
            /* Firmware may send this signal after disconnection. So don't
             * panic but ignore this signal.
             */
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalSmDivApproveInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_DIV_APPROVE_IND.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by SM_DIV_APPROVE_IND signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalSmDivApproveInd(SM_DIV_APPROVE_IND_T *p_event_data)
{
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {

        /* Request for approval from application comes only when pairing is not
         * in progress
         */
        case app_state_connected:
        {
            sm_div_verdict approve_div = SM_DIV_REVOKED;

            /* Check whether the application is still bonded (bonded flag gets
             * reset upon 'connect' button press by the user). Then check
             * whether the diversifier is the same as the one stored by the
             * application
             */
            if(g_app_data.bonded)
            {
                if(g_app_data.diversifier == p_event_data->div)
                {
                    approve_div = SM_DIV_APPROVED;
                }
            }

            SMDivApproval(p_event_data->cid, approve_div);
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;

    }
}



/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattAccessInd
 *
 *  DESCRIPTION
 *      This function handles GATT_ACCESS_IND messages for attributes maintained
 *      by the application.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by GATT_ACCESS_IND message
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalGattAccessInd(GATT_ACCESS_IND_T *p_event_data)
{

    unsigned char *ch= p_event_data->value;
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_connected:
        {
                                                                                    
                                              PioSetMode(CHG, pio_mode_user);         /*000 ====================> State-0 */
                                                PioSetDir(CHG, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<CHG), pio_mode_strong_pull_up);
                                                PioSet(CHG,FALSE);   
                                                
                                                                                            
                                                     
                                                
                                                                                    

            if(!Flag3)
            {
                if(p_event_data->size_value ==1)
                {
                     Flag7=1;
                    
                     switch((*ch))
                    {
                    case 0x31:
                              mode = MODE1;
                              
                              
                              break;
                    case 0x32:
                              mode = MODE2;
                              
                              break;
                              
                    case 0x03: 
                              
                              mode = MODE1;
                              Flag4=0;
                              reg=0x00;
                                if(I2CAcquire())
                                {
                                   I2CcommsInit();
                                   I2C_IO_Write(reg);
                                   I2CRelease(); 
                                   TimeDelayUSec(2* MILLISECOND);
                                 }
                              break;
                                        
                    case 0x04: 
                              
                              Flag4=1;
                              VFPWMStart(0xFF,w_int);
                              /*DebugWriteString("Debug 04");*/
                              break;
                              
                    case 0x02:
                              
                              VFPWMStart(0xFF,w_int);
                              Flag5=1;
                              break;
                    
                    case 0x05:
                              
                              mode = MODE2;
                              Flag5=0;
                              break;
                   case 0x01:
                              reg=0x00;
                                if(I2CAcquire())
                                {
                                   I2CcommsInit();
                                   I2C_IO_Write(reg);
                                   I2CRelease(); 
                                   TimeDelayUSec(2* MILLISECOND);
                                 }
                              
                              break;
                                          
                    
                                                                 
                      default:
                              break;
                 }
                
            }
            
            
            
            if(p_event_data->size_value ==2)
            {
                 switch((*ch++))
                 {
                     case 0x57:
                             Flag7=1;
                            {
                                 w_int = (uint8)*ch++;
                                 w_int = 0X00+ w_int;
                                                
                                
                            }
                            break;
                            
                       case 0x52:
                            Flag7=1;
                            {
                                               
                                ir_int = (uint8)*ch++;
                                ir_int = 0Xff- ir_int;
                                                                                                                 
                               
                            }
                            break;
                            
                      /*fixation leds(7) on & off commands to communicate with application*/      
                      case 0x49:
                            
                            
                            Flag7=0;
                           
                             
                           
                            {
                              io_led = (uint8)*ch++;
                                switch(io_led)
                                { 
                                    case 0x04:
                                              {
                                              
                                                                                          
                                                                                               
                                                PioSetMode(Fixation_databit_1, pio_mode_user);         /*00 ====================> State-0 */
                                                PioSetDir(Fixation_databit_1, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<Fixation_databit_1), pio_mode_strong_pull_up);
                                                PioSet(Fixation_databit_1,FALSE);
                                                
                                                PioSetMode(Fixation_databit_2, pio_mode_user);         /*00 ====================> State-0 */
                                                PioSetDir(Fixation_databit_2, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<Fixation_databit_2), pio_mode_strong_pull_up);
                                                PioSet(Fixation_databit_2,FALSE);
                                              }
                                              break;
                                              
                                    case 0x05:
                                              {
                                                                                              
                                                PioSetMode(Fixation_databit_1, pio_mode_user);         /*01 ====================> State-1 */
                                                PioSetDir(Fixation_databit_1, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<Fixation_databit_1), pio_mode_strong_pull_up);
                                                PioSet(Fixation_databit_1,FALSE);
                                                
                                                PioSetMode(Fixation_databit_2, pio_mode_user);         /*01 ====================> State-1 */
                                                PioSetDir(Fixation_databit_2, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<Fixation_databit_2), pio_mode_strong_pull_up);
                                                PioSet(Fixation_databit_2,TRUE);
                                              }
                                              break;
                                              
                                              /*D2-right led in a FIX LED board(8&9 commands to off and On*/
                                    case 0x06:
                                              {
                                                
                                                
                                              
                                                
                                                PioSetMode(Fixation_databit_1, pio_mode_user);         /*10 ====================> State-2 */
                                                PioSetDir(Fixation_databit_1, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<Fixation_databit_1), pio_mode_strong_pull_up);
                                                PioSet(Fixation_databit_1,TRUE);
                                                
                                                PioSetMode(Fixation_databit_2, pio_mode_user);         /*10 ====================> State-2 */
                                                PioSetDir(Fixation_databit_2, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<Fixation_databit_2), pio_mode_strong_pull_up);
                                                PioSet(Fixation_databit_2,FALSE);
                                              }
                                              break; 
                                        
                                      case 0x07:
                                              {
                                                                                                
                                                
                                                PioSetMode(Fixation_databit_1, pio_mode_user);         /*11 ====================> State-0 */
                                                PioSetDir(Fixation_databit_1, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<Fixation_databit_1), pio_mode_strong_pull_up);
                                                PioSet(Fixation_databit_1,TRUE);
                                                
                                                PioSetMode(Fixation_databit_2, pio_mode_user);         /*11 ====================> State-0 */
                                                PioSetDir(Fixation_databit_2, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<Fixation_databit_2), pio_mode_strong_pull_up);
                                                PioSet(Fixation_databit_2,TRUE);
                                              }
                                              break;
                                              
                                          case 0x09:
                                              { 
                                                PioSetMode(iphone_bat_lev_low, pio_mode_user);         /*25% 0f iPhone Battery*/
                                                PioSetDir(iphone_bat_lev_low, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<iphone_bat_lev_low), pio_mode_strong_pull_up);
                                                PioSet(iphone_bat_lev_low,TRUE);
                                              }
                                              break;
                                              
                                          case 0x0a:
                                              { 
                                                PioSetMode(iphone_bat_lev_low, pio_mode_user);         /*35% 0f iPhone Battery*/
                                                PioSetDir(iphone_bat_lev_low, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<iphone_bat_lev_low), pio_mode_strong_pull_up);
                                                PioSet(iphone_bat_lev_low,FALSE);
                                              }
                                              break;
                                       
                                     default: 
                                              break;
                                           tim_tidf = TimerCreate(250*MILLISECOND,TRUE,Handler_blk); 
                                              
                                }
                              
                                
                                if(I2CAcquire())
                                {
                                    I2CcommsInit();
                                    I2C_IO_Write(reg);
                                    I2CRelease(); 
                                    TimeDelayUSec(2* MILLISECOND);
                                    
                                }
                                
                                
                            }
                            break;
                            
                            /*Working distance leds(2) on & off commands to communicate with application*/
                      
                         
                 }                /*if((*ch++)==0x57)
                {
                    w_int = (uint8)*ch++;
                    w_int = 0Xff- w_int;
                    
                 }
                else if((*ch++)==0x52)
                {
                    ir_int = (uint8)*ch++;
                    ir_int = 0Xff- ir_int;
                }*/
            }
            
          
           
            if((!Flag5)&&(Flag7))
            {
          if(!Flag4)
          {  
            if(mode == MODE1)
            {
                VFPWMStart(ir_int,w_int);
                /*reg=0x00;
                if(I2CAcquire())
                {
                   I2CcommsInit();
                   I2C_IO_Write(reg);
                   I2CRelease(); 
                   TimeDelayUSec(2* MILLISECOND);
                 }*/
                /*Flag7=0;
                    Flag*/
                Flag7=1;
            }
            else if(mode == MODE2)
            {
                       
               VFPWMStart(ir_int,w_int);
               Flag7=1;
                           
            }
          }
         }

        
            }
            
     
            /* Received GATT ACCESS IND with write access */
            if(p_event_data->flags ==
                (ATT_ACCESS_WRITE |
                 ATT_ACCESS_PERMISSION |
                 ATT_ACCESS_WRITE_COMPLETE))
            { 
                /* Reset the idle timer */
                resetIdleTimer();
                
                HandleAccessWrite(p_event_data);
            }
            /* Received GATT ACCESS IND with read access */
            else if(p_event_data->flags ==
                (ATT_ACCESS_READ |
                ATT_ACCESS_PERMISSION))
            {
                /* Reset the idle timer */
                resetIdleTimer();
                
                HandleAccessRead(p_event_data);
            }
            else
            {
                /* No other request is supported */
                GattAccessRsp(p_event_data->cid, p_event_data->handle,
                              gatt_status_request_not_supported,
                              0, NULL);
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLmDisconnectComplete
 *
 *  DESCRIPTION
 *      This function handles LM Disconnect Complete event which is received
 *      at the completion of disconnect procedure triggered either by the
 *      device or remote host or because of link loss.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by LM_EV_DISCONNECT_COMPLETE
 *                              signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalLmDisconnectComplete(
                HCI_EV_DATA_DISCONNECT_COMPLETE_T *p_event_data)
{
#ifdef ENABLE_OTA
    if(OtaResetRequired())
    {
        OtaReset();
        /* The OtaReset function does not return */
    }
    else
#endif /* ENABLE_OTA */
    {
        /* Set UCID to INVALID_UCID */
        g_app_data.st_ucid = GATT_INVALID_UCID;

        /* LM_EV_DISCONNECT_COMPLETE event can have following disconnect 
         * reasons:
         *
         * HCI_ERROR_CONN_TIMEOUT - Link Loss case
         * HCI_ERROR_CONN_TERM_LOCAL_HOST - Disconnect triggered by device
         * HCI_ERROR_OETC_* - Other end (i.e., remote host) terminated 
         * connection
         */
        /* Handle signal as per current state */
        switch(g_app_data.state)
        {
            case app_state_connected:
                /* Initialise Application data instance */
                appDataInit();
                /* FALLTHROUGH */

            case app_state_disconnecting:
            {   
                                               
                                                
                                                PioSetMode(iphone_bat_lev_low, pio_mode_user);         /* relay became NC posiopon */
                                                PioSetDir(iphone_bat_lev_low, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<iphone_bat_lev_low), pio_mode_strong_pull_up);
                                                PioSet(iphone_bat_lev_low,FALSE);
                                                
                                                PioSetMode(Fixation_databit_1, pio_mode_user);         /*000 ====================> State-0 */
                                                PioSetDir(Fixation_databit_1, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<Fixation_databit_1), pio_mode_strong_pull_up);
                                                PioSet(Fixation_databit_1,FALSE);
                                                
                                                PioSetMode(Fixation_databit_2, pio_mode_user);         /*000 ====================> State-0 */
                                                PioSetDir(Fixation_databit_2, PIO_DIRECTION_OUTPUT);
                                                PioSetPullModes((1UL<<Fixation_databit_2), pio_mode_strong_pull_up);
                                                PioSet(Fixation_databit_2,FALSE);                       
                                                
                /* Link Loss Case */
                if(p_event_data->reason == HCI_ERROR_CONN_TIMEOUT)
                {
                    /* If advertising count is less than retry
                     * attempts,start advertisements 
                     */
                    if(g_adv_count<MAX_RETRY_ADVERT_ATTEMPT) 
                    {
                        /* Start either directed or fast advertisment */        
                        appStartAdvert();
                        
                        /* Increment the number of times we have started
                         * advertisements in case of link loss
                         */ 
                        g_adv_count++;
                    }
                    else
                    {
                        /* Clear the UART buffers with existing data */
                        emptySendRecvBuffers();
                        
                        /* Reset the advertisment count */                  
                        g_adv_count = 0;
                        
                        /* Set the app to idle state */
                        AppSetState(app_state_idle);
                    }
                }
                else if(p_event_data->reason == HCI_ERROR_CONN_TERM_LOCAL_HOST)
                {
                    if(g_app_data.state == app_state_connected)
                    {
                        /* It is possible to receive LM_EV_DISCONNECT_COMPLETE
                         * event in app_state_connected state at the expiry of
                         * lower layers ATT/SMP timer leading to disconnect
                         */
                        /* Move to app_state_fast_advertising state */
                        AppSetState(app_state_fast_advertising);
                    }
                    else
                    {
                        /* Case when application has triggered disconnect */
                        if(g_app_data.bonded)
                        {
                            /* Move to idle state */
                            AppSetState(app_state_idle);
                            
                            /* Clear the UART buffers with existing data */
                            emptySendRecvBuffers();
                        }
                        else /* Case of Bonding/Pairing removal */
                        {
                            /* Clear the UART buffers with existing data */
                            emptySendRecvBuffers();

                            /* Start undirected advertisements by moving to
                             * app_state_fast_advertising state
                             */
                            AppSetState(app_state_fast_advertising);
                        }
                    }
                }
                else /* Remote user terminated connection case */
                {
                    /* If the device has not bonded but disconnected, it may
                     * just have discovered the services supported by the
                     * application or read some un-protected characteristic
                     * value like device name and disconnected. The application
                     * should be connectable because the same remote device may
                     * want to reconnect and bond. If not the application
                     * should be discoverable by other devices.
                     */
                    if(!g_app_data.bonded)
                    {
                        /* Clear the UART buffers with existing data */
                        emptySendRecvBuffers();
                        
                        AppSetState( app_state_fast_advertising);
                    }
                    /* Case when disconnect is triggered by a bonded Host */
                    else 
                    {
                        /* Clear the UART buffers with existing data */
                        emptySendRecvBuffers();
                        
                        AppSetState( app_state_idle);
                    }
                }
            }
            break;

            default:
                /* Control should never come here */
                ReportPanic(app_panic_invalid_state);
            break;
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLsRadioEventInd
 *
 *  DESCRIPTION
 *      This function handles LS_RADIO_EVENT_IND event
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by LS_RADIO_EVENT_IND signal
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void handleSignalLsRadioEventInd(LS_RADIO_EVENT_IND_T * p_event_data)
{
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_connected:
        {
            /* Check whether the channel is free to send data */
            if( p_event_data->radio == radio_event_tx_data )
            {
                g_radio_event_configured = FALSE;
                
                /* Disable further radio events. */
                LsRadioEventNotification(p_event_data->cid, radio_event_none);
        
                /* Start sending Notifications again */
                ProcessRxData();
            }
        }
        break;
        
        default:
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleLED
 *
 *  DESCRIPTION
 *      This function plays the LED patterns.
 *
 *  PARAMETERS
 *      timer_id
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleLED(timer_id tid)
{
    if(g_app_data.led_timer_tid  == tid)
    {
       g_app_data.led_timer_tid  = TIMER_INVALID;

        /* Handle signal as per current state */
        switch(g_app_data.state)
        {
           case app_state_init:
           case app_state_directed_advertising:
           case app_state_fast_advertising:  /* FALLTHROUGH */
           case app_state_slow_advertising:
            {
                if(g_turn_on_led_advert)
                {
                    /* Turn on the LED */
                    /*TurnOnLED(TRUE);*/
                    g_turn_on_led_advert = FALSE;
                }
                else
                {
                    /* Turn off the LED */
                    /*TurnOnLED(FALSE);*/
                    g_turn_on_led_advert = TRUE;
                }

                /* Recreate advertising LED timer */
                g_app_data.led_timer_tid = TimerCreate(
                                                  TIMER_ADV_ON_OFF_DURATION,
                                                  TRUE,
                                                  handleLED);
                break;
            }
            case app_state_connected:
            {
                const uint16 mvs1 = AioRead(1);
                if(g_turn_on_led_connect)
                {
                    /* Turn on the LED */
                    g_turn_on_led_connect = FALSE;
                    /*TurnOnLED(TRUE);*/
                    
                    /* Recreate connected LED timer */
                    g_app_data.led_timer_tid = TimerCreate(
                                                  TIMER_CONNECTED_ON_DURATION,
                                                  TRUE,
                                                  handleLED);
         
            
             
         if(Adp ) 
            {
            
            device_battery = 0x65;/*charging*/
            GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&device_battery); 
            
            }
         
        else
            {
            if ((mvs1>=1300)) /*1245 100%*/
         {
            device_battery = 0x64;/*0x64*/
            GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&device_battery); 
         }    
          
        else if((mvs1<=1295) && (mvs1 >=1275))  /*85%*/
         {
            device_battery = 0x55;
            GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&device_battery); 
         }
         
                
         else if((mvs1<=1270) &&(mvs1 >=1250))  /* 70%*/
        { 
            device_battery = 0x46;
            GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&device_battery); 
        }
                
         else if((mvs1<=1245) &&(mvs1 >=1225))  /* 55%*/
        { 
            device_battery = 0x37;
            GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&device_battery);
        }
              
         else if((mvs1<=1220) &&(mvs1 >=1200))  /* 40%*/
        { 
            device_battery = 0x28;
            GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&device_battery); 
        }
         
         else if((mvs1<=1195) &&(mvs1 >=1175))  /* 25%*/
        { 
            device_battery = 0x19;
            GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&device_battery); 
        }
        
         else if((mvs1<=1170) &&(mvs1 >=1150))  /* 5%*/
        { 
            device_battery = 0x05;
            GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&device_battery); 
        }
                 
         else if ((mvs1<=1140))  /* 1%*/
         { 
            device_battery = 0x01;
            GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&device_battery); 
         } 
     }
                 
             }
                else
                {
                    /* Turn off the LED */
                    g_turn_on_led_connect = TRUE;
                    /*TurnOnLED(FALSE);*/

                    /* Recreate connected LED timer */
                    g_app_data.led_timer_tid = TimerCreate(
                                                 TIMER_CONNECTED_OFF_DURATION,
                                                 TRUE,
                                                 handleLED);
                }
                break;
            }
            case app_state_idle:
            case app_state_disconnecting:
            {
                /* Turn off the LED */
                /*urnOnLED(FALSE);*/

                /* Delete the timer */
                TimerDelete(g_app_data.led_timer_tid);
                g_app_data.led_timer_tid = TIMER_INVALID;
                break;
            }
        }
    }
}


/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/
#ifdef NVM_TYPE_FLASH
/*----------------------------------------------------------------------------*
 *  NAME
 *      WriteApplicationAndServiceDataToNVM
 *
 *  DESCRIPTION
 *      This function writes the application data to NVM. This function should 
 *      be called on getting nvm_status_needs_erase
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void WriteApplicationAndServiceDataToNVM(void)
{
    uint16 nvm_sanity = 0xffff;
    nvm_sanity = NVM_SANITY_MAGIC;

    /* Write NVM sanity word to the NVM */
    Nvm_Write(&nvm_sanity, sizeof(nvm_sanity), NVM_OFFSET_SANITY_WORD);

    /* Write bonded status to NVM */
    Nvm_Write((uint16*)&g_app_data.bonded,
              sizeof(g_app_data.bonded),
              NVM_OFFSET_BONDED_FLAG);
    
    /* Write typed bd address of bonded host */
    Nvm_Write((uint16*)&g_app_data.bonded_bd_addr,
                       sizeof(TYPED_BD_ADDR_T),
                       NVM_OFFSET_BONDED_ADDR);


    /* Write the diversifier to NVM */
    Nvm_Write(&g_app_data.diversifier,
               sizeof(g_app_data.diversifier),
               NVM_OFFSET_SM_DIV);

#ifdef ENABLE_OTA
    /* Write Gatt service data into NVM. */
    WriteGattServiceDataInNvm();
#endif /* ENABLE_OTA */    

    /* Write Serial service data into NVM */
    WriteSerialServiceDataInNvm();

    /* Write Battery service data into NVM */
    WriteBatteryServiceDataInNvm();
}
#endif /* NVM_TYPE_FLASH */

/*----------------------------------------------------------------------------*
 *  NAME
 *      ReportPanic
 *
 *  DESCRIPTION
 *      This function calls firmware panic routine and gives a single point
 *      of debugging any application level panics.
 *
 *  PARAMETERS
 *      panic_code [in]         Code to supply to firmware Panic function.
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void ReportPanic(app_panic_code panic_code)
{
    /* Raise panic */
    Panic(panic_code);
}

extern void HandleExtTrigger(void)
{
    uint8 temp= 0x43;
    reg=0x00;
    if(I2CAcquire())
    {
       I2CcommsInit();
       I2C_IO_Write(reg);
       I2CRelease(); 
       TimeDelayUSec(2* MILLISECOND);
     }
    if( mode == 0x31)
    {
        /*WLED_PWM(w_int);
        IR_PWM(0xFF);  */   
        /*VFPWMStart(0xFF,w_int);*/
        GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&temp); 
    }
    else if(mode == 0x32)
    {
        /*VFPWMStart(0xFF,w_int);*/
        /*WLED_PWM(w_int);
        IR_PWM(0xFF);*/
        GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&temp);
    }
        
    
    
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandleShortButtonPress
 *
 *  DESCRIPTION
 *      This function contains handling of short button press.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HandleShortButtonPress(void)
{
    /*uint8 temp[2]={0x47,0x02};*/
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_idle:
            appStartAdvert();
        break;
        
        
        case app_state_connected:
                /*if( mode == 0x31)
                {
                    WLED_PWM(w_int);
                    IR_PWM(0xFF);
                    VFPWMStart(0xFF,w_int);
                    GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&temp[0]);
                }
                else if(mode == 0x32)
                {
                    WLED_PWM(0xff);
                    IR_PWM(ir_int);
                    VFPWMStart(ir_int,0xFF)
                    GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&temp[0]);
                }*/
            
        break; 

        default:
            /* Ignore in remaining states */
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandleLongButtonPress
 *
 *  DESCRIPTION
 *      This function contains handling of short button press. If connected,
 *      the device disconnects from the connected host else it triggers
 *      advertisements.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HandleLongButtonPress(void)
{
    /*uint8 temp[2]={0x00,0x02};*/
    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_connected:
            /* Disconnect from the connected host */
            /*AppSetState(app_state_disconnecting);*/
            if( mode == 0x31)
            {
                /*WLED_PWM(w_int);
                IR_PWM(0xFF);*/
              if (Flag7)
            {
                VFPWMStart(ir_int,w_int);
              
              
            }
                /*GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&temp[0]); */
            }
            else if(mode == 0x32)
            { 
                
                /*WLED_PWM(0xff);
                IR_PWM(ir_int);*/
              if (Flag7)
              {
             VFPWMStart(ir_int,w_int);
                   
              }
                 /*GattCharValueNotification(GetConnectionID(),HANDLE_SERIAL_DATA_TRANSFER,1,&temp[1]);*/
            
            /* As per the specification Vendor may choose to initiate the
             * idle timer which will eventually initiate the disconnect.
             */
          }
        break;
        default:
            /* Ignore in remaining states */
        break;

    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IsAppWaitingForRadioEvent
 *
 *  DESCRIPTION
 *      This function checks if the application is waiting for a Radio event to 
 *      resume data transfer.
 *
 *  RETURNS
 *      TRUE if yes..
 *
 *----------------------------------------------------------------------------*/
extern bool IsAppWaitingForRadioEvent(void)
{
    return g_radio_event_configured;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppSetState
 *
 *  DESCRIPTION
 *      This function is used to set the state of the application.
 *
 *  PARAMETERS
 *      new_state [in]          State to move to
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void AppSetState(app_state new_state)
{
    /* Check that the new state is not the same as the current state */
    app_state old_state = g_app_data.state;

    if (old_state != new_state)
    {
        /* Exit current state */
        switch (old_state)
        {
            case app_state_init:
                appInitExit();
                PioSetMode(LED_CTD, pio_mode_user);         /*000 ====================> State-0 */
                PioSetDir(LED_CTD, PIO_DIRECTION_OUTPUT);
                PioSetPullModes((1UL<<LED_CTD), pio_mode_strong_pull_up);
                PioSet(LED_CTD,FALSE);                     /*Relay1= OFF*/
            break;

            case app_state_disconnecting:
                /* Common things to do whenever application exits
                 * app_state_disconnecting state.
                 */
                PioSetMode(LED_CTD, pio_mode_user);         /*000 ====================> State-0 */
                PioSetDir(LED_CTD, PIO_DIRECTION_OUTPUT);
                PioSetPullModes((1UL<<LED_CTD), pio_mode_strong_pull_up);
                PioSet(LED_CTD,FALSE);
                
               

                
                appDataInit();
            break;
            
            case app_state_directed_advertising:
            case app_state_fast_advertising:  /* FALLTHROUGH */
            case app_state_slow_advertising:
                /* Common things to do whenever application exits
                 * APP_*_ADVERTISING state.
                 */
                PioSetMode(LED_CTD, pio_mode_user);         /*000 ====================> State-0 */
                PioSetDir(LED_CTD, PIO_DIRECTION_OUTPUT);
                PioSetPullModes((1UL<<LED_CTD), pio_mode_strong_pull_up);
                PioSet(LED_CTD,FALSE);

                appExitAdvertising();
                
            break;

            case app_state_connected:
                /* The application may need to maintain the values of some
                 * profile specific data across connections and power cycles.
                 * These values would have changed in 'connected' state. So,
                 * update the values of this data stored in the NVM.
                 */
                PioSetMode(LED_CTD, pio_mode_user);         /*000 ====================> State-0 */
                PioSetDir(LED_CTD, PIO_DIRECTION_OUTPUT);
                PioSetPullModes((1UL<<LED_CTD), pio_mode_strong_pull_up);
                PioSet(LED_CTD,TRUE);

           
            break;

            case app_state_idle:
                PioSetMode(LED_CTD, pio_mode_user);        
                PioSetDir(LED_CTD, PIO_DIRECTION_OUTPUT);
                PioSetPullModes((1UL<<LED_CTD), pio_mode_strong_pull_up);
                PioSet(LED_CTD,FALSE);
                
                 PioSetMode(iphone_bat_lev_low, pio_mode_user);        
                 PioSetDir(iphone_bat_lev_low, PIO_DIRECTION_OUTPUT);
                 PioSetPullModes((1UL<<iphone_bat_lev_low), pio_mode_strong_pull_up);
                 PioSet(iphone_bat_lev_low,FALSE);

                
            break;

            default:
                /* Nothing to do */
            break;
        }
        
        /* Set new state */
        g_app_data.state = new_state;

        /* Enter new state */
        switch (new_state)
        {
            case app_state_directed_advertising:
            {
                /* Start directed advertisement */
                GattStartAdverts(FALSE,gap_mode_connect_directed);
                
                if( g_app_data.led_timer_tid  == TIMER_INVALID)
                {
                    /* Recreate 500ms timer */
                    g_app_data.led_timer_tid = TimerCreate(
                                                 TIMER_ADV_ON_OFF_DURATION,
                                                 TRUE,
                                                 handleLED);
                }
            }
            break;
            case app_state_fast_advertising:
            {
                /* Enable white list if application is bonded to some remote
                 * device and that device is not using resolvable random
                 * address.
                 */
                enableWhiteList();
                /* Trigger fast advertisements. */
                GattTriggerFastAdverts(&g_app_data.bonded_bd_addr);
                
                if( g_app_data.led_timer_tid  == TIMER_INVALID)
                {
                    /* Recreate 500ms timer */
                    g_app_data.led_timer_tid = TimerCreate(
                                                 TIMER_ADV_ON_OFF_DURATION,
                                                 TRUE,
                                                 handleLED);
                }
            }
            break;

            case app_state_slow_advertising:
                /* Start slow advertisements */
                GattStartAdverts(FALSE,gap_mode_connect_undirected);
   
                if( g_app_data.led_timer_tid  == TIMER_INVALID)
                {
                    /* Recreate 500ms timer */
                    g_app_data.led_timer_tid = TimerCreate(
                                                 TIMER_ADV_ON_OFF_DURATION,
                                                 TRUE,
                                                 handleLED);
                }
            break;

            case app_state_idle:
            break;

            case app_state_connected:
            {
                /* Common things to do whenever application enters
                 * app_state_connected state.
                 */
                /* Trigger SM Slave Security request only if the remote
                 * host is not using resolvable random address
                 */
                if(!GattIsAddressResolvableRandom(&g_app_data.con_bd_addr))
                {
                    SMRequestSecurityLevel(&g_app_data.con_bd_addr);
                }

                /* Reset the idle timer */
                resetIdleTimer();
                
                if( g_app_data.led_timer_tid  == TIMER_INVALID)
                {
                    /* Recreate 500ms timer */
                    g_app_data.led_timer_tid = TimerCreate(
                                                 TIMER_CONNECTED_ON_DURATION,
                                                 TRUE,
                                                 handleLED);
                }
             }
            break;

            case app_state_disconnecting:
                /* Disconnect the link */
                GattDisconnectReq(g_app_data.st_ucid);
            break;

            default:
            break;
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppGetState
 *
 *  DESCRIPTION
 *      This function returns the current state of the application.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Current application state
 *----------------------------------------------------------------------------*/
extern app_state AppGetState(void)
{
    return g_app_data.state;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IsWhiteListEnabled
 *
 *  DESCRIPTION
 *      This function returns whether white list is enabled or not.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      TRUE if white list is enabled, FALSE otherwise.
 *----------------------------------------------------------------------------*/
extern bool IsWhiteListEnabled(void)
{
    return g_app_data.enable_white_list;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandlePairingRemoval
 *
 *  DESCRIPTION
 *      This function contains pairing removal handling.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HandlePairingRemoval(void)
{
    /* Remove bonding information */
    /* The device will no longer be bonded */
    g_app_data.bonded = FALSE;

    /* Write bonded status to NVM */
    Nvm_Write((uint16*)&g_app_data.bonded,
              sizeof(g_app_data.bonded),
              NVM_OFFSET_BONDED_FLAG);

    switch(g_app_data.state)
    {
        case app_state_connected:
        {
            /* Disconnect from the connected host before triggering
             * advertisements again for any host to connect. Application
             * and services data related to bonding status will get
             * updated while exiting disconnecting state.
             */
            AppSetState(app_state_disconnecting);

            /* Reset and clear the white list */
            LsResetWhiteList();
        }
        break;
        case app_state_directed_advertising:
        case app_state_fast_advertising:
        case app_state_slow_advertising:
        {

            /* Set flag for pairing / bonding removal */
            g_app_data.pairing_button_pressed = TRUE;

            /* Stop advertisements first as it may be making use of the white
             * list. Once advertisements are stopped, reset the white list
             * and trigger advertisements again for any host to connect.
             */
            GattStopAdverts();
        }
        break;

        case app_state_disconnecting:
        {
            /* Disconnect procedure is on-going, so just reset the white list
             * and wait for the procedure to complete before triggering
             * advertisements again for any host to connect. Application
             * and services data related to bonding status will get
             * updated while exiting disconnecting state.
             */
            LsResetWhiteList();
        }
        break;

        default: /* app_state_init / app_state_idle handling */
        {
            /* Reset and clear the white list */
            LsResetWhiteList();

            /* Start fast undirected advertisements */
            AppSetState(app_state_fast_advertising);
        }
        break;

    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      StartAdvertTimer
 *
 *  DESCRIPTION
 *      This function starts the advertisement timer.
 *
 *  PARAMETERS
 *      interval [in]           Timer duration, microseconds
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void StartAdvertTimer(uint32 interval)
{
    /* Cancel existing timer, if valid */
    if (g_app_data.app_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_data.app_tid);
    }

    /* Start advertisement timer  */
    g_app_data.app_tid = TimerCreate(interval, TRUE, appAdvertTimerHandler);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IsDeviceBonded
 *
 *  DESCRIPTION
 *      This function returns the status whether the connected device is
 *      bonded or not.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      TRUE if device is bonded, FALSE if not.
 *----------------------------------------------------------------------------*/
extern bool IsDeviceBonded(void)
{
    return g_app_data.bonded;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GetConnectionID
 *
 *  DESCRIPTION
 *      This function returns the connection identifier.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Connection identifier.
 *----------------------------------------------------------------------------*/
extern uint16 GetConnectionID(void)
{
    return g_app_data.st_ucid;
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      PioFastPwmEnable
 *
 *  DESCRIPTION
 *      This function enables/disables PWM.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
/*void PioFastPwmEnable(bool enable)
{
    if(enable)
    {
        
        PioCtrlrStart();
    }
    else
    {
        PioCtrlrStop();
        
    }
}*/




/*----------------------------------------------------------------------------*
 *  NAME
 *      PioFastPwmConfig
 *
 *  DESCRIPTION
 *      This function selects the PWM ports to be configured.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
/*void PioFastPwmConfig(uint32 pio_mask)
{
    PioSetModes(pio_mask,  pio_mode_pio_controller);
    PioSetPullModes(pio_mask, pio_mode_no_pulls);

    PioCtrlrInit((uint16*)&pio_ctrlr_code);
    PioCtrlrClock(TRUE);
}*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      PioFastPwmSetWidth
 *
 *  DESCRIPTION
 *      This function sets the required pulse width in multiples of 4us
 *      on a PWM port.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
/* bool PioFastPwmSetWidth(uint8 pwm_port, uint8 bright_width, uint8 dull_width,
                        bool inverted)
{
    if(pwm_port < PWM0_PORT || pwm_port > PWM7_PORT ||
       bright_width > 255 || dull_width > 255)
        return FALSE;

    uint16*address=PIO_CONTROLLER_DATA_WORD+((pwm_port-PWM0_PORT)>>1);

    if(pwm_port&1)
    {
        *address&=0x00ff;
        *address|=(bright_width<<8);
        address+=4;
        *address&=0x00ff;
        *address|=(dull_width<<8);
    }
    else
    {
        *address&=0xff00;
        *address|=bright_width;
        address+=4;
        *address&=0xff00;
        *address|=dull_width;
    }

    address=PIO_CONTROLLER_DATA_WORD+8;

    if(inverted)
        *address&=~(1<<(pwm_port-PWM0_PORT));
    else
        *address|=1<<(pwm_port-PWM0_PORT);
    return TRUE;
}*/
 /*----------------------------------------------------------------------------*
 *  NAME
 *      PioFastPwmSetPeriods
 *
 *  DESCRIPTION
 *      This function sets bright and dull periods for PWM. This is
 *      applicable for all PWM ports enabled.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
/*void PioFastPwmSetPeriods(uint16 bright, uint16 dull)
{
    *(PIO_CONTROLLER_DATA_WORD+9)=bright;
    *(PIO_CONTROLLER_DATA_WORD+10)=dull;
    *(PIO_CONTROLLER_DATA_WORD+11)=1; 
}*/


/* White LED PWM*/

/*void WLED_PWM(uint8 wled)
{
    PioFastPwmSetWidth(WHITE_LED,wled, 0xFF - wled, TRUE);
    PioFastPwmSetPeriods(1, 0);
    PioFastPwmEnable(TRUE);
}*/


/* IR LED PWM*/

/*void IR_PWM(uint8 ir)
{
    PioFastPwmSetWidth(IR_LED,ir, 0xFF - ir, TRUE);
    PioFastPwmSetPeriods(1, 0);
    PioFastPwmEnable(TRUE);
}*/

static uint8 writeASCIICodedNumber(uint32 value)
{
#define BUFFER_SIZE 11          /* Buffer size required to hold maximum value */
    
    uint8  i = BUFFER_SIZE;     /* Loop counter */
    uint32 remainder = value;   /* Remaining value to send */
    char   buffer[BUFFER_SIZE]; /* Buffer for ASCII string */

    /* Ensure the string is correctly terminated */    
    buffer[--i] = '\0';
    
    /* Loop at least once and until the whole value has been converted */
    do
    {
        /* Convert the unit value into ASCII and store in the buffer */
        buffer[--i] = (remainder % 10) + '0';
        
        /* Shift the value right one decimal */
        remainder /= 10;
    } while (remainder > 0);

    /* Send the string to the UART */
    /*DebugWriteString(buffer + i);*/
    
    /* Return length of ASCII string sent to UART */
    return (BUFFER_SIZE - 1) - i;
}



static void Led_handler(timer_id const id)
{
    if(F2)
    {
        if(!F1)
        {
            F1=1;
            PioSetMode(RLED, pio_mode_user);         
            PioEnablePWM(2, FALSE);
            PioSetDir(RLED, PIO_DIRECTION_OUTPUT);
            PioSetPullModes((1UL<<RLED), pio_mode_strong_pull_up);
            PioSet(RLED,FALSE);             
                 
                 
            PioSetMode(GLED, pio_mode_user);         
            PioEnablePWM(3, FALSE);
            PioSetDir(GLED, PIO_DIRECTION_OUTPUT);
            PioSetPullModes((1UL<<GLED), pio_mode_strong_pull_up);
            PioSet(GLED,TRUE); 
            
        }
        else if(F1)
        {
            F1=0;
            
            PioSetMode(RLED, pio_mode_user);         
            PioEnablePWM(2, FALSE);
            PioSetDir(RLED, PIO_DIRECTION_OUTPUT);
            PioSetPullModes((1UL<<RLED), pio_mode_strong_pull_up);
            PioSet(RLED,TRUE);             
                 
                 
            PioSetMode(GLED, pio_mode_user);         
            PioEnablePWM(3, FALSE);
            PioSetDir(GLED, PIO_DIRECTION_OUTPUT);
            PioSetPullModes((1UL<<GLED), pio_mode_strong_pull_up);
            PioSet(GLED,FALSE); 
        }
    }
    
    tim_tid1 = TimerCreate(250*MILLISECOND,TRUE,Led_handler);
}



static void handler(timer_id const id)
{
     const uint16 mvs = AioRead(1);
     /*DebugWriteString("\r\n Analoge Voltage = ");*/
     writeASCIICodedNumber(mvs);
     /*DebugWriteString("mV\n\r");*/
     	

     if(PioGet(ADP))
     {
         Adp =0;
     }
     else
     {
         Adp=1;
    
     }
     if(PioGet(CHG))
     {
         Chg =0;
     }
     else
     {
         Chg=1;
    
     }
     if(!Adp)
     {
         
          F1=0;
          F2=0;
          TimerDelete(tim_tid1);	
          tim_tid1 = TIMER_INVALID;
          

        
         
         if ((mvs>1245)&&(!Flag0)) /*>7.6*/
         {
             
         }                
         else if((mvs<=1240) &&(mvs >=1183)&&(!Flag0))  /* 7.6v - 7.2V*/
         {
             Flag0=1;
             Flag1=0; 
             Flag2=0;  
         }
         else if((mvs<=1175) && (mvs >=1140)&&(!Flag1))  /* 7.2v - 6.9V*/
         {
            Flag0=1;
            Flag1=1;
            Flag2=0;
         }
         else if((mvs<=1132) &&(mvs >=1120)&&!(Flag2))  /* 6.9v - 6.8V*/
        { 
             Flag0=1;
             Flag1=1;
             Flag2=1;
             /*Flag3=1;*/
        }
         
         /*if ((mvs<1118)&&(mvs>=1108)&&!(Flag2))  */  /* < 6.8V*/
        else if ((mvs<=1110)&&!(Flag4)) 
         { 
            Flag4=1;
            Flag1=1;
            Flag2=1;
            Flag0=1;
            Flag3=1;
         }  

         /* if ((mvs<=1108)&&!(Flag4))     
         { 
            Flag4=1;
            Flag1=1;
            Flag2=1;
            Flag0=1;
             Flag3=1;
         }*/
        
       
          
        
          if(!(Flag0)&&!(Flag1)&&!(Flag2)) 
           {
             PioSetMode(RLED, pio_mode_user);         
             PioEnablePWM(2, FALSE);
             PioSetDir(RLED, PIO_DIRECTION_OUTPUT);
             PioSetPullModes((1UL<<RLED), pio_mode_strong_pull_up);
             PioSet(RLED,FALSE);             
             
             
            PioSetMode(GLED, pio_mode_user);         
            PioEnablePWM(3, FALSE);
            PioSetDir(GLED, PIO_DIRECTION_OUTPUT);
            PioSetPullModes((1UL<<GLED), pio_mode_strong_pull_up);
            PioSet(GLED,TRUE); 
            }
           else if((Flag0)&&!(Flag1)&&!(Flag2))
           {
            
                PioSetMode(RLED, pio_mode_user);         
                PioEnablePWM(2, FALSE);
                PioSetDir(RLED, PIO_DIRECTION_OUTPUT);
                PioSetPullModes((1UL<<RLED), pio_mode_strong_pull_up);
                PioSet(RLED,FALSE);             
             
             
                PioSetDir(GLED, PIO_DIRECTION_OUTPUT);
                PioSetMode(GLED, pio_mode_pwm3);
                PioEnablePWM(3, TRUE);
                PioConfigPWM(3, pio_pwm_mode_push_pull,
                 0,33, 10,33,0, 10,33);  
            }   
           else if((Flag0)&&(Flag1)&&!(Flag2))
           {
               PioSetMode(GLED, pio_mode_user);         
               PioEnablePWM(3, FALSE);
               PioSetDir(GLED, PIO_DIRECTION_OUTPUT);
               PioSetPullModes((1UL<<GLED), pio_mode_strong_pull_up);
               PioSet(GLED,FALSE);            
             
             
               PioSetMode(RLED, pio_mode_user);         
               PioEnablePWM(2, FALSE);
               PioSetDir(RLED, PIO_DIRECTION_OUTPUT);
               PioSetPullModes((1UL<<RLED), pio_mode_strong_pull_up);
               PioSet(RLED,TRUE);      
           } 
           else if((Flag0)&&(Flag1)&&(Flag2))
           {
               if(!Flag4)
               {
                   
                   PioSetMode(GLED, pio_mode_user);         
                   PioEnablePWM(3, FALSE);
                   PioSetDir(GLED, PIO_DIRECTION_OUTPUT);
                   PioSetPullModes((1UL<<GLED), pio_mode_strong_pull_up);
                   PioSet(GLED,FALSE);            
                 
                 
                   PioSetDir(RLED, PIO_DIRECTION_OUTPUT);
                   PioSetMode(RLED, pio_mode_pwm2);
                   PioEnablePWM(2, TRUE);
                   PioConfigPWM(2, pio_pwm_mode_push_pull,
                     0,33, 10,33,0, 10,33);
               }
               else if(Flag4)
               {
                   
                   VFPWMStart(0xFF,0x00);      
                   
                   PioSetMode(LED_CTD, pio_mode_user);         /*000 ====================> State-0 */
                   PioSetDir(LED_CTD, PIO_DIRECTION_OUTPUT);
                   PioSetPullModes((1UL<<LED_CTD), pio_mode_strong_pull_up);
                   PioSet(LED_CTD,FALSE);
                                 
                   
                   PioSetMode(GLED, pio_mode_user);         
                   PioEnablePWM(3, FALSE);
                   PioSetDir(GLED, PIO_DIRECTION_OUTPUT);
                   PioSetPullModes((1UL<<GLED), pio_mode_strong_pull_up);
                   PioSet(GLED,FALSE); 
                   
                   
                   PioSetMode(RLED, pio_mode_user);         
                   PioEnablePWM(2, FALSE);
                   PioSetDir(RLED, PIO_DIRECTION_OUTPUT);
                   PioSetPullModes((1UL<<RLED), pio_mode_strong_pull_up);
                   PioSet(RLED,FALSE);
                   
                   
                   
                   if (g_app_data.app_tid != TIMER_INVALID)
                    {
                        TimerDelete(g_app_data.app_tid);
                        g_app_data.app_tid = TIMER_INVALID;
                    }
                
                    /* Start the Idle timer again.*/
                    g_app_data.app_tid  = TimerCreate(10*MILLISECOND,
                                                    TRUE, appIdleTimerHandler);
                
                }
           }
           
          F1=0;
          F2=0;
          TimerDelete(tim_tid1);	
          tim_tid1 = TIMER_INVALID; 
          
     }
     else if(Adp)
     {
         Flag0=0;
         Flag1=0;
         Flag2=0;
         Flag3=0;
         Flag4=0;
         
         
            
         
               if(!F2)
               { 
                 F2=1;
                tim_tid1 = TimerCreate(10*MILLISECOND,TRUE,Led_handler);
            }
     }
     
     
    tim_tid = TimerCreate(1*SECOND,TRUE,handler);
}





/*----------------------------------------------------------------------------*
 *  NAME
 *      AppPowerOnReset
 *
 *  DESCRIPTION
 *      This user application function is called just after a power-on reset
 *      (including after a firmware panic), or after a wakeup from Hibernate or
 *      Dormant sleep states.
 *
 *      At the time this function is called, the last sleep state is not yet
 *      known.
 *
 *      NOTE: this function should only contain code to be executed after a
 *      power-on reset or panic. Code that should also be executed after an
 *      HCI_RESET should instead be placed in the AppInit() function.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void AppPowerOnReset(void)
{
    /* Code that is only executed after a power-on reset or firmware panic
     * should be implemented here - e.g. configuring application constants
     */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppInit
 *
 *  DESCRIPTION
 *      This user application function is called after a power-on reset
 *      (including after a firmware panic), after a wakeup from Hibernate or
 *      Dormant sleep states, or after an HCI Reset has been requested.
 *
 *      NOTE: In the case of a power-on reset, this function is called
 *      after AppPowerOnReset().
 *
 *  PARAMETERS
 *      last_sleep_state [in]   Last sleep state
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void AppInit(sleep_state last_sleep_state)
{
    uint16 gatt_db_length = 0;  /* GATT database size */
    uint16 *p_gatt_db = NULL;   /* GATT database */

     
     
    /* Initialise the Serial Server application state */
    g_app_data.state = app_state_init;

    /* Initialise the application timers */
    TimerInit(MAX_APP_TIMERS, (void*)app_timers);

    /* Initialise local timers */
    TimerDelete(g_app_data.app_tid);
    g_app_data.app_tid = TIMER_INVALID;
        
    TimerDelete(g_app_data.bonding_reattempt_tid );
    g_app_data.bonding_reattempt_tid = TIMER_INVALID;
 
    /* Initialise GATT entity */
    GattInit();

    /* Initialise Serial Server H/W */
    InitHardware();

    /* Install GATT Server support for the optional Write procedure.
     * This is mandatory only if the control point characteristic is supported.
     */
    GattInstallServerWrite();

#ifdef NVM_TYPE_EEPROM
    /* Configure the NVM manager to use I2C EEPROM for NVM store */
    NvmConfigureI2cEeprom();
#elif NVM_TYPE_FLASH
    /* Configure the NVM Manager to use SPI flash for NVM store. */
    NvmConfigureSpiFlash();
#endif /* NVM_TYPE_EEPROM */

    Nvm_Disable();

    /* Battery initialisation on chip reset */
    BatteryInitChipReset();

    /* Serial Service initialisation on chip reset */
    /*SerialInitChipReset();*/

    /* Read persistent storage */
    readPersistentStore();

    /* Tell Security Manager module what value it needs to initialise its
     * diversifier to.
     */
    SMInit(g_app_data.diversifier);

    /* Initialise hardware data */
    HwDataInit();

    /* Initialise application data structure */
    appDataInit();

    /* Tell GATT about our database. We will get a GATT_ADD_DB_CFM event when
     * this has completed.
     */
    p_gatt_db = GattGetDatabase(&gatt_db_length);
    GattAddDatabaseReq(gatt_db_length, p_gatt_db);
    
    /*DebugInit(1, NULL, NULL);
    DebugWriteString("Hello \n\r");*/
    /*PioFastPwmConfig(PIO_BIT_MASK(IR_LED) | PIO_BIT_MASK(WHITE_LED));
    PioFastPwmEnable(TRUE);*/
    VFPWMConfig(1,1,2,TRUE);
    PioSetModes(PIO_BIT_MASK(WHITE_LED)|PIO_BIT_MASK(IR_LED),pio_mode_pio_controller);
    
    SleepModeChange(sleep_mode_never );
    /*VFPWMStart(0xFF,0x00);*/
    /*VFPWMStart(0x00,0xFF);*/
    
    
    mode = MODE1;
    
/*    WLED_PWM(0xFF);
    IR_PWM(0xFF);*/
 
    PioSetDir(ADP, PIO_DIRECTION_INPUT);
    PioSetMode(ADP_PIO_MASK, pio_mode_user);
    PioSetPullModes((ADP_PIO_MASK), pio_mode_weak_pull_up);
    PioSetEventMask((ADP_PIO_MASK), pio_event_mode_both);
    
    
    PioSetDir(CHG, PIO_DIRECTION_INPUT);
    PioSetMode(CHG_PIO_MASK, pio_mode_user);
    PioSetPullModes((CHG_PIO_MASK), pio_mode_weak_pull_up);
    PioSetEventMask((CHG_PIO_MASK), pio_event_mode_both);
    

    /* Pull up the */
    
    
    if(I2CAcquire())
    {
      I2CcommsInit();
      I2C_IO_Write(0x00);
      I2CRelease(); 
      TimeDelayUSec(2* MILLISECOND);
    }
    tim_tid = TimerCreate(2*SECOND,TRUE,handler);
       
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 *  PARAMETERS
 *      id   [in]               System event ID
 *      data [in]               Event data
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppProcessSystemEvent(sys_event_id id, void *data)
{
    switch(id)
    {
        case sys_event_battery_low:
        {
            /* Battery low event received - notify the connected host. If
             * not connected, the battery level will get notified when
             * device gets connected again
             */
            if(g_app_data.state == app_state_connected)
            {
                BatteryUpdateLevel(g_app_data.st_ucid);
            }
        }
        break;

        case sys_event_pio_changed:
        {
             /* Handle the PIO changed event. */
             HandlePIOChangedEvent((pio_changed_data*)data);
        }
        break;

        default:
            /* Ignore anything else */
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event
 *      is received by the system.
 *
 *  PARAMETERS
 *      event_code [in]         LM event ID
 *      event_data [in]         LM event data
 *
 *  RETURNS
 *      TRUE if the app has finished with the event data; the control layer
 *      will free the buffer.
 *----------------------------------------------------------------------------*/
bool AppProcessLmEvent(lm_event_code event_code, LM_EVENT_T *p_event_data)
{
    switch (event_code)
    {
        /* Handle events received from Firmware */
        case GATT_ADD_DB_CFM:
            /* Attribute database registration confirmation */
            handleSignalGattAddDbCfm((GATT_ADD_DB_CFM_T*)p_event_data);
        break;

        case GATT_CANCEL_CONNECT_CFM:
            /* Confirmation for the completion of GattCancelConnectReq()
             * procedure
             */
            handleSignalGattCancelConnectCfm();
        break;

        case GATT_CONNECT_CFM:
            /* Confirmation for the completion of GattConnectReq()
             * procedure
             */
            handleSignalGattConnectCfm((GATT_CONNECT_CFM_T*)p_event_data);
        break;

        case SM_KEYS_IND:
            /* Indication for the keys and associated security information
             * on a connection that has completed Short Term Key Generation
             * or Transport Specific Key Distribution
             */
            handleSignalSmKeysInd((SM_KEYS_IND_T *)p_event_data);
        break;


        case SM_PAIRING_AUTH_IND:
            /* Authorise or Reject the pairing request */
            handleSignalSmPairingAuthInd((SM_PAIRING_AUTH_IND_T*)p_event_data);
        break;

        case SM_SIMPLE_PAIRING_COMPLETE_IND:
            /* Indication for completion of Pairing procedure */
            handleSignalSmSimplePairingCompleteInd(
                (SM_SIMPLE_PAIRING_COMPLETE_IND_T *)p_event_data);
        break;

        case LM_EV_ENCRYPTION_CHANGE:
            /* Indication for encryption change event */
            handleSignalLMEncryptionChange(
            (HCI_EV_DATA_ENCRYPTION_CHANGE_T *)&p_event_data->enc_change.data);
        break;


         case SM_DIV_APPROVE_IND:
            /* Indication for SM Diversifier approval requested by F/W when
             * the last bonded host exchange keys. Application may or may not
             * approve the diversifier depending upon whether the application
             * is still bonded to the same host
             */
            handleSignalSmDivApproveInd((SM_DIV_APPROVE_IND_T *)p_event_data);
        break;

        case GATT_ACCESS_IND:
            /* Indicates that an attribute controlled directly by the
             * application (ATT_ATTR_IRQ attribute flag is set) is being
             * read from or written to.
             */
            PioSetMode(LED_CTD, pio_mode_user);         /*000 ====================> State-0 */
            PioSetDir(LED_CTD, PIO_DIRECTION_OUTPUT);
            PioSetPullModes((1UL<<LED_CTD), pio_mode_strong_pull_up);
            PioSet(LED_CTD,TRUE);
            handleSignalGattAccessInd((GATT_ACCESS_IND_T *)p_event_data);
        break;

        case GATT_DISCONNECT_IND:
            /* Disconnect procedure triggered by remote host or due to
             * link loss is considered complete on reception of
             * LM_EV_DISCONNECT_COMPLETE event. So, it gets handled on
             * reception of LM_EV_DISCONNECT_COMPLETE event.
             */
            PioSetMode(LED_CTD, pio_mode_user);         /*000 ====================> State-0 */
            PioSetDir(LED_CTD, PIO_DIRECTION_OUTPUT);
            PioSetPullModes((1UL<<LED_CTD), pio_mode_strong_pull_up);
            PioSet(LED_CTD,FALSE);   
            VFPWMStart(0xFF,0x00);
             HandlePairingRemoval();
             /*WLED_PWM(0xFF);
             IR_PWM(0xFF);*/
        break;

        case GATT_DISCONNECT_CFM:
            /* Confirmation for the completion of GattDisconnectReq()
             * procedure is ignored as the procedure is considered complete
             * on reception of LM_EV_DISCONNECT_COMPLETE event. So, it gets
             * handled on reception of LM_EV_DISCONNECT_COMPLETE event.
             */
        break;

        case LM_EV_DISCONNECT_COMPLETE:
        {
            /* Disconnect procedures either triggered by application or remote
             * host or link loss case are considered completed on reception
             * of LM_EV_DISCONNECT_COMPLETE event
             */
             handleSignalLmDisconnectComplete(
                    &((LM_EV_DISCONNECT_COMPLETE_T *)p_event_data)->data);
        }
        break;

        case LS_RADIO_EVENT_IND:
        {
            /* This event is raised if the application has requested
             * notification of specific radio events for a GATT connection
             */
            handleSignalLsRadioEventInd((LS_RADIO_EVENT_IND_T *)p_event_data);
        }
        break;
        
        case GATT_CHAR_VAL_NOT_CFM:
        {
            handleSignalGattNotificationCfm(
                                      (GATT_CHAR_VAL_IND_CFM_T *)p_event_data);
        }
        break;

        default:
            /* Ignore any other event */
        break;

    }
    return TRUE;
}
