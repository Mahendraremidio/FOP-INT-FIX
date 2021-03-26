/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 *  FILE
 *      hw_access.c
 *
 *  DESCRIPTION
 *      This file defines the application hardware specific routines.
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header includes
 *============================================================================*/

#include <pio.h>
#include <pio_ctrlr.h>
#include <timer.h>

/*============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "hw_access.h"
#include "serial_server.h"
#include "user_config.h"

/*============================================================================*
 *  Private Definitions
 *============================================================================*/





/*============================================================================*
 *  Private data type
 *============================================================================*/

/* Application Hardware data structure */
typedef struct _APP_HW_DATA_T
{
    /* Timer for button press */
    timer_id                    button_press_tid;
    timer_id                    short_button_press_tid;
} APP_HW_DATA_T;

/*============================================================================*
 *  Private data
 *============================================================================*/

/* Application hardware data instance */
static APP_HW_DATA_T            g_app_hw_data;

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

/* Handle extra long button press */
static void handleExtraLongButtonPress(timer_id tid);

/* Handle short button press */
static void handleShortButtonPress(timer_id tid);

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/
/*----------------------------------------------------------------------------*
 *  NAME
 *      handleShortButtonPress
 *
 *  DESCRIPTION
 *      This function contains handling of short button press, which
 *      triggers pairing.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleShortButtonPress(timer_id tid)
{
     if(tid == g_app_hw_data.short_button_press_tid)
    {
        /* Re-initialise button press timer */
        g_app_hw_data.short_button_press_tid = TIMER_INVALID;

    } /* Else ignore timer */
}
/*----------------------------------------------------------------------------*
 *  NAME
 *      handleExtraLongButtonPress
 *
 *  DESCRIPTION
 *      This function contains handling of extra long button press, which
 *      triggers pairing / bonding removal.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleExtraLongButtonPress(timer_id tid)
{
    if(tid == g_app_hw_data.button_press_tid)
    {
        /* Re-initialise button press timer */
        g_app_hw_data.button_press_tid = TIMER_INVALID;

        /* Handle pairing removal */
        HandlePairingRemoval();

    } /* Else ignore timer */
}



/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      InitHardware
 *
 *  DESCRIPTION
 *      This function is called to initialise the application hardware.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void InitHardware(void)
{
    /* Setup PIOs
     * PIO11 - Button
     */
    /* Set the button PIO to user mode */

        PioSetModes(BUTTON_PIO_MASK, pio_mode_user);
        PioSetModes(PIO_BAUD_RATE_MASK, pio_mode_user);
    
        /* Set the PIO direction as input */
        PioSetDir(BUTTON_PIO, PIO_DIRECTION_INPUT);
    
        /* Pull up the PIO */
        PioSetPullModes(BUTTON_PIO_MASK, pio_mode_strong_pull_up);
        PioSetPullModes(PIO_BAUD_RATE_MASK, pio_mode_weak_pull_down);

    /* Request an event when the button PIO changes state */
    PioSetEventMask(BUTTON_PIO_MASK, pio_event_mode_both);

   /* Request an event when the button PIO changes state */
    PioSetEventMask(PIO_BAUD_RATE_MASK, pio_event_mode_both);

    /* Save power by changing the I2C pull mode to pull down.*/
    PioSetI2CPullMode(pio_i2c_pull_mode_strong_pull_down);
    
    /* Set the UART Rx PIO to user mode */
    PioSetModes(UART_RX_PIO_MASK, pio_mode_user);

    /* Set the UART Rx PIO direction as input */
    PioSetDir(UART_RX_PIO, PIO_DIRECTION_INPUT);

    /* Pull up the PIO to save power*/
    PioSetPullModes(UART_RX_PIO_MASK, pio_mode_strong_pull_up);
    
    /* Disable any events arising out of this PIO */
    PioSetEventMask(UART_RX_PIO_MASK, pio_event_mode_disable);

#ifdef ENABLE_LED
   /* Set LED0 and LED1 to be controlled directly via PioSet */
    PioSetModes((1UL << PIO_LED0), pio_mode_user);

    /* Configure LED0 and LED1 to be outputs */
    PioSetDir(PIO_LED0, PIO_DIRECTION_OUTPUT);

    /* Set the LED0 and LED1 to have strong internal pull ups */
    PioSetPullModes((1UL << PIO_LED0),pio_mode_strong_pull_up);

    /* Turn off the LED by setting output to Low */
    PioSets((1UL << PIO_LED0),0UL);
#endif /* ENABLE_LED */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HwDataInit
 *
 *  DESCRIPTION
 *      This function initialises the hardware data to a known state. It is
 *      intended to be called once, for example after a power-on reset or HCI
 *      reset.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HwDataInit(void)
{
    /* Delete button press timer */
    if (g_app_hw_data.button_press_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_hw_data.button_press_tid);
        g_app_hw_data.button_press_tid = TIMER_INVALID;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandlePIOChangedEvent
 *
 *  DESCRIPTION
 *      This function handles the PIO Changed event.
 *
 *  PARAMETERS
 *      pio_data [in]           State of the PIOs when the event occurred
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HandlePIOChangedEvent(pio_changed_data *pio_data)
{
    /* PIO changed */
    uint32 pios = PioGets();

    if(pio_data->pio_cause & BUTTON_PIO_MASK)
    {
        if(!(pios & BUTTON_PIO_MASK))
        {
            /* This event is triggered when a button is pressed. */

            /* Start a timer for EXTRA_LONG_BUTTON_PRESS_TIMER seconds. If the
             * timer expires before the button is released an extra long button
             * press is detected. If the button is released before the timer
             * expires a short button press is detected.
             */
            TimerDelete(g_app_hw_data.button_press_tid);
            TimerDelete(g_app_hw_data.short_button_press_tid);
            HandleExtTrigger();
            
           
            g_app_hw_data.short_button_press_tid =
            TimerCreate(SHORT_BUTTON_PRESS_TIMER,
                                           TRUE, handleShortButtonPress);

            g_app_hw_data.button_press_tid =
                TimerCreate(EXTRA_LONG_BUTTON_PRESS_TIMER,
                                           TRUE, handleExtraLongButtonPress);
        }
        else
        {
            /* This event comes when a button is released. */
            if(g_app_hw_data.short_button_press_tid != TIMER_INVALID)
            {
               /* Timer was already running. This means it was a
                * short button press.
                */
                TimerDelete(g_app_hw_data.short_button_press_tid);
                g_app_hw_data.short_button_press_tid = TIMER_INVALID;

                TimerDelete(g_app_hw_data.button_press_tid);
                g_app_hw_data.button_press_tid = TIMER_INVALID;

                HandleShortButtonPress();
                
            }

            /* This event comes when a button is released. */
            if(g_app_hw_data.button_press_tid != TIMER_INVALID)
            {
                /* Timer was already running. This means it was a long button
                 * press.
                 */
                TimerDelete(g_app_hw_data.button_press_tid);
                g_app_hw_data.button_press_tid = TIMER_INVALID;

                HandleLongButtonPress();
            }
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IsBaudRatePIOHigh
 *
 *  DESCRIPTION
 *      This function determines the baud rate pio status dynamically.
 *
 *  PARAMETERS
 *
 *
 *  RETURNS
 *      TRUE/FALSE
 *----------------------------------------------------------------------------*/
extern bool IsBaudRatePIOHigh(void)
{
      /* PIO changed */
      uint32 pios = PioGets();

      /* PIO 9 driving high */
      if(pios & PIO_BAUD_RATE_MASK)
      {
          return TRUE;
      }
      else
      {
          return FALSE;
      }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      TurnOnLED
 *
 *  DESCRIPTION
 *      This function either turns On/Off the LED.
 *
 *  PARAMETERS
 *
 *
 *  RETURNS
 *      TRUE/FALSE
 *----------------------------------------------------------------------------*/
extern void TurnOnLED(bool b_on)
{
#ifdef ENABLE_LED
    if(b_on)
    {
         /* Turn on LED */
         PioSet(PIO_LED0, TRUE);
    }
    else
    {
         /* Turn off the LED */
         PioSet(PIO_LED0, FALSE);
    }
#endif /* ENABLE_LED */
}