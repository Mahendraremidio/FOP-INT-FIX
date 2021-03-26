/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *     uart_interface.c
 *
 * DESCRIPTION
 *     This file defines routines for using the UART.
 *
 ******************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include <uart.h>
#include <debug.h>
#include <timer.h>
/*============================================================================*
 *  Local Header Files
 *============================================================================*/
#include "uart_interface.h"
#include "serial_gatt.h"
#include "byte_queue.h"
#include "serial_service.h"
#include "serial_server.h"
#include "user_config.h"
/*============================================================================*
 *  Private Definitions
 *============================================================================*/
 /* The application is required to create two buffers, one for receive, the
  * other for transmit. The buffers need to meet the alignment requirements
  * of the hardware. See the macro definition in uart.h for more details.
  */

/* Create 256-byte receive buffer for UART data */
//UART_DECLARE_BUFFER(rx_buffer, UART_BUF_SIZE_BYTES_256);

/* Create 256-byte transmit buffer for UART data */
//UART_DECLARE_BUFFER(tx_buffer, UART_BUF_SIZE_BYTES_256);

/* Next received number of bytes when it/they become available on UART */
#define NUMBER_OF_BYTES_RECEIVED_NEXT                (1)



/*============================================================================*
 *  Private data
 *============================================================================*/
/* variable to trigger uart callback when it is ready to accept data */
bool g_trigger_write_callback = FALSE;

/* variable to track flow control */
bool g_last_notification_success = TRUE;

/* determines whether the baudrate is high or low */
bool g_is_current_baud_rate_high = FALSE;

/* partial buffer send timer */
timer_id    g_partial_buffer_timer_tid;

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

/* UART receive callback to receive serial commands */
/*static uint16 uartRxDataCallback(void   *p_rx_buffer,
                                 uint16  length,
                                 uint16 *p_req_data_length);*/

/* UART transmit callback when a UART transmission has finished */
//static void uartTxDataCallback(void);

/* Send any pending data which we were not able to write previously */
//static void sendPendingData(void);

/* Function that sends the partial buffer less than packet length */
static void SendPartialBuffer(timer_id tid);

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      uartRxDataCallback
 *
 *  DESCRIPTION
 *      This is an internal callback function (of type uart_data_in_fn) that
 *      will be called by the UART driver when any data is received over UART.
 *      See DebugInit in the Firmware Library documentation for details.
 *
 * PARAMETERS
 *      p_rx_buffer [in]   Pointer to the receive buffer (uint8 if 'unpacked'
 *                         or uint16 if 'packed' depending on the chosen UART
 *                         data mode - this application uses 'unpacked')
 *
 *      length [in]        Number of bytes ('unpacked') or words ('packed')
 *                         received
 *
 *      p_additional_req_data_length [out]
 *                         Number of additional bytes ('unpacked') or words
 *                         ('packed') this application wishes to receive
 *
 * RETURNS
 *      The number of bytes ('unpacked') or words ('packed') that have been
 *      processed out of the available data.
 *----------------------------------------------------------------------------*/

/*static uint16 uartRxDataCallback(void   *p_rx_buffer,
                                 uint16  length,
                                 uint16 *p_additional_req_data_length)
{
    if ( length > 0 )
    {
        if(AppGetState() == app_state_connected)
        {
            
            BQForceQueueBytes((const uint8 *)p_rx_buffer, length,SEND_QUEUE_ID);
        }
    }

  
    *p_additional_req_data_length = (uint16)NUMBER_OF_BYTES_RECEIVED_NEXT;
    
    
    if(!IsAppWaitingForRadioEvent())
    {
        ProcessRxData();
    }

    
    return length;
}*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      uartTxDataCallback
 *
 *  DESCRIPTION
 *      This is an internal callback function (of type uart_data_out_fn) that
 *      will be called by the UART driver when data transmission over the UART
 *      is finished. See DebugInit in the Firmware Library documentation for
 *      details.
 *
 * PARAMETERS
 *      None
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/


/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      InitUart
 *
 *  DESCRIPTION
 *      This function is called to initialise the UART.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/

extern void InitUart(void)
{
    /* Initialise UART and configure with default baud rate and port
     * configuration
     */
    /*UartInit(uartRxDataCallback,
             uartTxDataCallback,
             rx_buffer, UART_BUF_SIZE_BYTES_256,
             tx_buffer, UART_BUF_SIZE_BYTES_256,
             uart_data_unpacked);*/
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      ProcessRxData
 *
 *  DESCRIPTION
 *      Read and process the next command from the byte queue.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/

extern void ProcessRxData(void)
{
    /* Data to be sent */
    uint8  data[SERIAL_RX_DATA_LENGTH]; 
    
    /* Length of the data to be sent */    
    uint16 size_val; 
    
    /* Length of data available in the queue */
    uint16 length = BQGetDataSize(SEND_QUEUE_ID); 

    /* Proceed only if byte queue is not empty */
    if(length > 0)
    {
        /* Make sure that the maximum data length is not exceeded */
        size_val = length > SERIAL_RX_DATA_LENGTH ?
                   SERIAL_RX_DATA_LENGTH : length;

        if( length<SERIAL_RX_DATA_LENGTH ) 
        {
            /* Length of the data is less than serial data length. */
            if(g_is_current_baud_rate_high)
            {
                if (g_partial_buffer_timer_tid == TIMER_INVALID)
                {
                  /* Create the partial buffer timer */
                   g_partial_buffer_timer_tid = TimerCreate(
                                                 PARTIAL_BUFFER_WAIT_TIME_HIGH,
                                                 TRUE,
                                                 SendPartialBuffer);
                }
            }
            else
            {
               if (g_partial_buffer_timer_tid == TIMER_INVALID)
               {
                /* Create the partial buffer timer */
                g_partial_buffer_timer_tid = TimerCreate(
                                                 PARTIAL_BUFFER_WAIT_TIME_LOW,
                                                 TRUE,
                                                 SendPartialBuffer);
               }
            }
        }
        else
        {
            if (g_partial_buffer_timer_tid != TIMER_INVALID)
            {
                /* Kill the partial buffer timer. */
                TimerDelete(g_partial_buffer_timer_tid);
                g_partial_buffer_timer_tid = TIMER_INVALID;
            }
           
            
            /* Peek data and send it. */
            if (BQPeekBytes(data, size_val,SEND_QUEUE_ID) > 0)
            {
                SerialSendNotification(data, size_val);
            }
        
            /* Pop the data if the last sent status was success */
            if(g_last_notification_success)
            {
                BQPopBytes(data, size_val,SEND_QUEUE_ID);
            }
        }

    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      SendDataToUart
 *
 *  DESCRIPTION
 *      Sends the required data to UART.
 *
 *  PARAMETERS
 *      *data[in]              pointer to the data to be sent to UART
 *      size                   size of the data to be sent
 *z
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/

extern void SendDataToUart(uint8 *data, uint16 size)
{
    
    /* We initially attempt to directly write to the UART and from there on
     * we send the data using callback mechanism, whenever the UART is ready
     * to accept more incoming data. This is to avoid data loss. The data is
     * buffered and sent to UART using callback mechanism.
     */
    if(!g_trigger_write_callback)
    {
        /*UartWrite(data, size);*/
        
        g_trigger_write_callback = TRUE;
        return;
    }

    /* Queue the incoming data to the queue. will be written when UART is ready
     */  
    if(g_trigger_write_callback)
    {
        /* First copy all the bytes received into the byte queue */
        /*BQSafeQueueBytes((const uint8 *)data, size,RECV_QUEUE_ID);*/

        /*Send Pending Data */
        /*sendPendingData();*/
        if(size==1)
        {
            /*DebugWriteString("Size1");
            DebugWriteString("ar[0] = ");
            DebugWriteUint8((uint8)*data++);
            DebugWriteString("\n\r");*/
        }
        
        if(size==2)
        {
            /*DebugWriteString("ar[0] = ");
            DebugWriteUint8((uint8)*data++);
            DebugWriteString("\n\r");
            DebugWriteString("ar[1] = ");
            DebugWriteUint8((uint8)*data++);
            DebugWriteString("\n\r");*/
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      ConfigureUart
 *
 *  DESCRIPTION
 *      Configures the UART.
 *
 *  PARAMETERS
 *      bool              TRUE/FALSE
 *
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void ConfigureUart(bool bHigh)
{
    if(bHigh)
    {
        /* Configure the UART for high baud rate */
        //UartConfig(HIGH_BAUD_RATE,0);
        
        /* Enable the UART back */
        //UartEnable(TRUE);
        
        /* Read from UART */
       // UartRead(1,0);
        
        /* Disable deep sleep, so characters dont go missing  */
        SleepModeChange(sleep_mode_never); 
                
        /* Setup the variable to indicate the current baud rate is high */
          g_is_current_baud_rate_high = TRUE;
    }
    else
    {
        /* Configure the UART for high baud rate */
       // UartConfig(LOW_BAUD_RATE,0);
        
        /* Enable the UART back */
       // UartEnable(TRUE);
        
        /* Read from UART */
       // UartRead(1,0);
        
        /* Enable deep sleep */
        SleepModeChange(sleep_mode_deep);  
        
        /* Setup the variable to indicate the current baud rate is low */
        g_is_current_baud_rate_high = FALSE;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      SetLastNotificationStatus
 *
 *  DESCRIPTION
 *      Updates the last sent notification status, whether success or failed
 *
 *  PARAMETERS
 *
 *      bool    - If TRUE  Notification was sent.
 *                If FALSE Unable to send notification.
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void SetLastNotificationStatus(bool bsuccess)
{
    g_last_notification_success = bsuccess;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      sendPendingData
 *
 *  DESCRIPTION
 *      Send buffered data over UART that was waiting to be sent. Perform some
 *      translation to ensured characters are properly displayed.
 *
 * PARAMETERS
 *      None
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
/*static void sendPendingData(void)
{
        
        uint8  data[SERIAL_RX_DATA_LENGTH]; 
        uint16 size_val; 

        
        uint16 length = BQGetDataSize(RECV_QUEUE_ID);

        
        while (BQGetDataSize(RECV_QUEUE_ID) > 0)
        {
            
            size_val = length > SERIAL_RX_DATA_LENGTH ?
                       SERIAL_RX_DATA_LENGTH : length;

            if (BQPeekBytes(data, size_val,RECV_QUEUE_ID) > 0)
            {
                
                bool ok_to_commit = UartWrite( data,size_val );

                if(!ok_to_commit)
                {
                   
                    break;
                }
                else 
                    BQPopBytes(data, size_val,RECV_QUEUE_ID);
            }
        }
}
*/
/*----------------------------------------------------------------------------*
 *  NAME
 *      SendPartialBuffer
 *
 *  DESCRIPTION
 *      Partial send timer expired.Send whatever data in buffer now.
 *
 *  PARAMETERS
 *      timer_id  id of the partial buffer timer
 *
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void SendPartialBuffer(timer_id tid)
{
     /* The relevant parital timeout period expired. Send whatever data present
      * in the queue now.
      */
    uint8  data[SERIAL_RX_DATA_LENGTH]; /* Data to be sent */

    if((tid == g_partial_buffer_timer_tid)) 
    {
        uint16 length;

        g_partial_buffer_timer_tid = TIMER_INVALID;         
        
        length = BQGetDataSize(SEND_QUEUE_ID);

        if(length > 0)
        {
            if(length < SERIAL_RX_DATA_LENGTH)
            {
                /* Peek data and send it. */
                if (BQPeekBytes(data,length,SEND_QUEUE_ID) > 0)
                {
                    SerialSendNotification(data, length);
                }
        
                /* Pop the data if the last sent status was success */
                if(g_last_notification_success)
                {
                    BQPopBytes(data, length,SEND_QUEUE_ID);
                }
            }
        }
    }
}
