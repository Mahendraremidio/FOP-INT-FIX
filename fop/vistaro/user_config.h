/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *     user_config.h
 *
 *  DESCRIPTION
 *      This file contains definitions to customise the application.
 *
 ******************************************************************************/
#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

/*=============================================================================*
 *  Public Definitions
 *============================================================================*/


/* 	Macro to enable/disable LED */
#define ENABLE_LED

/* OTA code has been put under complier flag ENABLE_OTA */
#ifndef NVM_TYPE_FLASH
#define ENABLE_OTA
#endif /* NVM_TYPE_FLASH */

/* The CONNECTED_IDLE_TIMEOUT_VALUE macro specifies how long the application may
 * be idle for during the Connected state. The device will disconnect when this
 * timer expires. Vendors are free to decide whether to enable this timeout, and
 * the time out duration, as per their use case.
 * If this macro is not defined there will be no disconnection.
 */
#define CONNECTED_IDLE_TIMEOUT_VALUE     (30 * MINUTE)

/* Defines the High Baud Rate. 115K2 is the maximum supported baud rate */
#define HIGH_BAUD_RATE                   (0x01d9) /* 115K2*/

/* Defines the Low Baud Rate. */ 
#define LOW_BAUD_RATE                    (0x000a) /* 2400 */

/* Extra long button press time duration */
#define EXTRA_LONG_BUTTON_PRESS_TIMER    (5 * SECOND)

/* short button press time duration */
#define SHORT_BUTTON_PRESS_TIMER         (1 * SECOND)

#define MAX_RETRY_ADVERT_ATTEMPT         (3)  

/* Partial buffer timeout values */
#define PARTIAL_BUFFER_WAIT_TIME_HIGH    (10  * MILLISECOND)
#define PARTIAL_BUFFER_WAIT_TIME_LOW     (100 * MILLISECOND)

/* Macros for device name and its length */
#define DEVICE_NAME                      "VISTARO"

/* I2C Communication lines */
#define I2C_SCL_PIO                               (1)
#define I2C_SDA_PIO                               (0)

#endif /* __USER_CONFIG_H__ */
