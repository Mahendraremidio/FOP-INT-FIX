/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *     hw_access.h
 *
 *  DESCRIPTION
 *      Header definitions for hardware setup.
 *
 ******************************************************************************/
#ifndef __HW_ACCESS_H__
#define __HW_ACCESS_H__

/*============================================================================*
 *  SDK Header includes
 *============================================================================*/

#include <sys_events.h>

/*============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Convert a PIO number into a bit mask */
#define PIO_BIT_MASK(pio)       (0x01UL << (pio))

/* PIO 1 is configured by default for UART Rx. If we have not connected the
 * UART device,the UART Rx PIO drives low resulting in higher power consumption.
 * To save power, we explicitly drive this PIO high by doing a strong pull up, 
 * before we initialize/enable the UART.
 */
#define UART_RX_PIO             (1)
#define UART_RX_PIO_MASK        (PIO_BIT_MASK(UART_RX_PIO)) 


/* PIO direction */
#define PIO_DIRECTION_INPUT     (FALSE)
#define PIO_DIRECTION_OUTPUT    (TRUE)

/* PIO state */
#define PIO_STATE_HIGH          (TRUE)
#define PIO_STATE_LOW           (FALSE)

/* PIO for toggling baud rate */
#define PIO_BAUD_RATE           (3)

/* Mask for PIO_BAUD_RATE */
#define PIO_BAUD_RATE_MASK      PIO_BIT_MASK(PIO_BAUD_RATE)

/* Mask defined for button PIO 11 */
#define BUTTON_PIO_MASK                     (PIO_BIT_MASK(BUTTON_PIO))


/* PIOs that connect to the LED */
#define PIO_LED0                (11)       /* LED 0 connected to PIO 4   */

/* Setup PIO 11 as Button PIO */
#define BUTTON_PIO                          (4)

#define Fixation_databit_1                  (3)
#define Fixation_databit_2                  (0)
#define Fixation_databit_3                  (1)

#define IR_LED                              (9)
#define WHITE_LED                           (10)

#define LED_CTD                             (11)

#define LED_CLK                             (5)
#define LED_CS                              (6)

#define ADP                                 (7)
#define CHG                                 (8)



#define ADP_PIO_MASK                     (PIO_BIT_MASK(ADP))
#define CHG_PIO_MASK                     (PIO_BIT_MASK(CHG))

#define GLED                               (5)
#define RLED                               (6)






/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* Initialise the application hardware */
extern void InitHardware(void);

/* Initialise the application hardware data structure */
extern void HwDataInit(void);

/* Handle the PIO changed event */
extern void HandlePIOChangedEvent(pio_changed_data *pio_data);

/* Get the current status of baud rate pio */
extern bool IsBaudRatePIOHigh(void);

/* Turn on/off the LED */
extern void TurnOnLED(bool b_on);

#endif /* __HW_ACCESS_H__ */

