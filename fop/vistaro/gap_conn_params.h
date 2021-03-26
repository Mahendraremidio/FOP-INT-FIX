/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *     gap_conn_params.h
 *
 * DESCRIPTION
 *     MACROs for connection and advertisement parameter values
 *
 ******************************************************************************/

#ifndef __GAP_CONN_PARAMS_H__
#define __GAP_CONN_PARAMS_H__

/*============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Advertising parameters. Time is expressed in microseconds and the firmware
 * will round this down to the nearest slot. Acceptable range is 20ms to 10.24s
 * and the minimum must be no larger than the maximum. This value needs to be
 * modified at a later stage as decided GPA for specific profile.
 *
 * For the Serial Server application, use 60ms as the fast connection
 * advertisement interval. For reduced power connections the recommended range
 * is 1s to 2.5s. Vendors will need to tune these values as per their
 * requirements.
 */
#define FC_ADVERTISING_INTERVAL_MIN          (60   * MILLISECOND)
#define FC_ADVERTISING_INTERVAL_MAX          (60   * MILLISECOND)

#define RP_ADVERTISING_INTERVAL_MIN          (1280 * MILLISECOND)
#define RP_ADVERTISING_INTERVAL_MAX          (1280 * MILLISECOND)

/* Brackets should not be used around the values of these macros. This file is
 * imported by the GATT Database Generator (gattdbgen) which does not understand
 * brackets and will raise syntax errors.
 */

#endif /* __GAP_CONN_PARAMS_H__ */

