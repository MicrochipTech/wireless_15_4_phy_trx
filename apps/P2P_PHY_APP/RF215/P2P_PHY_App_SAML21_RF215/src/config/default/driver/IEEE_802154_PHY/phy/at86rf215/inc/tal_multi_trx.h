/**
 * @file tal_multi_trx.h
 *
 * @brief This file contains TAL API function declaration for multi trx support
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 */

/*
 * Copyright (c) 2015-2018, Microchip Technology Inc All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef TAL_MULTI_TRX_H
#define TAL_MULTI_TRX_H

/* === INCLUDES ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include "config/default/driver/IEEE_802154_PHY/pal/inc/pal.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_generic.h"

/* Default modulation */
#define MOD_SUB1        FSK
#define MOD_2_4G        LEG_OQPSK
#ifdef MULTI_TRX_SUPPORT

/* === TYPES =============================================================== */

/* === EXTERNALS =========================================================== */
extern volatile trx_id_t trx_id;
/* === MACROS ============================================================== */

/* === PROTOTYPES ========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Resets TAL state machine and sets the default PIB values if requested
 *
 * @param trx_id Transceiver identifier
 * @param set_default_pib Defines whether PIB values need to be set
 *                        to its default values
 *
 * @return
 *      - @ref PHY_SUCCESS if the transceiver state is changed to PHY_TRX_OFF
 *      - @ref PHY_FAILURE otherwise
 * @ingroup apiTalApi
 */
PHY_Retval_t tal_reset(trx_id_t trx_id, bool set_default_pib);

// #if (MAC_SCAN_ED_REQUEST_CONFIRM == 1) || defined(DOXYGEN)

/**
 * @brief Starts ED Scan
 *
 * This function starts an ED Scan for the scan duration specified by the
 * MAC layer.
 *
 * @param scan_duration Specifies the ED scan duration in symbols
 * @param trx_id Transceiver identifier
 *
 * @return
 *      - @ref PHY_SUCCESS - ED scan duration timer started successfully
 *      - @ref PHY_BUSY - TAL is busy servicing the previous request from MAC
 *      - @ref PHY_TRX_ASLEEP - Transceiver is currently sleeping
 *      - @ref PHY_FAILURE otherwise
 * @ingroup apiTalApi
 */
PHY_Retval_t tal_ed_start(trx_id_t trx_id, uint8_t scan_duration);

/**
 * User call back function for finished ED Scan
 *
 * @param trx_id Transceiver identifier
 * @param energy_level Measured energy level during ED Scan
 * @ingroup apiTalApi
 */
void tal_ed_end_cb(trx_id_t trx_id, uint8_t energy_level);

// #endif /* (MAC_SCAN_ED_REQUEST_CONFIRM == 1) */

/**
 * @brief Gets a TAL PIB attribute
 *
 * This function is called to retrieve the transceiver information base
 * attributes.
 *
 * @param trx_id Transceiver identifier
 * @param[in] attribute TAL infobase attribute ID
 * @param[out] value TAL infobase attribute value
 *
 * @return
 *      - @ref PHY_UNSUPPORTED_ATTRIBUTE if the TAL infobase attribute is not
 * found
 *      - @ref PHY_SUCCESS otherwise
 * @ingroup apiTalApi
 */

PHY_Retval_t tal_pib_get(trx_id_t trx_id, uint8_t attribute, uint8_t *value);

/**
 * @brief Sets a TAL PIB attribute
 *
 * This function is called to set the transceiver information base
 * attributes.
 *
 * @param trx_id Transceiver identifier
 * @param attribute TAL infobase attribute ID
 * @param value TAL infobase attribute value to be set
 *
 * @return
 *      - @ref PHY_UNSUPPORTED_ATTRIBUTE if the TAL info base attribute is not
 * found
 *      - @ref PHY_BUSY if the TAL is not in TAL_IDLE state. An exception is
 *         macBeaconTxTime which can be accepted by TAL even if TAL is not
 *         in TAL_IDLE state.
 *      - @ref PHY_SUCCESS if the attempt to set the PIB attribute was
 * successful
 *      - @ref PHY_TRX_ASLEEP if trx is in SLEEP mode and access to trx is
 * required
 * @ingroup apiTalApi
 */
PHY_Retval_t tal_pib_set(trx_id_t trx_id, uint8_t attribute, PibValue_t *value);

/**
 * @brief Sets a TAL PIB attribute for all existing trx Ids for this device
 *
 * This function is called to set the transceiver information base
 * attributes for all transceivers on this device. This function simply calls
 * function tal_pib_set() for all existing trx Ids.
 *
 * @param attribute TAL infobase attribute ID
 * @param value TAL infobase attribute value to be set
 *
 * @return
 *      - @ref PHY_UNSUPPORTED_ATTRIBUTE if the TAL info base attribute is not
 *found
 *      - @ref PHY_BUSY if the TAL is not in TAL_IDLE state. An exception is
 *         macBeaconTxTime which can be accepted by TAL even if TAL is not
 *         in TAL_IDLE state.
 *      - @ref PHY_SUCCESS if the attempt to set the PIB attribute was
 *successful
 *      - @ref PHY_TRX_ASLEEP if trx is in SLEEP mode and access to trx is
 *required
 * @ingroup apiTalApi
 */
PHY_Retval_t tal_pib_set_all(uint8_t attribute, PibValue_t *value);

/**
 * @brief Switches receiver on or off
 *
 * This function switches the receiver on (PHY_STATE_RX_ON) or off (PHY_STATE_TRX_OFF).
 *
 * @param trx_id Transceiver identifier
 * @param state New state of receiver
 *
 * @return
 *      - @ref PHY_BUSY if the TAL state machine cannot switch receiver on or
 * off,
 *      - @ref PHY_STATE_TRX_OFF if receiver has been switched off, or
 *      - @ref PHY_STATE_RX_ON otherwise.
 * @ingroup apiTalApi
 */
PHY_TrxStatus_t tal_rx_enable(trx_id_t trx_id, PHY_TrxState_t state);

/**
 * User call back function for frame reception
 *
 * @param trx_id Transceiver identifier
 * @param rx_frame Pointer to received frame structure of type PHY_FrameInfo_t
 *                 or to received frame array
 * @ingroup apiTalApi
 */
void tal_rx_frame_cb(trx_id_t trx_id, PHY_FrameInfo_t *rx_frame);

#ifdef ENABLE_RTB

/**
 * @brief Callback function called by TAL on frame reception if RTB is used.
 *
 * This function pushes an event into the TAL-RTB queue, indicating a
 * frame reception.
 *
 * @param frame Pointer to received frame
 */
void rtb_rx_frame_cb(trx_id_t trx_id, PHY_FrameInfo_t *rx_frame);

#endif  /* ENABLE_RTB */

/**
 * @brief Requests to TAL to transmit frame
 *
 * This function is called by the MAC to deliver a frame to the TAL
 * to be transmitted by the transceiver.
 *
 * @param trx_id Transceiver identifier
 * @param tx_frame Pointer to the PHY_FrameInfo_t structure or
 *                 to frame array to be transmitted
 * @param csma_mode Indicates mode of csma-ca to be performed for this frame
 * @param perform_frame_retry Indicates whether to retries are to be performed
 * for
 *                            this frame
 *
 * @return
 *      - @ref PHY_SUCCESS  if the TAL has accepted the data from NHLE for frame
 *                 transmission
 *      - @ref PHY_BUSY if the TAL is busy servicing the previous MAC request
 *      - @ref PHY_TRX_ASLEEP if the device is in sleep mode
 *      - @ref PHY_INVALID_PARAMETER is a provided parameter is invalid
 * @ingroup apiTalApi
 */
PHY_Retval_t tal_tx_frame(trx_id_t trx_id, PHY_FrameInfo_t *tx_frame,
		PHY_CSMAMode_t csma_mode, bool perform_frame_retry);

/**
 * User call back function for frame transmission
 *
 * @param trx_id Transceiver identifier
 * @param status Status of frame transmission attempt
 * @param frame Pointer to frame structure of type PHY_FrameInfo_t
 * @ingroup apiTalApi
 */
void tal_tx_frame_done_cb(trx_id_t trx_id, PHY_Retval_t status,
		PHY_FrameInfo_t *frame);

/**
 * @brief Sets the transceiver to sleep
 *
 * This function sets the transceiver to sleep state.
 *
 * @param trx_id Transceiver identifier
 *
 * @return
 *      - @ref PHY_BUSY - The transceiver is busy in TX or RX
 *      - @ref PHY_SUCCESS - The transceiver is put to sleep
 *      - @ref PHY_TRX_ASLEEP - The transceiver is already asleep
 *      - @ref PHY_INVALID_PARAMETER - The specified sleep mode is not supported
 * @ingroup apiTalApi
 */
PHY_Retval_t tal_trx_sleep(trx_id_t trx_id);

/**
 * @brief Wakes up the transceiver from sleep
 *
 * This function awakes the transceiver from sleep state.
 *
 * @param trx_id Transceiver identifier
 *
 * @return
 *      - @ref PHY_TRX_AWAKE - The transceiver is already awake
 *      - @ref PHY_SUCCESS - The transceiver is woken up from sleep
 *      - @ref PHY_FAILURE - The transceiver did not wake-up from sleep
 * @ingroup apiTalApi
 */
PHY_Retval_t tal_trx_wakeup(trx_id_t trx_id);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* #ifdef MULTI_TRX_SUPPORT */

#endif /* TAL_MULTI_TRX_H */
/* EOF */
