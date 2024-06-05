/**
 * @file tfa.h
 *
 * @brief This file is the interface for Transceiver Feature Access (TFA)
 * functionality.
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
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
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef TFA_H
#define TFA_H

#include "config/default/driver/IEEE_802154_PHY/phy/inc/phy.h"
#if (defined ENABLE_TFA) || (defined TFA_BAT_MON) || \
	(defined TFA_BAT_MON_READ) || (defined TFA_BAT_MON_IRQ) || \
	(defined TFA_CW) || (defined TFA_CCA)

/**
 * \defgroup group_tfa           Transceiver Feature Access
 * The Atmel transceivers provide a variety of additional hardware features
 * features that are not reflected in the IEEE 802.15.4 standard.
 * These features are for eg .Reading battery voltage, Continuous wave
 * transmission etc .
 * @{
 */

/* === INCLUDES ============================================================ */

//#include "return_val.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/ieee_const.h"
#include "tal_types.h"

/* === TYPES =============================================================== */

/** Transceiver commands */
//typedef enum tfa_pib_t {
//	TFA_PIB_RX_SENS         = 0
//} tfa_pib_t;

/** Continuous Transmission modes */
/* RF212B has a 2nd CW mode, described as "Additional CW Mode" in the datasheet
 **/
//typedef enum continuous_tx_mode_tag {
//	CW_MODE = 0,
//	PRBS_MODE = 1
//#if (TAL_TYPE == AT86RF212B)
//	,
//	CW_MODE_2 = 2,
//	CW_ALL_ZEROS_MODE = 3
//#endif /* (TAL_TYPE == AT86RF212B) */
//} continuous_tx_mode_t;

/* === MACROS ============================================================== */

/**
 * Default value of TFA PIB attribute to reduce the Rx sensitivity.
 * By default, reduced sensitivity is disabled.
 */
#if (TAL_TYPE == AT86RF212)
#define TFA_PIB_RX_SENS_DEF                 (RSSI_BASE_VAL_BPSK_40_DBM)
#elif (TAL_TYPE == AT86RF212B)
#define TFA_PIB_RX_SENS_DEF                 (RSSI_BASE_VAL_BPSK_600_DBM)
#else
#define TFA_PIB_RX_SENS_DEF                 (RSSI_BASE_VAL_DBM)
#endif

/**
 * Supply voltage above upper limit.
 */
#define SUPPLY_VOLTAGE_ABOVE_UPPER_LIMIT    (0xFFFF)

/**
 * Supply voltage below lower limit.
 */
#define SUPPLY_VOLTAGE_BELOW_LOWER_LIMIT    (0)

/* === GLOBALS ============================================================= */

/* === PROTOTYPES ========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#if (defined ENABLE_TFA)

/**
 * @brief Initializes the TFA
 *
 * This function is called to initialize the TFA.
 *
 * @return PHY_SUCCESS if everything went correct;
 *         PHY_FAILURE otherwise
 *
 * @ingroup apiTfaApi
 */
PHY_Retval_t tfa_init(void);

#endif

#if (defined ENABLE_TFA)

/**
 * @brief Reset the TFA
 *
 * This function is called to reset the TFA.
 *
 * @param set_default_pib Defines whether PIB values need to be set
 *                        to its default values
 *
 * @ingroup apiTfaApi
 */
void tfa_reset(bool set_default_pib);

#endif

#if (defined ENABLE_TFA)

/**
 * @brief Gets a TFA PIB attribute
 *
 * This function is called to retrieve the transceiver information base
 * attributes.
 *
 * @param[in] tfa_pib_attribute TAL infobase attribute ID
 * @param[out] value TFA infobase attribute value
 *
 * @return PHY_UNSUPPORTED_ATTRIBUTE if the TFA infobase attribute is not found
 *         PHY_SUCCESS otherwise
 *
 * @ingroup apiTfaApi
 */
PHY_Retval_t tfa_pib_get(PHY_tfa_pib_t tfa_pib_attribute, void *value);

#endif

#if (defined ENABLE_TFA)

/**
 * @brief Sets a TFA PIB attribute
 *
 * This function is called to set the transceiver information base
 * attributes.
 *
 * @param[in] tfa_pib_attribute TFA infobase attribute ID
 * @param[in] value TFA infobase attribute value to be set
 *
 * @return PHY_UNSUPPORTED_ATTRIBUTE if the TFA info base attribute is not found
 *         PHY_BUSY if the TAL is not in TAL_IDLE state.
 *         PHY_SUCCESS if the attempt to set the PIB attribute was successful
 *
 * @ingroup apiTfaApi
 */
PHY_Retval_t tfa_pib_set(PHY_tfa_pib_t tfa_pib_attribute, void *value);

#endif

#if (defined ENABLE_TFA) || (defined TFA_CCA)
#ifdef MULTI_TRX_SUPPORT

/**
 * @brief Perform a CCA
 *
 * This function performs a CCA request.
 *
 * @param trx_id Transceiver identifier
 *
 * @return PHY_Retval_t PHY_IDLE or PHY_BUSY
 *
 * @ingroup apiTfaApi
 */
PHY_Retval_t tfa_cca_perform(trx_id_t trx_id);

#else

/**
 * @brief Perform a CCA
 *
 * This function performs a CCA request.
 *
 * @return PHY_Retval_t PHY_IDLE or PHY_BUSY
 *
 * @ingroup apiTfaApi
 */
PHY_Retval_t tfa_cca_perform(void);
PHY_Retval_t PHY_CCAPerform(void);

#endif /* #ifdef MULTI_TRX_SUPPORT */
#endif

#if (defined ENABLE_TFA) || (defined TFA_CCA)
#ifdef MULTI_TRX_SUPPORT

/**
 * @brief Perform a single ED measurement
 *
 * @param trx_id Identifier of the transceiver
 *
 * @return ed_value Result of the measurement (transceiver's register value)
 *         If the build switch TRX_REG_RAW_VALUE is defined, the transceiver's
 *         register value is returned.
 *
 * @ingroup apiTfaApi
 */
uint8_t tfa_ed_sample(trx_id_t trx_id);

#else

/**
 * @brief Perform a single ED measurement
 *
 * @return ed_value Result of the measurement (transceiver's register value)
 *         If the build switch TRX_REG_RAW_VALUE is defined, the transceiver's
 *         register value is returned.
 *
 * @ingroup apiTfaApi
 */
uint8_t tfa_ed_sample(void);
uint8_t PHY_EdSample(void);
#endif /* #ifdef MULTI_TRX_SUPPORT */
#endif

#if (defined ENABLE_TFA) || (defined TFA_BAT_MON) || \
	(defined TFA_BAT_MON_READ)

/**
 * @brief Gets the transceiver's supply voltage
 *
 * @return mv Milli Volt; 0 if below threshold, 0xFFFF if above threshold
 *
 * @ingroup apiTfaApi
 */
uint16_t tfa_get_batmon_voltage(void);

#endif

#if (defined ENABLE_TFA) || (defined TFA_BAT_MON) || (defined TFA_BAT_MON_IRQ)

/**
 * @brief Setups the battery monitor interrupt
 *
 * @param   batmon_irq_cb Callback function for the batmon interrupt
 * @param   vth Threshold value in millivolt
 *
 * @return PHY_SUCCESS if provided voltage is within range, else
 *         PHY_INVALID_PARAMETER
 *
 * @ingroup apiTfaApi
 */
PHY_Retval_t tfa_batmon_irq_init(void*(batmon_irq_cb), uint16_t vth);
#endif

#if (defined ENABLE_TFA) || (PAL_GENERIC_TYPE == MEGA_RF)

/**
 * @brief Get the temperature value from the integrated sensor
 *
 * @return temperature value in degree Celsius
 *
 * @ingroup apiTfaApi
 */
double tfa_get_temperature(void);

#endif

#if ((defined ENABLE_TFA) || (defined TFA_CW))
#ifdef MULTI_TRX_SUPPORT

/**
 * @brief Starts continuous transmission on current channel
 *
 * @param trx_if Transceiver identifier
 * @param tx_mode Transmission mode
 * @param random_content Use random content if true
 */
void tfa_continuous_tx_start(trx_id_t trx_id, PHY_ContinuousTxMode_t tx_mode);

#else

/**
 * @brief Starts continuous transmission on current channel
 *
 * @param trx_id Identifier of the transceiver
 * @param tx_mode Transmission mode
 * @param random_content Use random content if true
 */
void tfa_continuous_tx_start(continuous_tx_mode_t tx_mode, bool random_content);
void PHY_StartContinuousTransmit(PHY_ContinuousTxMode_t txMode, bool randomContent);
#endif /* #ifdef MULTI_TRX_SUPPORT */
#endif

#if ((defined ENABLE_TFA) || (defined TFA_CW))
#ifdef MULTI_TRX_SUPPORT

/**
 * @brief Stops CW transmission
 *
 * @param trx_if Transceiver identifier
 */
void tfa_continuous_tx_stop(trx_id_t trx_id);

#else

/**
 * @brief Stops CW transmission
 */
void tfa_continuous_tx_stop(void);
void PHY_StopContinuousTransmit(void);
#endif /* #ifdef MULTI_TRX_SUPPORT */
#endif

#if ((defined SUPPORT_TFA) || (defined TFA_PLL_BIST))
void tfa_pll_bist_init(void);
void tfa_pll_bist_run(trx_id_t trx_id);

#endif  /* #if ((defined SUPPORT_TFA) || (defined TFA_PLL_BIST) ||
         *defined(DOXYGEN)) */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* #ifdef ENABLE_TFA */

#endif /* TFA_H */
/* EOF */
