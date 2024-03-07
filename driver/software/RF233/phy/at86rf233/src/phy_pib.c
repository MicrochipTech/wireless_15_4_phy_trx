/*******************************************************************************
* Copyright (C) 2024 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/



/* === INCLUDES ============================================================ */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "../../../phy/inc/phy.h"
#include "../../../phy/inc/ieee_phy_const.h"
#include "../../../phy/inc/phy_constants.h"
#include "../../at86rf/inc/phy_pib.h"
#include "../../at86rf/inc/at86rf.h"
#include "../../../phy/at86rf/inc/phy_internal.h"

/**
 * \addtogroup group_tal_pib_233
 * @{
 */

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */



/*
 * Default translation table converting register values to power levels (dBm).
 */
const int8_t tx_pwr_table[16] = {
	4, /* 4 */
	3, /* 3.7 */
	3, /* 3.4 */
	3, /* 3 */
	3, /* 2.5 */
	2, /* 2 */
	1, /* 1 */
	0, /* 0 */
	-1, /* -1 */
	-2, /* -2 */
	-3, /* -3 */
	-4, /* -4 */
	-6, /* -6 */
	-8, /* -8 */
	-12, /* -12 */
	-17, /* -17 */
};



/* === GLOBALS ============================================================= */

/* === PROTOTYPES ========================================================== */


static uint8_t limit_tx_pwr(uint8_t curr_transmit_power);

uint8_t convert_phyTransmitPower_to_reg_value(uint8_t phyTransmitPower_value);

#ifdef HIGH_DATA_RATE_SUPPORT
static bool apply_channel_page_configuration(uint8_t ch_page);
#endif



/* ! @} */
/* === IMPLEMENTATION ====================================================== */

/*
 * \brief Initialize the TAL PIB
 *
 * This function initializes the TAL information base attributes
 * to their default values.
 */
void init_tal_pib(void)
{
	tal_pib.MaxCSMABackoffs = PHY_MAX_CSMA_BACKOFFS_DEFAULT;
	tal_pib.MinBE = PHY_MINBE_DEFAULT;
	tal_pib.PANId = PHY_PANID_BC_DEFAULT;
	tal_pib.ShortAddress = PHY_SHORT_ADDRESS_DEFAULT;
	tal_pib.CurrentChannel = PHY_CURRENT_CHANNEL_DEFAULT;
	tal_pib.SupportedChannels = TRX_SUPPORTED_CHANNELS;
	tal_pib.CurrentPage = PHY_CURRENT_PAGE_DEFAULT;
	tal_pib.MaxFrameDuration = PHY_MAX_FRAME_DURATION_DEFAULT;
	tal_pib.SHRDuration = PHY_SHR_DURATION_DEFAULT;
	tal_pib.SymbolsPerOctet = PHY_SYMBOLS_PER_OCTET_DEFAULT;
	tal_pib.MaxBE = PHY_MAXBE_DEFAULT;
	tal_pib.MaxFrameRetries = PHY_MAXFRAMERETRIES_DEFAULT;
	tal_pib.TransmitPower = limit_tx_pwr(PHY_TRANSMIT_POWER_DEFAULT);
	tal_pib.CCAMode = (uint8_t)PHY_CCA_MODE_DEFAULT;
	tal_pib.PrivatePanCoordinator = PHY_PAN_COORDINATOR_DEFAULT;

#ifdef PROMISCUOUS_MODE
	tal_pib.PromiscuousMode = PHY_PIB_PROMISCUOUS_MODE_DEFAULT;
#endif
}

/*
 * \brief Write all shadow PIB variables to the transceiver
 *
 * This function writes all shadow PIB variables to the transceiver.
 * It is assumed that the radio does not sleep.
 */
void write_all_tal_pib_to_trx(void)
{
	uint8_t *ptr_to_reg;

	ptr_to_reg = (uint8_t *)&tal_pib.PANId;
	for (uint8_t i = 0; i < 2; i++) {
		trx_reg_write((RG_PAN_ID_0 + i), *ptr_to_reg);
		ptr_to_reg++;
	}

	ptr_to_reg = (uint8_t *)&tal_pib.IeeeAddress;
	for (uint8_t i = 0; i < 8; i++) {
		trx_reg_write((RG_IEEE_ADDR_0 + i), *ptr_to_reg);
		ptr_to_reg++;
	}

	ptr_to_reg = (uint8_t *)&tal_pib.ShortAddress;
	for (uint8_t i = 0; i < 2; i++) {
		trx_reg_write((RG_SHORT_ADDR_0 + i), *ptr_to_reg);
		ptr_to_reg++;
	}

	/* configure TX_ARET; CSMA and CCA */
	trx_reg_bit_write(SR_CCA_MODE, tal_pib.CCAMode);
	trx_reg_bit_write(SR_MIN_BE, tal_pib.MinBE);

	trx_reg_bit_write(SR_AACK_I_AM_COORD, tal_pib.PrivatePanCoordinator);

	/* set phy parameter */
	trx_reg_bit_write(SR_MAX_BE, tal_pib.MaxBE);

#ifdef HIGH_DATA_RATE_SUPPORT
	apply_channel_page_configuration(tal_pib.CurrentPage);
#endif

	trx_reg_bit_write(SR_CHANNEL, tal_pib.CurrentChannel);
    


	{
		uint8_t reg_value;

		reg_value = convert_phyTransmitPower_to_reg_value(
				tal_pib.TransmitPower);
		trx_reg_bit_write(SR_TX_PWR, reg_value);
	}

#ifdef PROMISCUOUS_MODE
	if (tal_pib.PromiscuousMode) {
		(void)set_trx_state(CMD_RX_ON);
	}
#endif
}

/*
 * \brief Gets a TAL PIB attribute
 *
 * This function is called to retrieve the transceiver information base
 * attributes.
 *
 * \param[in] attribute TAL infobase attribute ID
 * \param[out] value TAL infobase attribute value
 *
 * \return MAC_UNSUPPORTED_ATTRIBUTE if the TAL infobase attribute is not found
 *         MAC_SUCCESS otherwise
 */

PHY_Retval_t PHY_PibGet(uint8_t attribute, uint8_t *value)
{
	switch (attribute) {
	case macMaxCSMABackoffs:
		*value = tal_pib.MaxCSMABackoffs;
		break;

	case macMinBE:
		*value = tal_pib.MinBE;
		break;

	case macPANId:
		*(uint16_t *)value = tal_pib.PANId;
		break;

#ifdef PROMISCUOUS_MODE
	case macPromiscuousMode:
		*(uint16_t *)value = tal_pib.PromiscuousMode;
		break;
#endif
	case macShortAddress:
		*(uint16_t *)value = tal_pib.ShortAddress;
		break;

	case phyCurrentChannel:
		*value = tal_pib.CurrentChannel;
		break;

	case phyChannelsSupported:
		*(uint32_t *)value = tal_pib.SupportedChannels;
		break;

	case phyTransmitPower:
		*value = tal_pib.TransmitPower;
		break;

	case phyCCAMode:
		*value = tal_pib.CCAMode;
		break;

	case phyCurrentPage:
		*value = tal_pib.CurrentPage;
		break;

	case phyMaxFrameDuration:
		*(uint16_t *)value = tal_pib.MaxFrameDuration;
		break;

	case phySymbolsPerOctet:
		*value = tal_pib.SymbolsPerOctet;
		break;

	case phySHRDuration:
		*value = tal_pib.SHRDuration;
		break;

	case macMaxBE:
		*value = tal_pib.MaxBE;
		break;

	case macMaxFrameRetries:
		*value = tal_pib.MaxFrameRetries;
		break;

	case macIeeeAddress:
		(void)memcpy((uint8_t *)value, (uint8_t *)&tal_pib.IeeeAddress,
				sizeof(tal_pib.IeeeAddress));
		break;


	case mac_i_pan_coordinator:
		*(bool *)value = tal_pib.PrivatePanCoordinator;
		break;

	case macAckWaitDuration:

		/*
		 * AT86RF233 does not support changing this value w.r.t.
		 * compliance operation.
		 */
		return PHY_UNSUPPORTED_ATTRIBUTE;

	default:
		/* Invalid attribute id */
		return PHY_UNSUPPORTED_ATTRIBUTE;
		break;
	}

	return PHY_SUCCESS;
} /* tal_pib_get() */



/*
 * \brief Sets a TAL PIB attribute
 *
 * This function is called to set the transceiver information base
 * attributes.
 *
 * \param attribute TAL infobase attribute ID
 * \param value TAL infobase attribute value to be set
 *
 * \return MAC_UNSUPPORTED_ATTRIBUTE if the TAL info base attribute is not found
 *         TAL_BUSY if the TAL is not in TAL_IDLE state. An exception is
 *         macBeaconTxTime which can be accepted by TAL even if TAL is not
 *         in TAL_IDLE state.
 *         PHY_SUCCESS if the attempt to set the PIB attribute was successful
 *         TAL_TRX_ASLEEP if trx is in SLEEP mode and access to trx is required
 */
PHY_Retval_t PHY_PibSet(uint8_t attribute, PibValue_t *value)
{
	/*
	 * Do not allow any changes while ED or TX is done.
	 * We allow changes during RX, but it's on the user's own risk.
	 */
	if (phy_info.tal_state == PHY_ED_RUNNING) {
		return PHY_BUSY;
	}

	/*
	 * Distinguish between PIBs that need to be changed in trx directly
	 * and those that are simple variable udpates.
	 * Ensure that the transceiver is not in SLEEP.
	 * If it is in SLEEP, change it to TRX_OFF.
	 */

	switch (attribute) {
	case macMaxFrameRetries:

		/*
		 * The new PIB value is not immediately written to the
		 * transceiver. This is done on a frame-by-frame base.
		 */
		tal_pib.MaxFrameRetries = value->pib_value_8bit;
		break;

	case macMaxCSMABackoffs:

		/*
		 * The new PIB value is not immediately written to the
		 * transceiver. This is done on a frame-by-frame base.
		 */
		tal_pib.MaxCSMABackoffs = value->pib_value_8bit;
		break;

#ifdef PROMISCUOUS_MODE
	case macPromiscuousMode:
		tal_pib.PromiscuousMode = value->pib_value_8bit;
		if (tal_pib.PromiscuousMode) {
			PHY_TrxWakeup();

			/* Check if receive buffer is available or queue is not
			 * full. */
			if (NULL == tal_rx_buffer) {
				(void)set_trx_state(CMD_PLL_ON);
				phy_info.tal_rx_on_required = true;
			} else {
				(void)set_trx_state(CMD_RX_ON);
			}
		} else {
			(void)set_trx_state(CMD_TRX_OFF);
			phy_info.tal_rx_on_required = false;
		}
		break;
#endif

	default:

		/*
		 * Following PIBs require access to trx.
		 * Therefore trx must be at least in TRX_OFF.
		 */

		if (phy_info.tal_trx_status == TRX_SLEEP) {
			/* While trx is in SLEEP, register cannot be accessed.
			**/
			return PHY_TRX_ASLEEP;
		}

		switch (attribute) {
		case macMinBE:
			tal_pib.MinBE = value->pib_value_8bit;

#ifndef REDUCED_PARAM_CHECK

			/*
			 * macMinBE must not be larger than macMaxBE or
			 * calculation
			 * of macMaxFrameWaitTotalTime will fail.
			 */
			if (tal_pib.MinBE > tal_pib.MaxBE) {
				tal_pib.MinBE = tal_pib.MaxBE;
			}
#endif  /* REDUCED_PARAM_CHECK */

			trx_reg_bit_write(SR_MIN_BE, tal_pib.MinBE);
			break;

		case macPANId:
			tal_pib.PANId = value->pib_value_16bit;
			{
				uint8_t *ptr_pan;
				ptr_pan = (uint8_t *)&tal_pib.PANId;
				for (uint8_t iter = 0; iter < 2U; iter++) {
					trx_reg_write((RG_PAN_ID_0 + iter),
							*ptr_pan);
					ptr_pan++;
				}
			}
			break;

		case macShortAddress:
			tal_pib.ShortAddress = value->pib_value_16bit;
			{
				uint8_t *ptr_shrt;
				ptr_shrt = (uint8_t *)&tal_pib.ShortAddress;
				for (uint8_t iter = 0; iter < 2U; iter++) {
					trx_reg_write((RG_SHORT_ADDR_0 + iter),
							*ptr_shrt);
					ptr_shrt++;
				}
			}
			break;

		case phyCurrentChannel:
			if (phy_info.tal_state != PHY_IDLE) {
				return PHY_BUSY;
			}

			if ((bool)((uint32_t)TRX_SUPPORTED_CHANNELS &
					((uint32_t)0x01 <<
					value->pib_value_8bit))) {
				tal_trx_status_t previous_trx_status = TRX_OFF;

				/*
				 * Set trx to "soft" off avoiding that ongoing
				 * transaction (e.g. ACK) are interrupted.
				 */
				if (phy_info.tal_trx_status != TRX_OFF) {
					previous_trx_status = RX_AACK_ON; /* any
					                                   *
					                                   *
					                                   *
					                                   *other
					                                   *
					                                   *
					                                   *
					                                   *than
					                                   *
					                                   *
					                                   *
					                                   *TRX_OFF
					                                   *
					                                   *
					                                   *
					                                   *state
					                                   **/
					do {
						/* set TRX_OFF until it could be
						 * set;
						 * trx might be busy */
					} while (set_trx_state(CMD_TRX_OFF) !=
							TRX_OFF);
				}

				tal_pib.CurrentChannel = value->pib_value_8bit;
				trx_reg_bit_write(SR_CHANNEL,
						tal_pib.CurrentChannel);
                


                
				/* Re-store previous trx state */
				if (previous_trx_status != TRX_OFF) {
					/* Set to default state */
					(void)set_trx_state(CMD_RX_AACK_ON);

				}
			} else {
				return PHY_INVALID_PARAMETER;
			}

			break;

		case phyCurrentPage:
#ifdef HIGH_DATA_RATE_SUPPORT
			if (phy_info.tal_state != PHY_IDLE) {
				return PHY_BUSY;
			} else {
				uint8_t page;
				tal_trx_status_t previous_trx_status = TRX_OFF;
				bool ret_val;

				/*
				 * Changing the channel, channel page or
				 * modulation
				 * requires that TRX is in TRX_OFF.
				 * Store current trx state and return to default
				 * state
				 * after channel page has been set.
				 */
				if (phy_info.tal_trx_status != TRX_OFF) {
					previous_trx_status = RX_AACK_ON; /* any
					                                   *
					                                   *
					                                   *
					                                   *other
					                                   *
					                                   *
					                                   *
					                                   *than
					                                   *
					                                   *
					                                   *
					                                   *TRX_OFF
					                                   *
					                                   *
					                                   *
					                                   *state
					                                   **/
					do {
						/* set TRX_OFF until it could be
						 * set;
						 * trx might be busy */
					} while (set_trx_state(CMD_TRX_OFF) !=
							TRX_OFF);
				}

				page = value->pib_value_8bit;

				ret_val
					= apply_channel_page_configuration(page);

				if (previous_trx_status != TRX_OFF) {
					/* Set to default state */
					(void)set_trx_state(CMD_RX_AACK_ON);
                    

				}

				if (ret_val) {
					tal_pib.CurrentPage = page;
				} else {
					return PHY_INVALID_PARAMETER;
				}
			}

#else
			if (phy_info.tal_state != PHY_IDLE) {
				return PHY_BUSY;
			} else {
				uint8_t page;

				page = value->pib_value_8bit;
				if (page != 0U) {
					return PHY_INVALID_PARAMETER;
				}
			}
#endif  /* #ifdef HIGH_DATA_RATE_SUPPORT */
			break;

		case macMaxBE:
			tal_pib.MaxBE = value->pib_value_8bit;
#ifndef REDUCED_PARAM_CHECK

			/*
			 * macMinBE must not be larger than macMaxBE or
			 * calculation
			 * of macMaxFrameWaitTotalTime will fail.
			 */
			if (tal_pib.MaxBE < tal_pib.MinBE) {
				tal_pib.MinBE = tal_pib.MaxBE;
			}
#endif  /* REDUCED_PARAM_CHECK */
			trx_reg_bit_write(SR_MAX_BE, tal_pib.MaxBE);
			break;

		case phyTransmitPower:
			tal_pib.TransmitPower = value->pib_value_8bit;
			{
				/* Limit tal_pib.TransmitPower to max/min trx
				 * values */
				tal_pib.TransmitPower = limit_tx_pwr(
						tal_pib.TransmitPower);
				uint8_t reg_value
					= convert_phyTransmitPower_to_reg_value(
						tal_pib.TransmitPower);
				trx_reg_bit_write(SR_TX_PWR, reg_value);


                
			}
			break;

		case phyCCAMode:
			tal_pib.CCAMode = value->pib_value_8bit;
			trx_reg_bit_write(SR_CCA_MODE, tal_pib.CCAMode);
			break;

		case macIeeeAddress:
			tal_pib.IeeeAddress = value->pib_value_64bit;
			{
				uint8_t *ptr;
				ptr = (uint8_t *)&tal_pib.IeeeAddress;
				for (uint8_t iter = 0; iter < 8U; iter++) {
					trx_reg_write((RG_IEEE_ADDR_0 + iter),
							*ptr);
					ptr++;
				}
			}
			break;

		case mac_i_pan_coordinator:
			tal_pib.PrivatePanCoordinator = value->pib_value_bool;
			trx_reg_bit_write(SR_AACK_I_AM_COORD,
					tal_pib.PrivatePanCoordinator);
			break;

		case macAckWaitDuration:

			/*
			 * AT86RF233 does not support changing this value w.r.t.
			 * compliance operation.
			 * The ACK timing can be reduced to 2 symbols using TFA
			 * function.
			 */
			return PHY_UNSUPPORTED_ATTRIBUTE;
			break;

		default:
			return PHY_UNSUPPORTED_ATTRIBUTE;
			break;
		}

		break; /* end of 'default' from 'switch (attribute)' */
	}
	return PHY_SUCCESS;
} /* tal_pib_set() */

/**
 * \brief Limit the phyTransmitPower to the trx limits
 *
 * \param phyTransmitPower phyTransmitPower value
 *
 * \return limited tal_pib_TransmitPower
 */

static uint8_t limit_tx_pwr(uint8_t curr_transmit_power)
{
	uint8_t ret_val = curr_transmit_power;
	int8_t dbm_value;

	dbm_value = (int8_t)CONV_phyTransmitPower_TO_DBM(curr_transmit_power);
	if (dbm_value > (int8_t)*(&tx_pwr_table[0])) {
		dbm_value = (int8_t)*(&tx_pwr_table[0]);
		ret_val = (uint8_t)CONV_DBM_TO_phyTransmitPower(dbm_value);
	} else if (dbm_value <
			(int8_t)*(&tx_pwr_table[sizeof(tx_pwr_table)
			- 1U])) {
		dbm_value
			= (int8_t)*(&tx_pwr_table[sizeof(
					tx_pwr_table)
				- 1U]);
		ret_val = (uint8_t)CONV_DBM_TO_phyTransmitPower(dbm_value);
	}else{
		/*DO NOTHING*/
	}

	return (ret_val | TX_PWR_TOLERANCE);
}

/**
 * \brief Converts a phyTransmitPower value to a register value
 *
 * \param phyTransmitPower_value phyTransmitPower value
 *
 * \return register value
 */
uint8_t convert_phyTransmitPower_to_reg_value(uint8_t phyTransmitPower_value)
{
	int8_t dbm_value;
	uint8_t index;
	int8_t trx_tx_level;

	dbm_value = CONV_phyTransmitPower_TO_DBM(phyTransmitPower_value);

	/* Compare to the register value to identify the value that matches. */
	for (index = 0; index < sizeof(tx_pwr_table); index++) {
		trx_tx_level = (int8_t)*(&tx_pwr_table[index]);
		if (trx_tx_level <= dbm_value) {
			if (trx_tx_level < dbm_value) {
				return (index - 1U); 
			}

			return index;
		}
	}

	/* This code should never be reached. */
	return 0;
}


#ifdef HIGH_DATA_RATE_SUPPORT

/**
 * \brief Apply channel page configuartion to transceiver
 *
 * \param ch_page Channel page
 *
 * \return true if changes could be applied else false
 */
static bool apply_channel_page_configuration(uint8_t ch_page)
{
	/*
	 * Before updating the transceiver a number of TAL PIB attributes need
	 * to be updated depending on the channel page.
	 */
	tal_pib.MaxFrameDuration = MAX_FRAME_DURATION;
	tal_pib.SHRDuration = NO_OF_SYMBOLS_PREAMBLE_SFD;
	tal_pib.SymbolsPerOctet = SYMBOLS_PER_OCTET;

	switch (ch_page) {
	case 0: /* compliant O-QPSK */
		trx_reg_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_250_KBPS);
		/* Apply compliant ACK timing */
		trx_reg_bit_write(SR_AACK_ACK_TIME, ACK_TIME_12_SYMBOLS);
		break;

	case 2: /* non-compliant OQPSK mode 1 */
		trx_reg_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_500_KBPS);
		/* Apply reduced ACK timing */
		trx_reg_bit_write(SR_AACK_ACK_TIME, ACK_TIME_2_SYMBOLS);
		break;

	case 16:    /* non-compliant OQPSK mode 2 */
		trx_reg_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_1_MBPS);
		/* Apply reduced ACK timing */
		trx_reg_bit_write(SR_AACK_ACK_TIME, ACK_TIME_2_SYMBOLS);
		break;

	case 17:    /* non-compliant OQPSK mode 3 */
		trx_reg_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_2_MBPS);
		/* Apply reduced ACK timing */
		trx_reg_bit_write(SR_AACK_ACK_TIME, ACK_TIME_2_SYMBOLS);
		break;

	default:
		return false;
	}

	return true;
}

#endif

/* EOF */
