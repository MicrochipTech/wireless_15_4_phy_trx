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
#include <stdlib.h>
#include "../../../pal/inc/pal.h"
#include "../../../phy/inc/phy.h"
#include "../../../phy/inc/ieee_phy_const.h"
#include "../../../phy/inc/phy_constants.h"
#include "../../at86rf/inc/at86rf.h"
#include "../../../phy/at86rf/inc/phy_internal.h"


/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* Constant define for the ED scaling: register value at -35dBm */
#define CLIP_VALUE_REG                  (62)

/* === GLOBALS ============================================================= */

/**
 * TFA PIB attribute to reduce the Rx sensitivity.
 * Represents the Rx sensitivity value in dBm; example: -52
 */
static int8_t tfa_pib_rx_sens;

/* === PROTOTYPES ========================================================== */

static void init_tfa_pib(void);
static void write_all_tfa_pibs_to_trx(void);

/* === IMPLEMENTATION ====================================================== */

uint8_t txcwdata[128];



/*
 * \brief Perform a CCA
 *
 * This function performs a CCA request.
 *
 * \return phy_enum_t PHY_IDLE or PHY_BUSY
 */
PHY_Retval_t PHY_CCAPerform(void)
{
	tal_trx_status_t trx_status;
	uint8_t cca_status;
	uint8_t cca_done;

	/* Ensure that trx is not in SLEEP for register access */
	do {
		trx_status = set_trx_state(CMD_TRX_OFF);
	} while (trx_status != TRX_OFF);

	/* no interest in receiving frames while doing CCA */
	trx_reg_bit_write(SR_RX_PDT_DIS, RX_DISABLE); /* disable frame reception
	                                           * indication */

	/* Set trx to rx mode. */
	do {
		trx_status = set_trx_state(CMD_RX_ON);
	} while (trx_status != RX_ON);

	/* Start CCA */
	trx_reg_bit_write(SR_CCA_REQUEST, CCA_START);

	/* wait until CCA is done */
	trx_delay_micros(PHY_CONVERT_SYMBOLS_TO_US(CCA_DURATION_SYM));
	do {
		/* poll until CCA is really done */
		cca_done = trx_reg_bit_read(SR_CCA_DONE);
	} while (cca_done != CCA_COMPLETED);

	(void)set_trx_state(CMD_TRX_OFF);

	/* Check if channel was idle or busy. */
	if (trx_reg_bit_read(SR_CCA_STATUS) == CCA_CH_IDLE) {
		cca_status = (uint8_t)PHY_CHANNEL_IDLE;
	} else {
		cca_status = (uint8_t)PHY_CHANNEL_BUSY;
	}

	/* Enable frame reception again. */
	trx_reg_bit_write(SR_RX_PDT_DIS, RX_ENABLE);

	return ((PHY_Retval_t)cca_status);
}





/*
 * \brief Perform a single ED measurement
 *
 * \return ed_value Result of the measurement
 *         If the build switch TRX_REG_RAW_VALUE is defined, the transceiver's
 *         register value is returned.
 */
uint8_t PHY_EdSample(void)
{
	trx_irq_reason_t trx_irq_cause;
	uint8_t ed_value;
	tal_trx_status_t trx_status;
    
    phy_info.tal_state = PHY_ED_RUNNING;

	/* Make sure that receiver is switched on. */
	do {
		trx_status = set_trx_state(CMD_RX_ON);
	} while (trx_status != RX_ON);

	/*
	 * Disable the transceiver interrupts to prevent frame reception
	 * while performing ED scan.
	 */
	trx_reg_bit_write(SR_RX_PDT_DIS, RX_DISABLE);

	/* Write dummy value to start measurement. */
	trx_reg_write(RG_PHY_ED_LEVEL, 0xFF);

	/* Wait for ED measurement completion. */
	trx_delay_micros(PHY_CONVERT_SYMBOLS_TO_US(ED_SAMPLE_DURATION_SYM));
	do {
		trx_irq_cause
			= (trx_irq_reason_t)trx_reg_read(RG_IRQ_STATUS);
	} while ((trx_irq_reason_t)(((uint8_t)trx_irq_cause) & ((uint8_t)TRX_IRQ_4_CCA_ED_DONE)) !=
			TRX_IRQ_4_CCA_ED_DONE);

	/* Read the ED Value. */
	ed_value = trx_reg_read(RG_PHY_ED_LEVEL);

	/* Clear IRQ register */
	(void)trx_reg_read(RG_IRQ_STATUS);
	/* Enable reception agian */
	trx_reg_bit_write(SR_RX_PDT_DIS, RX_ENABLE);
	/* Switch receiver off again */
	(void)set_trx_state(CMD_TRX_OFF);

    phy_info.tal_state = PHY_IDLE;
	return ed_value;
}



/*
 * \brief Starts continuous transmission on current channel
 *
 * \param tx_mode Mode of continuous transmission (CW or PRBS)
 * \param random_content Use random content if true
 */
void PHY_StartContinuousTransmit(PHY_ContinuousTxMode_t txMode, bool randomContent)
{

	/* step 3,6: Channel is assumed to be set before */
	trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_TRX_OFF);
	/* step 7: Enable continuous transmission - step #1 */
	trx_reg_write(0x36, 0x0F);
	if (txMode == CW_MODE) {
		/* step 8: Register access: CW at Fc +/- 0.1 MHz */
		if (((tal_pib.CurrentPage == 0) || (tal_pib.CurrentPage == 2) || \
				(tal_pib.CurrentPage == 16) ||
				(tal_pib.CurrentPage == 17)) &&
				(tal_pib.CurrentChannel == 0)) {                                          /*
			                                                                                   *
			                                                                                   *
			                                                                                   *868.3MHz
			                                                                                   **/
			trx_reg_write(RG_TRX_CTRL_2, 0x0A); /* 400 kchip/s
			                                     * mode, step 8
			                                     * ,SUB_MODE = 0 */
		} else {
			trx_reg_write(RG_TRX_CTRL_2, 0x0E); /* 1000kchip/s */
		}

		txcwdata[0] = 1; /* length */
		txcwdata[1] = 0;
		/* step 9: Frame buffer access */
        trx_frame_write(txcwdata, 2);
	} else { /* PRBS mode */
		 /* step 8: */

		/*
		 * Step 8 is not explicitly written here, because the proper
		 * value is set during reset or by updating the Channel Page.
		 * After finishing CW/PRBS another reset is performed with
		 * parameter set_default_pib set to false, which restores the
		 * original value based on the current Channel Page.
		 *
		 * I.e., in order to use PRBS with a specific data rate,
		 * the Channel Page needs to be udpated before starting PRBS.
		 */

		txcwdata[0] = 127U; /* = max length */
	for (uint8_t data_index = 1U; data_index < 128U; data_index++) { 
			if (randomContent) {
				txcwdata[data_index] = (uint8_t)rand();
			} else {
				txcwdata[data_index] = 0;
			}
		}
		/* step 9: Frame buffer access */
		trx_frame_write(txcwdata, 128);
	}

	/* step 10: Enable continuous transmission - step #2 */
	trx_reg_write(RG_PART_NUM, 0x54);
	/* step 11: Enable continuous transmission - step #3 */
	trx_reg_write(RG_PART_NUM, 0x46);
	/* step 12, 13: Stwitch PLL on */
	(void)set_trx_state(CMD_PLL_ON);
	/* step 14: Initiate transmission using SLP_TR line */
	TRX_SLP_TR_HIGH();
	trx_delay_micros(1);
	TRX_SLP_TR_LOW();                                      
}



/*
 * \brief Stops continuous transmission
 */
void PHY_StopContinuousTransmit(void)
{

    
	(void)PHY_Reset(false);
}

/*
 * \brief Initializes the TFA
 *
 * This function is called to initialize the TFA.
 *
 * \return MAC_SUCCESS if everything went correct;
 *         FAILURE otherwise
 */
PHY_Retval_t tfa_init(void)
{
	init_tfa_pib();
	write_all_tfa_pibs_to_trx();

	return PHY_SUCCESS;
}

/*
 * \brief Reset the TFA
 *
 * This function is called to reset the TFA.
 *
 * \param set_default_pib Defines whether PIB values need to be set
 *                        to its default values
 */
void tfa_reset(bool set_default_pib)
{
	if (set_default_pib) {
		init_tfa_pib();
	}

	write_all_tfa_pibs_to_trx();
}

/**
 * \brief Initialize the TFA PIB
 *
 * This function initializes the TFA information base attributes
 * to their default values.
 * \ingroup group_tfa
 */
static void init_tfa_pib(void)
{
	tfa_pib_rx_sens = TFA_PIB_RX_SENS_DEF;
}

/**
 * \brief Write all shadow PIB variables to the transceiver
 *
 * This function writes all shadow PIB variables to the transceiver.
 * It is assumed that the radio does not sleep.
 * \ingroup group_tfa
 */
static void write_all_tfa_pibs_to_trx(void)
{
	(void)tfa_pib_set(TFA_PIB_RX_SENS, (void *)&tfa_pib_rx_sens);
}



/*
 * \brief Sets a TFA PIB attribute
 *
 * This function is called to set the transceiver information base
 * attributes.
 *
 * \param[in] tfa_pib_attribute TFA infobase attribute ID
 * \param[in] value TFA infobase attribute value to be set
 *
 * \return MAC_UNSUPPORTED_ATTRIBUTE if the TFA info base attribute is not found
 *         TAL_BUSY if the TAL is not in TAL_IDLE state.
 *         MAC_SUCCESS if the attempt to set the PIB attribute was successful
 */
PHY_Retval_t tfa_pib_set(PHY_tfa_pib_t tfa_pib_attribute, void *value)
{
	switch (tfa_pib_attribute) {
	case TFA_PIB_RX_SENS:
	{
		uint8_t reg_val;
		int8_t rssi_base_val;

		switch (tal_pib.CurrentPage) {
		case 0:     /* BPSK */
			if (tal_pib.CurrentChannel == 0) { /* BPSK20 */
				rssi_base_val = RSSI_BASE_VAL_BPSK_300_DBM;
			} else { /* BPSK40 */
				rssi_base_val = RSSI_BASE_VAL_BPSK_600_DBM;
			}

			break;

		case 2:     /* O-QPSK */
			if (tal_pib.CurrentChannel == 0) { /* OQPSK100 */
				rssi_base_val
					= RSSI_BASE_VAL_OQPSK_400_SIN_RC_DBM;
			} else { /* OQPSK250 */
				rssi_base_val
					= RSSI_BASE_VAL_OQPSK_1000_SIN_DBM;
			}

			break;

		case 5:     /* Chinese band */
			rssi_base_val = RSSI_BASE_VAL_OQPSK_1000_RC_DBM;
			break;

#ifdef HIGH_DATA_RATE_SUPPORT
		case 16:     /* non-compliant OQPSK mode 1 */
			if (tal_pib.CurrentChannel == 0) { /* 200kbps, EU */
				rssi_base_val
					= RSSI_BASE_VAL_OQPSK_400_SIN_RC_DBM;
			} else { /* 500kbps, NA */
				rssi_base_val
					= RSSI_BASE_VAL_OQPSK_1000_SIN_DBM;
			}

			break;

		case 17:     /* non-compliant OQPSK mode 2 */
			if (tal_pib.CurrentChannel == 0) { /* 400kbps, EU */
				rssi_base_val
					= RSSI_BASE_VAL_OQPSK_400_SIN_RC_DBM;
			} else { /* 1000kbps, NA */
				rssi_base_val
					= RSSI_BASE_VAL_OQPSK_1000_SIN_DBM;
			}

			break;

		case 18:        /* Chinese band, non-compliant mode 1 using
				 * O-QPSK 500 */
			rssi_base_val = RSSI_BASE_VAL_OQPSK_1000_RC_DBM;
			break;

		case 19:        /* Chinese band, non-compliant mode 2 using
				 * O-QPSK 1000 */
			rssi_base_val = RSSI_BASE_VAL_OQPSK_1000_RC_DBM;
			break;
#endif /* #ifdef HIGH_DATA_RATE_SUPPORT */
		default:

			/* is rather an unsupported channel page than an
			 * unsupported attribute
			 */
			return PHY_UNSUPPORTED_ATTRIBUTE;
			break;
		}

		tfa_pib_rx_sens = *((int8_t *)value);

		if (tfa_pib_rx_sens > (rssi_base_val + 45)) {
			reg_val = 0xF;
			tfa_pib_rx_sens = (rssi_base_val + 45);
		} else if (tfa_pib_rx_sens <= rssi_base_val) {
			reg_val = 0x0;
			tfa_pib_rx_sens = rssi_base_val;
		} else {
			int8_t temp = ((tfa_pib_rx_sens - (rssi_base_val)) / 3) + 1;
			reg_val = (uint8_t)temp;
		}

		trx_reg_bit_write(SR_RX_PDT_LEVEL, reg_val);
	}
	break;

	default:
		/* Invalid attribute id */
		return PHY_UNSUPPORTED_ATTRIBUTE;
		break;
	}

	return PHY_SUCCESS;
}

/*
 * \brief Gets a TFA PIB attribute
 *
 * This function is called to retrieve the transceiver information base
 * attributes.
 *
 * \param[in] tfa_pib_attribute TAL infobase attribute ID
 * \param[out] value TFA infobase attribute value
 *
 * \return MAC_UNSUPPORTED_ATTRIBUTE if the TFA infobase attribute is not found
 *         MAC_SUCCESS otherwise
 */
PHY_Retval_t tfa_pib_get(PHY_tfa_pib_t tfa_pib_attribute, void *value)
{
	switch (tfa_pib_attribute) {
	case TFA_PIB_RX_SENS:
		*(uint8_t *)value = tfa_pib_rx_sens;
		break;

	default:
		/* Invalid attribute id */
		return PHY_UNSUPPORTED_ATTRIBUTE;
		break;
	}

	return PHY_SUCCESS;
}





/* EOF */
