/**
 * @file tal_irq_handler.c
 *
 * @brief This file handles the interrupt generated by the transceiver
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

/* === INCLUDES ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "config/default/driver/IEEE_802154_PHY/pal/inc/pal.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_config.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_internal.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/ieee_const.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/trx_access_2.h"
/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === EXTERNALS =========================================================== */

/* === GLOBALS ============================================================= */

/* === PROTOTYPES ========================================================== */

#ifdef IQ_RADIO
static void switch_rf_to_txprep(trx_id_t trx_id);

#endif

/* === IMPLEMENTATION ====================================================== */

/**
 * @brief Transceiver interrupt handler
 *
 * This function handles the transceiver interrupt. It reads all IRQs from the
 * transceivers and stores them to a variable. If a transceiver is currently
 * sleeping, then the IRQs are not handled.
 * The actual processing of the IRQs is triggered from tal_task().
 */
void trx_irq_handler_cb(void)
{
	/* Get all IRQS values */
	uint8_t irqs_array[4];

	trx_read( RG_RF09_IRQS, irqs_array, 4);

	/* Handle BB IRQS */
	for (trx_id_t trx_id = (trx_id_t)0; trx_id < NUM_TRX; trx_id++) {
		if (tal_state[trx_id] == TAL_SLEEP) {
			continue;
		}

		bb_irq_t irqs = (bb_irq_t)irqs_array[trx_id + 2];

		if (irqs != BB_IRQ_NO_IRQ) {
			if (irqs & BB_IRQ_RXEM) {
				irqs &= (uint8_t)(~((uint32_t)BB_IRQ_RXEM)); /*
				                                              * avoid
				                                              * Pa091 */
			}

			if (irqs & BB_IRQ_RXAM) {
				irqs &= (uint8_t)(~((uint32_t)BB_IRQ_RXAM)); /*
				                                              * avoid
				                                              * Pa091 */
			}

			if (irqs & BB_IRQ_AGCR) {
#if ((defined RF215v1) && (defined SUPPORT_LEGACY_OQPSK))
				/* Workaround for errata reference #4908 */
				/* Keep flag set to trigger workaround; see
				 *tal.c */
#else
				irqs &= (uint8_t)(~((uint32_t)BB_IRQ_AGCR)); /*
				                                              * avoid
				                                              * Pa091 */
#endif
			}

			if (irqs & BB_IRQ_AGCH) {
#if ((defined RF215v1) && (defined SUPPORT_LEGACY_OQPSK))
				/* Workaround for errata reference #4908 */
				/* Keep flag set to trigger workaround; see
				 *tal.c */
#else
				irqs &= (uint8_t)(~((uint32_t)BB_IRQ_AGCH)); /*
				                                              * avoid
				                                              * Pa091 */
#endif
			}

			if (irqs & BB_IRQ_RXFS) {
#ifdef ENABLE_TSTAMP
				// pal_get_current_time(&fs_tstamp[trx_id]);
				fs_tstamp[trx_id] = PAL_GetCurrentTime();
#endif
#if ((defined RF215v1) && (defined SUPPORT_LEGACY_OQPSK))
				/* Workaround for errata reference #4908 */
				/* Keep flag set to trigger workaround; see
				 *tal.c */
#else
				irqs &= (uint8_t)(~((uint32_t)BB_IRQ_RXFS)); /*
				                                              * avoid
				                                              * Pa091 */
#endif
			}

			if (irqs & BB_IRQ_RXFE) {
				// pal_get_current_time(&rxe_txe_tstamp[trx_id]);
				rxe_txe_tstamp[trx_id] = PAL_GetCurrentTime();

#if (defined RF215v1) && (!defined BASIC_MODE)
				/* Workaround for errata reference #4830 */
				/* Check if ACK transmission is actually
				 *requested by the received frame */
				uint16_t buf_reg_offset = BB_RX_FRM_BUF_OFFSET *
						trx_id;
				uint8_t fcf0 = trx_reg_read(
						buf_reg_offset + RG_BBC0_FBRXS);
				if ((fcf0 & FCF_ACK_REQUEST) == 0x00) {
					/* Ensure ACK is not transmitted */
					uint16_t offset = RF_BASE_ADDR_OFFSET *
							trx_id;
					trx_bit_write(
							offset + SR_BBC0_AMCS_AACK,
							0);
					trx_bit_write(
							offset + SR_BBC0_AMCS_AACK,
							1);
				}

#endif
			}

			if (irqs & BB_IRQ_TXFE) {
				/* used for IFS and for MEASURE_ON_AIR_DURATION
				 **/
//				pal_get_current_time(&rxe_txe_tstamp[trx_id]);
                rxe_txe_tstamp[trx_id] = PAL_GetCurrentTime();
			}

			/*
			 * Store remaining flags to global TAL variable and
			 * handle them within tal_task()
			 */
			tal_bb_irqs[trx_id] |= irqs;
		}
        else
        {
            tal_bb_irqs[trx_id] |= irqs;//modified by Deepthi
        }
	}

	/* Handle RF IRQS */
	for (trx_id_t trx_id = (trx_id_t)0; trx_id < NUM_TRX; trx_id++) {
		if (tal_state[trx_id] == TAL_SLEEP) {
			continue;
		}

		rf_irq_t irqs = (rf_irq_t)irqs_array[trx_id];

		if (irqs != RF_IRQ_NO_IRQ) {
			if (irqs & RF_IRQ_TRXRDY) {
				irqs &= (uint8_t)(~((uint32_t)RF_IRQ_TRXRDY)); /*
				                                                * avoid
				                                                * Pa091 */
			}

			if (irqs & RF_IRQ_TRXERR) {
				irqs &= (uint8_t)(~((uint32_t)RF_IRQ_TRXERR)); /*
				                                                * avoid
				                                                * Pa091 */
			}

			if (irqs & RF_IRQ_BATLOW) {
			}

			if (irqs & RF_IRQ_WAKEUP) {
			}

			if (irqs & RF_IRQ_IQIFSF) {
			}

			if (irqs & RF_IRQ_EDC) {
			}

			tal_rf_irqs[trx_id] |= irqs;
		}
        else
        {
            tal_bb_irqs[trx_id] |= irqs;//modified by Deepthi
        }
	}
    PHY_PostTask(false);
} /* trx_irq_handler_cb() */

#ifdef IQ_RADIO
void bb_irq_handler_cb(void)
{
	/* Get all IRQS values */
	uint8_t irqs_array[4];

	trx_read(RF215_BB, RG_RF09_IRQS, irqs_array, 4);

	/* Handle BB IRQS */
	for (trx_id_t trx_id = (trx_id_t)0; trx_id < NUM_TRX; trx_id++) {
		if (tal_state[trx_id] == TAL_SLEEP) {
			continue;
		}

		uint8_t irqs = irqs_array[trx_id + 2];

		if (irqs != BB_IRQ_NO_IRQ) {
			if (irqs & BB_IRQ_RXEM) {
				irqs &= (uint8_t)(~((uint32_t)BB_IRQ_RXEM)); /*
				                                              * avoid
				                                              * Pa091 */
			}

			if (irqs & BB_IRQ_RXAM) {
				irqs &= (uint8_t)(~((uint32_t)BB_IRQ_RXAM)); /*
				                                              * avoid
				                                              * Pa091 */
			}

			if (irqs & BB_IRQ_AGCR) {
				irqs &= (uint8_t)(~((uint32_t)BB_IRQ_AGCR)); /*
				                                              * avoid
				                                              * Pa091 */
				uint16_t reg_offset = RF_BASE_ADDR_OFFSET *
						trx_id;
				/* Release AGC */
				trx_bit_write(RF215_RF,
						reg_offset + SR_RF09_AGCC_FRZC,
						0);

#if (defined RF215v1) && (!defined BASIC_MODE)
				/* Workaround for errata reference #4830 */
				if ((irqs & BB_IRQ_RXFE) == 0) {
					uint16_t reg_offset
						= RF_BASE_ADDR_OFFSET *
							trx_id;
					trx_bit_write(
							reg_offset + SR_BBC0_AMCS_AACK,
							0);
					trx_bit_write(
							reg_offset + SR_BBC0_AMCS_AACK,
							1);
				}

#endif
			}

			if (irqs & BB_IRQ_AGCH) {
				irqs &= (uint8_t)(~((uint32_t)BB_IRQ_AGCH)); /*
				                                              * avoid
				                                              * Pa091 */
				/* Hold AGC */
				uint16_t reg_offset = RF_BASE_ADDR_OFFSET *
						trx_id;

				trx_bit_write(RF215_RF,
						reg_offset + SR_RF09_AGCC_FRZC,
						1);
			}

			if (irqs & BB_IRQ_RXFS) {
#ifdef ENABLE_TSTAMP
				pal_get_current_time(&fs_tstamp[trx_id]);
#endif
				irqs &= (uint8_t)(~((uint32_t)BB_IRQ_RXFS)); /*
				                                              * avoid
				                                              * Pa091 */
			}

			if (irqs & BB_IRQ_RXFE) {
				pal_get_current_time(&rxe_txe_tstamp[trx_id]);
				/* Wait for TXPREP and clear TRXRDY IRQ */
				switch_rf_to_txprep((trx_id_t)trx_id);
			}

			if (irqs & BB_IRQ_TXFE) {
				/* used for IFS and for MEASURE_ON_AIR_DURATION
				 **/
				pal_get_current_time(&rxe_txe_tstamp[trx_id]);
				/* BB interrupt handles further processing */
			}

			/*
			 * Store remaining flags to global TAL variable and
			 * handle them within tal_task()
			 */
			tal_bb_irqs[trx_id] |= irqs;
		}
	}

	/* Handle RF IRQS */
	for (trx_id_t trx_id = (trx_id_t)0; trx_id < NUM_TRX; trx_id++) {
		if (tal_state[trx_id] == TAL_SLEEP) {
			continue;
		}

		uint8_t irqs = irqs_array[trx_id];

		if (irqs != RF_IRQ_NO_IRQ) {
			if (irqs & RF_IRQ_TRXRDY) {
				irqs &= (uint8_t)(~((uint32_t)RF_IRQ_TRXRDY)); /*
				                                                * avoid
				                                                * Pa091 */
			}

			if (irqs & RF_IRQ_TRXERR) {
			}

			if (irqs & RF_IRQ_BATLOW) {
				irqs &= (uint8_t)(~((uint32_t)RF_IRQ_BATLOW)); /*
				                                                * avoid
				                                                * Pa091 */
			}

			if (irqs & RF_IRQ_WAKEUP) {
				irqs &= (uint8_t)(~((uint32_t)RF_IRQ_WAKEUP)); /*
				                                                * avoid
				                                                * Pa091 */
			}

			if (irqs & RF_IRQ_IQIFSF) {
			}

			if (irqs & RF_IRQ_EDC) {
				irqs &= (uint8_t)(~((uint32_t)RF_IRQ_EDC)); /*
				                                             * avoid
				                                             * Pa091 */
			}

			if (irqs != 0) {
			}

			tal_rf_irqs[trx_id] |= irqs;
		}
	}
} /* bb_irq_handler_cb() */

#endif

#ifdef IQ_RADIO
void rf_irq_handler_cb(void)
{
	/* Get all IRQS values */
	uint8_t irqs_array[4];

	trx_read(RF215_RF, RG_RF09_IRQS, irqs_array, 4);

	/* Handle BB IRQS */
	for (trx_id_t trx_id = (trx_id_t)0; trx_id < NUM_TRX; trx_id++) {
		if (tal_state[trx_id] == TAL_SLEEP) {
			continue;
		}

		uint8_t irqs = irqs_array[trx_id + 2];

		if (irqs != BB_IRQ_NO_IRQ) {
			if (irqs & BB_IRQ_RXEM) {
			}

			if (irqs & BB_IRQ_RXAM) {
			}

			if (irqs & BB_IRQ_AGCR) {
			}

			if (irqs & BB_IRQ_AGCH) {
			}

			if (irqs & BB_IRQ_RXFS) {
			}

			if (irqs & BB_IRQ_RXFE) {
			}

			if (irqs & BB_IRQ_TXFE) {
			}

			if (irqs != 0) {
			}

			/*
			 * Store remaining flags to global TAL variable and
			 * handle them within tal_task()
			 */
			tal_bb_irqs[trx_id] |= irqs;
		}
	}

	/* Handle RF IRQS */
	for (trx_id_t trx_id = (trx_id_t)0; trx_id < NUM_TRX; trx_id++) {
		if (tal_state[trx_id] == TAL_SLEEP) {
			continue;
		}

		uint8_t irqs = irqs_array[trx_id];

		if (irqs != RF_IRQ_NO_IRQ) {
			if (irqs & RF_IRQ_TRXRDY) {
				irqs &= (uint8_t)(~((uint32_t)RF_IRQ_TRXRDY)); /*
				                                                * avoid
				                                                * Pa091 */
			}

			if (irqs & RF_IRQ_TRXERR) {
			}

			if (irqs & RF_IRQ_BATLOW) {
			}

			if (irqs & RF_IRQ_WAKEUP) {
			}

			if (irqs & RF_IRQ_IQIFSF) {
			}

			if (irqs & RF_IRQ_EDC) {
			}

			tal_rf_irqs[trx_id] |= irqs;
		}
	}
} /* rf_irq_handler_cb() */

#endif /* #ifdef IQ_RADIO */

#ifdef IQ_RADIO
static void switch_rf_to_txprep(trx_id_t trx_id)
{
	uint16_t reg_offset = RF_BASE_ADDR_OFFSET * trx_id;
	trx_reg_write(RF215_RF, reg_offset + RG_RF09_CMD, RF_TXPREP);
	/* Wait for TXPREP */
	rf_cmd_state_t state;
	do {
		state = (rf_cmd_state_t)trx_reg_read(RF215_RF,
				reg_offset +
				RG_RF09_STATE);
	} while (state != RF_TXPREP);
	/* Clear TRXRDY interrupt */
	uint8_t irqs = trx_reg_read(RF215_RF, trx_id + RG_RF09_IRQS);
	tal_rf_irqs[trx_id] |= irqs & ((uint8_t)(~((uint32_t)RF_IRQ_TRXRDY))); /*
	                                                                        * avoid
	                                                                        * Pa091 */
	pal_dev_irq_flag_clr(RF215_RF);
}

#endif /* #ifdef IQ_RADIO */
/* EOF */
