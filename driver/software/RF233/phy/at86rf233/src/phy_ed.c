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

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "../../../pal/inc/pal.h"
#include "../../../phy/inc/phy.h"
#include "../../../phy/inc/phy_tasks.h"
#include "../../../phy/inc/ieee_phy_const.h"
#include "../../../phy/inc/phy_constants.h"
#include "../../at86rf/inc/at86rf.h"
#include "../../at86rf/inc/phy_internal.h"
#include "../../../phy/at86rf/inc/phy_irq_handler.h"
#include "../../../phy/at86rf/inc/phy_trx_reg_access.h"
/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/**
 * \addtogroup group_tal_ed_233
 * @{
 */

/* Constant define for the ED scaling: register value at -35dBm */
#define CLIP_VALUE_REG                  (56U)

#define MIN_ED_VAL                      (20U)
#define MAX_ED_VAL                      (83U)
/*
 * Scan duration formula: \f$aBaseSuperframeDuration (2^SD + 1)\f$
 * where \f$0 <= SD <= 14\f$
 */
#define CALCULATE_SYMBOL_TIME_SCAN_DURATION(SD)	\
	(aBaseSuperframeDuration * ((1UL << (SD)) + 1UL))

/* === GLOBALS ============================================================= */

/**
 * The peak_ed_level is the maximum ED value received from the transceiver for
 * the specified Scan Duration.
 */
static uint8_t max_ed_level;
static uint32_t sampler_counter;

/* === PROTOTYPES ========================================================== */

/* ! @} */

/* === IMPLEMENTATION ====================================================== */

/*
 * \brief Starts ED Scan
 *
 * This function starts an ED Scan for the scan duration specified by the
 * MAC layer.
 *
 * \param scan_duration Specifies the ED scan duration in symbols
 *
 * \return MAC_SUCCESS - ED scan duration timer started successfully
 *         TAL_BUSY - TAL is busy servicing the previous request from MAC
 *         TAL_TRX_ASLEEP - Transceiver is currently sleeping
 *         FAILURE otherwise
 */
PHY_Retval_t PHY_EdStart(uint8_t scan_duration)
{
	/*
	 * Check if the TAL is in idle state. Only in idle state it can
	 * accept and ED request from the MAC.
	 */
	if (PHY_IDLE != phy_info.tal_state) {
		if (phy_info.tal_trx_status == TRX_SLEEP) {
			return PHY_TRX_ASLEEP;
		} else {

			return PHY_BUSY;
		}
	}  
	/*
	 * Disable the transceiver interrupts to prevent frame reception
	 * while performing ED scan.
	 */
	pal_trx_irq_dis(); /* Disable transceiver main interrupt. */
    (void)set_trx_state(CMD_FORCE_PLL_ON);
	(void)trx_reg_read(RG_IRQ_STATUS);    /* Clear existing interrupts */
	trx_reg_bit_write(SR_RX_PDT_DIS, RX_DISABLE);
	trx_reg_bit_write(SR_IRQ_MASK, (uint8_t)TRX_IRQ_4_CCA_ED_DONE); /* enable
	                                                    * interrupt */
	pal_trx_irq_en(); /* Enable transceiver main interrupt. */
    
	/* Make sure that receiver is switched on. */
	if (set_trx_state(CMD_RX_ON) != RX_ON) {
		/* Restore previous configuration */
		trx_reg_bit_write(SR_RX_PDT_DIS, RX_ENABLE);
		
		trx_reg_write(RG_IRQ_MASK, (uint8_t)TRX_IRQ_DEFAULT); /* enable
		                                              * TRX_END
		                                              * interrupt */
		return PHY_FAILURE;
	}

	/* Perform ED in TAL_ED_RUNNING state. */
	phy_info.tal_state = PHY_ED_RUNNING;
    
    /* write dummy value to start measurement */
	trx_reg_write(RG_PHY_ED_LEVEL, 0xFF);

	max_ed_level = 0; /* reset max value */

	sampler_counter = CALCULATE_SYMBOL_TIME_SCAN_DURATION(scan_duration) /
			ED_SAMPLE_DURATION_SYM;

	return PHY_SUCCESS;
}

/**
 * \brief ED Scan Interrupt
 *
 * This function handles an ED done interrupt from the transceiver.
 *
 */
void trx_ed_irq_handler_cb(void)
{
	uint8_t ed_value;
	
		/* Read the ED Value. */
		ed_value = trx_reg_read(RG_PHY_ED_LEVEL);
    /*
     * Update the peak ED value received, if greater than the
     * previously
     * read ED value.
     */
    if (ed_value > max_ed_level) {
        max_ed_level = ed_value;
    }

    sampler_counter--;
    if (sampler_counter > 0U) {
        /* write dummy value to start measurement */
        trx_reg_write(RG_PHY_ED_LEVEL, 0xFF);
    } else {

        phy_info.tal_state = PHY_ED_DONE;
        //OSAL_SEM_PostISR(&semPhyRxInternalHandler);
        PHY_PostTask(false); 
    }
}

/*
 * \brief Scan done
 *
 * This function updates the max_ed_level and invokes the callback function
 * tal_ed_end_cb().
 *
 * \param parameter unused callback parameter
 */
void ed_scan_done(void)
{
	trx_reg_bit_write(SR_RX_PDT_DIS, RX_ENABLE);

	trx_reg_write(RG_IRQ_MASK,  (uint8_t)TRX_IRQ_DEFAULT); /* enable TRX_END
	                                              * interrupt */
	pal_trx_irq_en(); /* Enable transceiver main interrupt. */

	phy_info.tal_state = PHY_IDLE; /* ed scan is done */

	(void)set_trx_state(CMD_RX_AACK_ON);
    
#ifndef TRX_REG_RAW_VALUE
	/*
	 * Scale ED result.
	 * Clip values to 0xFF if > -35dBm
	 */
    if (max_ed_level > CLIP_VALUE_REG) {
		max_ed_level = 0xFF;
	} else {
		max_ed_level
			= (uint8_t)(((uint16_t)max_ed_level *
				0xFFU) / CLIP_VALUE_REG);
	}
#endif /* TRX_REG_RAW_VALUE */

	PHY_EdEndCallback(max_ed_level);
}






/* EOF */
