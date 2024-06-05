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
#include <stdbool.h>
#include <stddef.h>
#include "../../../pal/inc/pal.h"
#include "../../../phy/inc/phy.h"
#include "../../../phy/inc/ieee_phy_const.h"
#include "config/default/driver/IEEE_802154_PHY/resources/buffer/inc/bmm.h"
#include "config/default/driver/IEEE_802154_PHY/resources/queue/inc/qmm.h"
#include "../../../phy/at86rf/inc/phy_irq_handler.h"
#include "../../at86rf/inc/phy_rx.h"
#include "../../at86rf/inc/at86rf.h"
#include "../../../phy/at86rf/inc/phy_internal.h"
#include "../../../phy/inc/phy_constants.h"
#include "../../at86rf/inc/phy_tx.h"
#include "../../../phy/at86rf/inc/phy_trx_reg_access.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf/inc/phy_irq_handler.h"
#include "../../../phy/inc/phy_tasks.h"

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === GLOBALS ============================================================= */

/* === PROTOTYPES ========================================================== */

/* === IMPLEMENTATION ====================================================== */

/*
 * \brief Transceiver interrupt handler
 *
 * This function handles the transceiver generated interrupts.
 */
void trx_irq_handler_cb(void)
{
	trx_irq_reason_t trx_irq_cause;

	trx_irq_cause = (trx_irq_reason_t) trx_reg_read(RG_IRQ_STATUS);


    if ((bool)((uint8_t)trx_irq_cause & (uint8_t)TRX_IRQ_4_CCA_ED_DONE)) {
        if (phy_info.tal_state == PHY_ED_RUNNING){
            trx_ed_irq_handler_cb();  
        }
        
        else if (phy_info.tal_trx_status == TRX_SLEEP ){
            trx_irq_awake_handler_cb();
        }
		else{
            /*DO NOTHING*/ 
        }
           
    }
	else if ((bool)((uint8_t)trx_irq_cause & (uint8_t)TRX_IRQ_3_TRX_END)) {
		/*
		 * TRX_END reason depends on if the trx is currently used for
		 * transmission or reception.
		 */

        if (phy_info.tal_state == PHY_TX_AUTO)
		{
			/* Get the result and push it to the queue. */
			if ((bool)((uint8_t)trx_irq_cause & (uint8_t)TRX_IRQ_6_TRX_UR)) {
				handle_tx_end_irq(true); /* see tal_tx.c */
			} else {
				handle_tx_end_irq(false); /* see tal_tx.c */
			}
		} else { /* Other tal_state than TAL_TX_... */
			 /* Handle rx interrupt. */
			handle_received_frame_irq(); /* see tal_rx.c */
		}
	}
	else{
        /*DO NOTHING*/ 
    }
} /* trx_irq_handler_cb() */


/*
 * \brief Transceiver interrupt handler for awake end IRQ
 *
 * This function handles the transceiver awake end interrupt.
 */
void trx_irq_awake_handler_cb(void)
{

		/* Set the wake-up flag. */
		phy_info.tal_awake_end_flag = true;
		trx_reg_write(RG_IRQ_MASK, (uint8_t)TRX_IRQ_DEFAULT);

}


void EIC_interrupt_cb(uintptr_t context)
{
    
    TAL_PostTask(true);
    
}




/* EOF */
