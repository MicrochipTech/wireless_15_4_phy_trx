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
#include "../../../pal/inc/pal.h"
#include "../../at86rf/inc/phy_tx.h"
#include "../../../phy/inc/phy.h"
#include "../../../phy/inc/phy_constants.h"
#include "../../at86rf/inc/at86rf.h"
#include "config/default/driver/IEEE_802154_PHY/resources/buffer/inc/bmm.h"
#include "config/default/driver/IEEE_802154_PHY/resources/queue/inc/qmm.h"
#include "../../at86rf/inc/phy_rx.h"
#include "../../../phy/at86rf/inc/phy_internal.h"
#include "../../../phy/inc/phy_tasks.h"

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === GLOBALS ============================================================= */

/* === PROTOTYPES ========================================================== */

/* === IMPLEMENTATION ====================================================== */

/*
 * \brief Switches receiver on or off
 *
 * This function switches the receiver on (PHY_RX_ON) or off (PHY_TRX_OFF).
 *
 * \param state New state of receiver
 *
 * \return TAL_BUSY if the TAL state machine cannot switch receiver on or off,
 *         TRX_OFF if receiver has been switched off, or
 *         RX_ON otherwise.
 *
 */
PHY_TrxStatus_t PHY_RxEnable(PHY_TrxState_t state)
{
    
    
	/*
	 * Trx can only be enabled if TAL is not busy;
	 * i.e. if TAL is IDLE.
	 */
	if (PHY_IDLE != phy_info.tal_state) {

		return PHY_BUSY_TX;

	}

	if (state == PHY_STATE_TRX_OFF) {
		/*
		 * If the rx needs to be switched off, we are not interested in
		 * a frame
		 * that is currently being received.
		 * This must not be a Forced TRX_OFF (CMD_FORCED_TRX_OFF) since
		 * this could
		 * corrupt an already outoing ACK frame.
		 */
		(void)set_trx_state(CMD_TRX_OFF);
		phy_info.tal_rx_on_required = false;
		return PHY_TRX_OFF;
	} else {
		if (NULL != tal_rx_buffer) {
#ifdef PROMISCUOUS_MODE
			if (tal_pib.PromiscuousMode) {
				(void)set_trx_state(CMD_RX_ON);
                while(((uint8_t)tal_get_trx_status() & 0x1FU) != (uint8_t)RX_ON);
			} else {
				(void)set_trx_state(CMD_RX_AACK_ON);
                while(((uint8_t)tal_get_trx_status() & 0x1FU) != (uint8_t)RX_AACK_ON)
				{
					/*WAIT*/
				}
			}   
#else   /* Normal operation */
			(void)set_trx_state(CMD_RX_AACK_ON);
            while(((uint8_t)tal_get_trx_status() & 0x1FU) != (uint8_t)RX_AACK_ON)
			{
				/*WAIT*/
			}
#endif  /* PROMISCUOUS_MODE */

		} else {
			/*
			 * If no rx buffer is available, the corresponding
			 * information is stored and will be used by tal_task()
			 * to
			 * switch on the receiver later.
			 *
			 * Even if a receive buffer is not available,
			 * the TAL returns MAC_SUCCESS. The TAL will try to
			 * allocate a receive
			 * buffer as soon as possible and will switch on the
			 * receiver.
			 */
			phy_info.tal_rx_on_required = true;
            PHY_PostTask(false);
		}
		return PHY_RX_ON; /* MAC layer assumes RX_ON as return value */
	}
}

/* EOF */
