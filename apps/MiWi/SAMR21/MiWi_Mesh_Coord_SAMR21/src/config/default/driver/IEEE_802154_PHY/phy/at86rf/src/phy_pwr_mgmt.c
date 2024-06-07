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

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "../../../pal/inc/pal.h"
#include "../../../phy/inc/phy.h"
#include "../../../phy/inc/ieee_phy_const.h"
#include "../../at86rf/inc/at86rf.h"
#include "../../../phy/at86rf/inc/phy_internal.h"

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === GLOBALS ============================================================= */

/* === PROTOTYPES ========================================================== */

/* === IMPLEMENTATION ====================================================== */

/*
 * \brief Sets the transceiver to sleep
 *
 * This function sets the transceiver to sleep state.
 *
 * \param mode Defines sleep mode of transceiver: SLEEP_MODE_1 or
 * DEEP_SLEEP_MODE)
 *
 * \return   TAL_BUSY - The transceiver is busy in TX or RX
 *           MAC_SUCCESS - The transceiver is put to sleep
 *           TAL_TRX_ASLEEP - The transceiver is already asleep;
 *           either in SLEEP or in DEEP_SLEEP
 *           MAC_INVALID_PARAMETER - The specified sleep mode is not supported
 */
PHY_Retval_t PHY_TrxSleep(PHY_SleepMode_t mode)
{
	tal_trx_status_t trx_status;

#ifndef ENABLE_DEEP_SLEEP
	if (SLEEP_MODE_1 != mode)
#else
	if ((SLEEP_MODE_1 != mode) && (DEEP_SLEEP_MODE != mode))
#endif
	{
		return PHY_INVALID_PARAMETER;
	}


#ifdef ENABLE_DEEP_SLEEP
	if (((phy_info.tal_trx_status == TRX_SLEEP) && (mode == SLEEP_MODE_1)) || \
			((phy_info.tal_trx_status == TRX_DEEP_SLEEP) &&
			(mode == DEEP_SLEEP_MODE)))
#else
	if (phy_info.tal_trx_status == TRX_SLEEP)
#endif
	{
		return PHY_TRX_ASLEEP;
	}

	/* Device can be put to sleep only when the TAL is in IDLE state. */
	if (PHY_IDLE != phy_info.tal_state) {
		return PHY_BUSY;
	}

	phy_info.tal_rx_on_required = false;

	/*
	 * First set trx to TRX_OFF.
	 * If trx is busy, like ACK transmission, do not interrupt it.
	 */
	do {
		trx_status = set_trx_state(CMD_TRX_OFF);
	} while (trx_status != TRX_OFF);


#ifndef ENABLE_DEEP_SLEEP
	trx_status = set_trx_state(CMD_SLEEP);
#else
	if (mode == SLEEP_MODE_1) {
        trx_status = set_trx_state(CMD_SLEEP);
	} else { /* deep sleep */
        trx_status = set_trx_state(CMD_DEEP_SLEEP);
	}
#endif

#ifndef ENABLE_DEEP_SLEEP
	if (trx_status == TRX_SLEEP)
#else
	if ((trx_status == TRX_SLEEP) || (trx_status == TRX_DEEP_SLEEP))
#endif
	{
		return PHY_SUCCESS;
	} else {
		/* State could not be set due to TAL_BUSY state. */
		return PHY_BUSY;
	}
}

/*
 * \brief Wakes up the transceiver from sleep
 *
 * This function awakes the transceiver from sleep state.
 *
 * \return   TAL_TRX_AWAKE - The transceiver is already awake
 *           MAC_SUCCESS - The transceiver is woken up from sleep
 *           FAILURE - The transceiver did not wake-up from sleep
 */
PHY_Retval_t PHY_TrxWakeup(void)
{
	tal_trx_status_t trx_status;

#ifndef ENABLE_DEEP_SLEEP
	if (phy_info.tal_trx_status != TRX_SLEEP)
#else
	if ((phy_info.tal_trx_status != TRX_SLEEP) && (phy_info.tal_trx_status != TRX_DEEP_SLEEP))
#endif
	{
		return PHY_TRX_AWAKE;
	}
    

	trx_status = set_trx_state(CMD_TRX_OFF);

	if (trx_status == TRX_OFF) {
		return PHY_SUCCESS;
	} else {
		return PHY_FAILURE;
	}
}




void tal_trx_wakeup(void)
{ 
   /* The pending transceiver interrupts on the microcontroller are
		 * cleared. */
		/* Clear existing interrupts */
    (void)trx_reg_read(RG_IRQ_STATUS);
    /* Leave trx sleep mode. */
    TRX_SLP_TR_LOW();
    /* Poll wake-up interrupt flag until set within ISR. */
    while (!phy_info.tal_awake_end_flag) {
    }
      
}
/* EOF */
