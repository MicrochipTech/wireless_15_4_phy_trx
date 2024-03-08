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
#include "../../../phy/inc/phy.h"
#include "../../../phy/inc/ieee_phy_const.h"
#include "../../at86rf/inc/phy_pib.h"
#include "../../at86rf/inc/phy_irq_handler.h"
#include "../../../phy/inc/phy_constants.h"
#include "../../at86rf/inc/phy_tx.h"
#include "../../../resources/buffer/inc/bmm.h"
#include "../../../resources/queue/inc/qmm.h"
#include "../../at86rf/inc/phy_rx.h"
#include "../../../phy/at86rf/inc/phy_internal.h"
#include "../../at86rf/inc/at86rf.h"
#include "../../../phy/at86rf/inc/phy_trx_reg_access.h"
#include "../../../phy/inc/phy_tasks.h"



/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === GLOBALS ============================================================= */

static uint8_t tal_sw_retry_count;
static bool tal_sw_retry_no_csma_ca;
static trx_trac_status_t trx_trac_status;

extern TimerId_t TAL_RETRY_TIMER; 

void trxEIC_waitTimerCb(uintptr_t context);
SYS_TIME_HANDLE trxEIC_waitTimer;

/* === PROTOTYPES ========================================================== */
static void retransmissionTimerCallback(void);
/* === IMPLEMENTATION ====================================================== */

void trxEIC_waitTimerCb(uintptr_t context) 
{
    PHY_PostTask(false);  
}


/*
 * \brief Requests to TAL to transmit frame
 *
 * This function is called by the MAC to deliver a frame to the TAL
 * to be transmitted by the transceiver.
 *
 * \param tx_frame Pointer to the PHY_FrameInfo_t structure updated by the MAC
 * layer
 * \param csma_mode Indicates mode of csma-ca to be performed for this frame
 * \param perform_frame_retry Indicates whether to retries are to be performed
 * for
 *                            this frame
 *
 * \return MAC_SUCCESS  if the TAL has accepted the data from the MAC for frame
 *                 transmission
 *         TAL_BUSY if the TAL is busy servicing the previous MAC request
 */
  PHY_Retval_t PHY_TxFrame(PHY_FrameInfo_t *txFrame, PHY_CSMAMode_t csmaMode,
		bool performFrameRetry)
{
   
	if (phy_info.tal_state != PHY_IDLE) {
		return PHY_BUSY;
	}

	/*
	 * Store the pointer to the provided frame structure.
	 * This is needed for the callback function.
	 */
	mac_frame_ptr = txFrame;

	/* Set pointer to actual mpdu to be downloaded to the transceiver. */
	tal_frame_to_tx = txFrame->mpdu;
    phy_info.last_frame_length = tal_frame_to_tx[0] - 1U;
	/*
	 * In case the frame is too large, return immediately indicating
	 * invalid status.
	 */
	if (tal_frame_to_tx == NULL) {
		return PHY_INVALID_PARAMETER;
    }
    
	send_frame(csmaMode, performFrameRetry);

	return PHY_SUCCESS;
}


/*
 * \brief Implements the handling of the transmission end.
 *
 * This function handles the callback for the transmission end.
 */
void tx_done_handling(void)
{  
	phy_info.tal_state = PHY_IDLE;

	PHY_Retval_t status;

	switch (trx_trac_status) {
	case TRAC_SUCCESS:
		status = PHY_SUCCESS;
		break;

	case TRAC_SUCCESS_DATA_PENDING:
		status = PHY_FRAME_PENDING;
		break;

	case TRAC_CHANNEL_ACCESS_FAILURE:
		status = PHY_CHANNEL_ACCESS_FAILURE;
		break;

	case TRAC_NO_ACK:
		status = PHY_NO_ACK;
		break;

	case TRAC_INVALID:
		status = PHY_FAILURE;
		break;

	default:
		status = PHY_FAILURE;
		break;
	}

	PHY_TxDoneCallback(status, mac_frame_ptr);
    
} /* tx_done_handling() */

/*
 * \brief Sends frame
 *
 * \param use_csma Flag indicating if CSMA is requested
 * \param tx_retries Flag indicating if transmission retries are requested
 *                   by the MAC layer
 */
void send_frame(PHY_CSMAMode_t csmaMode, bool txRetries)
{
	tal_trx_status_t trx_status;

	/* Configure tx according to tx_retries */
	if (txRetries) {
		trx_reg_bit_write(SR_MAX_FRAME_RETRIES,
				tal_pib.MaxFrameRetries);
	} else {
		trx_reg_bit_write(SR_MAX_FRAME_RETRIES, 0);
	}

	/* Configure tx according to csma usage */
	if ((csmaMode == NO_CSMA_NO_IFS) || (csmaMode == NO_CSMA_WITH_IFS)) 
    {
		trx_reg_bit_write(SR_MAX_CSMA_RETRIES, 7); /* immediate
		                                        * transmission */
		if (txRetries) {
			tal_sw_retry_count = tal_pib.MaxFrameRetries;
			tal_sw_retry_no_csma_ca = true;
		}
	} else {
		trx_reg_bit_write(SR_MAX_CSMA_RETRIES, tal_pib.MaxCSMABackoffs);
	}
    
    	/* Handle interframe spacing */
	if (csmaMode == NO_CSMA_WITH_IFS) {
		if (phy_info.last_frame_length > aMaxSIFSFrameSize) {
			PAL_TimerDelay(PHY_CONVERT_SYMBOLS_TO_US(
					macMinLIFSPeriod_def)
					- TRX_IRQ_DELAY_US -
					PRE_TX_DURATION_US);
			phy_info.last_frame_length = 0;
		} else if (phy_info.last_frame_length > 0) {
			PAL_TimerDelay(PHY_CONVERT_SYMBOLS_TO_US(
					macMinSIFSPeriod_def)
					- TRX_IRQ_DELAY_US -
					PRE_TX_DURATION_US);
			phy_info.last_frame_length = 0;
		}else{
			/*DO NOTHING*/
		}
	}
	do {
		trx_status = set_trx_state(CMD_TX_ARET_ON);
	} while (trx_status != TX_ARET_ON);
	/*
	 * Send the frame to the transceiver.
	 * Note: The PhyHeader is the first byte of the frame to
	 * be sent to the transceiver and this contains the frame
	 * length.
	 * The actual length of the frame to be downloaded
	 * (parameter two of trx_frame_write)
	 * is
	 * 1 octet frame length octet
	 * + n octets frame (i.e. value of frame_tx[0])
	 * - 2 octets FCS
	 */

	trx_frame_write(tal_frame_to_tx, tal_frame_to_tx[0] - 1U);
    phy_info.tal_state = PHY_TX_AUTO;
    
    /* Toggle the SLP_TR pin triggering transmission. */
    TRX_SLP_TR_HIGH();
	trx_delay_micros(1);
    TRX_SLP_TR_LOW();

    uint8_t context = 0U;
    trxEIC_waitTimer = SYS_TIME_CallbackRegisterUS(&trxEIC_waitTimerCb, (uintptr_t)&context, 54000, SYS_TIME_SINGLE);
    if(trxEIC_waitTimer == SYS_TIME_HANDLE_INVALID)
    {
            return;
    }
     
    (void)trx_status;

}

/*
 * \brief Handles interrupts issued due to end of transmission
 *
 * \param underrun_occured  true if under-run has occurred
 */
void handle_tx_end_irq(bool underrun_occured)
{
   

	{

		/* Read trac status before enabling RX_AACK_ON. */
		if (underrun_occured) {
			trx_trac_status = TRAC_INVALID;
		} else {
			trx_trac_status = (trx_trac_status_t)trx_reg_bit_read(
					SR_TRAC_STATUS);
		}

		/* Trx has handled the entire transmission incl. CSMA */
		{
			if (tal_sw_retry_no_csma_ca && ((bool)tal_sw_retry_count) &&
					TRAC_NO_ACK == trx_trac_status ) {
				tal_trx_status_t trx_status;
				do {
					trx_status = set_trx_state(
							CMD_TX_ARET_ON);
				} while (trx_status != TX_ARET_ON);

				/* Toggle the SLP_TR pin triggering
				 * transmission. */
				TRX_SLP_TR_HIGH();
				trx_delay_micros(1);
				TRX_SLP_TR_LOW();
				uint8_t context = 0U;
    			trxEIC_waitTimer = SYS_TIME_CallbackRegisterUS(&trxEIC_waitTimerCb, (uintptr_t)&context, 54000, SYS_TIME_SINGLE);
    			if(trxEIC_waitTimer == SYS_TIME_HANDLE_INVALID)
    			{
            		return;
    			}
				if (--tal_sw_retry_count == 0U) {
					tal_sw_retry_no_csma_ca = false;
				}
			} else {
				phy_info.tal_state = PHY_TX_DONE; /* Further handling is
				                          * done by				                          * tx_done_handling()
				                         **/
                

                //tx_done_handling(); 
                phy_info.tal_rx_on_required = true;
                PHY_PostTask(false);
			}
		}
	}   
    
}


void tal_start_retransmission_timer(uint32_t us)
{
    
    if((bool)tal_sw_retry_count)
	{
    tal_sw_retry_count--;
	}
    
    if(tal_sw_retry_count > 0U)
    {
        if (PAL_SUCCESS == PAL_TimerStart(TAL_RETRY_TIMER,  us,
        TIMEOUT_RELATIVE,
        (void *)retransmissionTimerCallback,
        NULL, CALLBACK_SINGLE))
        {
            return;
        }
    }
    
        
    phy_info.tal_state = PHY_TX_DONE; /* Further handling is
                                      * done by				                          
                                      * tx_done_handling()
                                     **/
    //tx_done_handling(); 
    phy_info.tal_rx_on_required = true;
    PHY_PostTask(false);
}

static void retransmissionTimerCallback(void)
{
    tal_trx_status_t trx_status;
    
    do {
        trx_status = set_trx_state(
                CMD_TX_ARET_ON);
    } while (trx_status != TX_ARET_ON);

    /* Toggle the SLP_TR pin triggering
     * transmission. */
    TRX_SLP_TR_HIGH();
    trx_delay_micros(1);
    TRX_SLP_TR_LOW();
    
}
/* EOF */
