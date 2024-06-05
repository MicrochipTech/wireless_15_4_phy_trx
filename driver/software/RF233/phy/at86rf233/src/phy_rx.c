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
#include "../../../phy/inc/ieee_phy_const.h"
#include "config/default/driver/IEEE_802154_PHY/resources/buffer/inc/bmm.h"
#include "config/default/driver/IEEE_802154_PHY/resources/queue/inc/qmm.h"
#include "../../../phy/inc/phy_constants.h"
#include "../../at86rf/inc/phy_pib.h"
#include "../../at86rf/inc/phy_irq_handler.h"
#include "../../at86rf/inc/at86rf.h"
#include "../../at86rf/inc/phy_rx.h"
#include "../../../phy/at86rf/inc/phy_internal.h"
#include "../../../phy/inc/phy_tasks.h"


/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* Constant defines for the LQI calculation */
#define ED_THRESHOLD                    (60)
#define ED_MAX                          (-RSSI_BASE_VAL_DBM - ED_THRESHOLD)
#define LQI_MAX                         (3)

#define ED_LEVEL_MAX_REG_VALUE          (84)
#define ED_LEVEL_MULTIPLIER             (255.0 / ED_LEVEL_MAX_REG_VALUE)

#define US_PER_OCTECT                   (32)


/* === GLOBALS ============================================================= */

/* === PROTOTYPES ========================================================== */

#ifndef TRX_REG_RAW_VALUE
#ifdef RSSI_TO_LQI_MAPPING
static inline uint8_t normalize_lqi(uint8_t ed_value);

#else
static inline uint8_t normalize_lqi(uint8_t lqi, uint8_t ed_value);

#endif
#endif /* #ifndef TRX_REG_RAW_VALUE */

/* === IMPLEMENTATION ====================================================== */

/*
 * \brief Handle received frame interrupt
 *
 * This function handles transceiver interrupts for received frames and
 * uploads the frames from the trx.
 */
void handle_received_frame_irq(void)
{
	/* Actual frame length of received frame. */
	uint8_t phy_frame_len;
	/* Extended frame length appended by LQI and ED. */
	uint8_t ext_frame_length;
	PHY_FrameInfo_t *receive_frame;
	uint8_t *frame_ptr;

	if (tal_rx_buffer == NULL) {

		/*
		 * Although the buffer protection mode is enabled and the
		 * receiver has
		 * been switched to PLL_ON, the next incoming frame was faster.
		 * It cannot be handled and is discarded. Reading anything from
		 * the
		 * frame resets the buffer protection mode.
		 */
		uint8_t dummy;
		trx_frame_read(&dummy, 1);
		return;
	}

	receive_frame = (PHY_FrameInfo_t *)BMM_BUFFER_POINTER(tal_rx_buffer);

#ifdef PROMISCUOUS_MODE
	if (tal_pib.PromiscuousMode) {
		/* Check for valid FCS */
		if (trx_reg_bit_read(SR_RX_CRC_VALID) == CRC16_NOT_VALID) {
			return;
		}
	}
#endif

#if (defined ENABLE_TRX_SRAM) || defined(ENABLE_TRX_SRAM_READ)
	/* Use SRAM read to keep rx safe mode armed. */
	trx_sram_read(0x00, &phy_frame_len, LENGTH_FIELD_LEN); /* 0x00: SRAM
	                                                        * offset
	                                                        * address */
#else

	/* Get frame length from transceiver. */
    trx_frame_read(&phy_frame_len, LENGTH_FIELD_LEN);

#endif
    
    phy_info.last_frame_length =  phy_frame_len; 
	/* Check for valid frame length. */
	if (phy_frame_len > 127U) {
		return;
	}

	/*
	 * The PHY header is also included in the frame (length field), hence
	 * the frame length
	 * is incremented.
	 * In addition to that, the LQI and ED value are uploaded, too.
	 */
    // phy_frame_len = 1(length byte(PHR))+9((MHR)+ 1(seq no.)+(length of data)
	ext_frame_length = phy_frame_len + LENGTH_FIELD_LEN + LQI_LEN +
			ED_VAL_LEN;

	/* Update payload pointer to store received frame. */
	frame_ptr = (uint8_t *)receive_frame + LARGE_BUFFER_SIZE -
			ext_frame_length ;

	/*
	 * Note: The following code is different from single chip
	 * transceivers, since reading the frame via SPI contains the length
	 * field
	 * in the first octet. RF233's frame buffer includes ED value too.
	 */
	trx_frame_read(frame_ptr,
			LENGTH_FIELD_LEN + phy_frame_len + LQI_LEN + ED_VAL_LEN);
    

	receive_frame->mpdu = frame_ptr;




	/* Append received frame to incoming_frame_queue and get new rx buffer.
	**/
	qmm_queue_append(&tal_incoming_frame_queue, tal_rx_buffer);

	/* The previous buffer is eaten up and a new buffer is not assigned yet.
	**/
	tal_rx_buffer = bmm_buffer_alloc(LARGE_BUFFER_SIZE);
/* Check if receive buffer is available */
	if (NULL == tal_rx_buffer) {
		/*
		 * Turn off the receiver until a buffer is available again.
		 * tal_task() will take care of eventually reactivating it.
		 * Due to ongoing ACK transmission do not force to switch it
		 * off.
		 */
        (void)set_trx_state(CMD_PLL_ON);
        phy_info.tal_rx_on_required = true;  
        
    }
	PHY_PostTask(false);
}

/*
 * \brief Parses received frame and create the PHY_FrameInfo_t structure
 *
 * This function parses the received frame and creates the PHY_FrameInfo_t
 * structure to be sent to the MAC as a parameter of tal_rx_frame_cb().
 *
 * \param buf Pointer to the buffer containing the received frame
 */

void process_incoming_frame(buffer_t *buf_ptr)
{
    
#ifndef TRX_REG_RAW_VALUE
	uint8_t frame_len;
	uint8_t *frame_ptr;
	uint8_t ed_level;
	uint8_t lqi;    
#endif

	PHY_FrameInfo_t *receive_frame
		= (PHY_FrameInfo_t *)BMM_BUFFER_POINTER(buf_ptr);
    

	/* The frame is present towards the end of the buffer. */

#ifndef TRX_REG_RAW_VALUE

	/*
	 * Store the last frame length for IFS handling.
	 * Substract LQI and length fields.
	 */
	frame_len = phy_info.last_frame_length = receive_frame->mpdu[0];
    
#else
	phy_info.last_frame_length = receive_frame->mpdu[0];
#endif

#ifdef PROMISCUOUS_MODE
	if (tal_pib.PromiscuousMode) {
#ifndef TRX_REG_RAW_VALUE
		frame_ptr = &(receive_frame->mpdu[frame_len + LQI_LEN]);

		/*
		 * The LQI is stored after the FCS.
		 * The ED value is stored after the LQI.
		 */
		lqi = *frame_ptr++;
        phy_info.last_pkt_lqi = lqi;
		ed_level = *frame_ptr;

		/*
		 * The LQI normalization is done using the ED level measured
		 * during
		 * the frame reception.
		 */
#ifdef RSSI_TO_LQI_MAPPING
		lqi = normalize_lqi(ed_level);
#else
		lqi = normalize_lqi(lqi, ed_level);
#endif

		/* Store normalized LQI value again. */
		frame_ptr--;
		*frame_ptr = lqi;
		phy_info.last_pkt_lqi = lqi;
#endif  /* #ifndef TRX_REG_RAW_VALUE */

		receive_frame->buffer_header = buf_ptr;

		/* The callback function implemented by MAC is invoked. */
		PHY_RxFrameCallback(receive_frame);

		return;
	}
#endif   /* #ifdef PROMISCUOUS_MODE */



#ifndef TRX_REG_RAW_VALUE

	/*
	 * The LQI is stored after the FCS.
	 * The ED value is stored after the LQI.
	 */
	frame_ptr = &(receive_frame->mpdu[frame_len + LQI_LEN]);
	lqi = *frame_ptr++;
	ed_level = *frame_ptr;
    
//    *frame_ptr = ed_level = phy_info.last_pkt_ed_level;

	/*
	 * The LQI normalization is done using the ED level measured during
	 * the frame reception.
	 */
#ifdef RSSI_TO_LQI_MAPPING
	lqi = normalize_lqi(ed_level);
#else
	lqi = normalize_lqi(lqi, ed_level);
#endif

	/* Store normalized LQI value again. */
	frame_ptr--;
	*frame_ptr = lqi;
#endif  /* #ifndef TRX_REG_RAW_VALUE */

	receive_frame->buffer_header = buf_ptr;

	/* The callback function implemented by MAC is invoked. */
	PHY_RxFrameCallback(receive_frame);
 
} /* process_incoming_frame() */

#ifndef TRX_REG_RAW_VALUE
#ifdef RSSI_TO_LQI_MAPPING

/**
 * \brief Normalize LQI
 *
 * This function normalizes the LQI value based on the RSSI/ED value.
 *
 * \param ed_value Read ED value
 *
 * \return The calculated/normalized LQI value: ppduLinkQuality
 */
static inline uint8_t normalize_lqi(uint8_t ed_value)
{
	/*
	 * Scale ED result.
	 */
	if (ed_value > (ED_LEVEL_MAX_REG_VALUE - 1)) {
		return 0xFF;
	} else {
		/* Scale ED value to span up to 0xFF. */
		return (uint8_t)(ed_value * ED_LEVEL_MULTIPLIER + 0.5);
	}
}

#else /* #ifdef RSSI_TO_LQI_MAPPING */

/**
 * \brief Normalize LQI
 *
 * This function normalizes the LQI value based on the ED and
 * the originally appended LQI value.
 *
 * \param lqi Measured LQI
 * \param ed_value Read ED value
 *
 * \return The calculated LQI value: ppduLinkQuality
 */
static inline uint8_t normalize_lqi(uint8_t lqi, uint8_t ed_value)
{
	uint16_t link_quality;
	uint8_t lqi_star;
	uint8_t ed_max_val = (uint8_t)ED_MAX;

#ifdef HIGH_DATA_RATE_SUPPORT
	if (tal_pib.CurrentPage == 0) {
#endif
	if (ed_value > ed_max_val) {
		ed_value = ed_max_val;
	} else if (ed_value == 0U) {
		ed_value = 1;
	}else{
        /*DO NOTHING*/
    }

	lqi_star = lqi >> 6U;
	link_quality = (uint16_t)lqi_star * (uint16_t)ed_value * 255U /
			(ed_max_val * LQI_MAX);

	if (link_quality > 255U) {
		return 255;
	} else {
		return (uint8_t)link_quality;
	}

#ifdef HIGH_DATA_RATE_SUPPORT
} else {    /* if (tal_pib.CurrentPage == 0) */
	/* High data rate modes do not provide a valid LQI value. */
	if (ed_value > ed_max_val) {
		return 0xFF;
	} else {
		return (ed_value * (255U / ed_max_val));
	}
}
#endif
}
#endif /* #ifdef RSSI_TO_LQI_MAPPING */
#endif /* #ifndef TRX_REG_RAW_VALUE */

/*  EOF */
