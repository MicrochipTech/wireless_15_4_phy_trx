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



/* Prevent double inclusion */
#ifndef PHY_INTERNAL_H
#define PHY_INTERNAL_H

/* === INCLUDES ============================================================ */

#include "../../../resources/buffer/inc/bmm.h"
#include "../../../resources/queue/inc/qmm.h"
#include "../../../phy/inc/phy_constants.h"
#include "phy_trx_reg_access.h"
#include "../../../phy/inc/phy.h"
#include "at86rf.h"
#include "xc.h"
#include "definitions.h"



/**
 * \ingroup group_tal
 * \defgroup group_tal_233 AT86RF233 Transceiver Abstraction Layer
 * The AT86RF233 is a feature rich, low-power 2.4 GHz radio transceiver designed
 * for industrial
 *  and consumer ZigBee/IEEE 802.15.4, 6LoWPAN, RF4CE and high data rate sub
 * 1GHz  ISM band applications
 * The Transceiver Abstraction Layer (TAL) implements the transceiver specific
 * functionalities and
 * provides interfaces to the upper layers (like IEEE 802.15.4 MAC )and  uses
 * the services of PAL.
 */

/**
 * \ingroup group_tal_233
 * \defgroup group_tal_state_machine_233 TAL State Machine
 * The different operating states of the Transceiver are controlled by the TAL
 * state machine.
 *
 */

/**
 * \ingroup group_tal_233
 * \defgroup group_tal_init_233  TAL Initialization and reset
 * Performs initialization and reset functionalities of the transceiver
 *
 */

/**
 * \ingroup group_tal_233
 * \defgroup group_tal_ed_233   TAL Energy Detection
 * Performs the ED scan functionalities.
 *
 */

/**
 * \ingroup group_tal_233
 * \defgroup group_tal_irq_233 Transceiver Interrupt Handling
 * Handles Transceiver related Interrupts.
 *
 */

/**
 * \ingroup group_tal_233
 * \defgroup group_tal_pib_233   TAL PIB Storage
 * The PIB(Pan Information Base) attributes related to the TAL are Stored and
 * handled  by the TAL PIB storage.
 *
 */

/**
 * \ingroup group_tal_233
 * \defgroup group_tal_tx_233   TAL Frame Transmission Unit
 * The Frame Transmission Unit generates and transmits the frames using PAL .
 *
 */

/**
 * \ingroup group_tal_tx_233
 * \defgroup group_tal_tx_csma_233   TAL CSMA/CA Module
 * Performs channel access mechanism for frame transmission
 * For Detailed information refer  CSMA-CA algorithm section of IEEE Std
 * 802.15.4-2006
 *
 */

/**
 * \ingroup group_tal_233
 * \defgroup group_tal_rx_233   TAL Frame Reception Unit
 * The Frame Reception Unit reads/uploads the incoming frames .
 *
 */

/* === TYPES =============================================================== */

/** TAL states */

typedef enum tal_state_tag {
	PHY_IDLE           = 0,
	PHY_TX_AUTO        = 1,
	PHY_TX_DONE        = 2,
	PHY_SLOTTED_CSMA   = 3,
	PHY_ED_RUNNING     = 4,
	PHY_ED_DONE        = 5
            
}  tal_state_t;


/* Structure implementing the PIB values stored in TAL */
typedef __PACKED_STRUCT tal_pib_tag {
	/**
	 * 64-bit (IEEE) address of the node.
	 */
	uint64_t IeeeAddress;

	/**
	 * Supported channels
	 *
	 * Legacy trx:
	 * Bit mask, whereas each bit position set indicates that the channel,
	 * corresponding to this particular bit position, is actually supported
	 *
	 * Multi-Trx devices:
	 * Min channel: Low word of variable SupportedChannels:
	 *(uint16_t)(SupportedChannels)
	 * Max channel: High word of variable SupportedChannels:
	 *(uint16_t)(SupportedChannels >> 16)
	 */
	uint32_t SupportedChannels;

	/**
	 * 16-bit short address of the node.
	 */
	uint16_t ShortAddress;

	/**
	 * 16-bit PAN ID
	 */
	uint16_t PANId;

	/**
	 * Maximum number of symbols in a frame:
	 * = phySHRDuration + ceiling([aMaxPHYPacketSize + 1] x
	 * phySymbolsPerOctet)
	 */
	uint16_t MaxFrameDuration;

	/**
	 * CCA Mode
	 */
	uint8_t CCAMode;

	/**
	 * Current RF channel to be used for all transmissions and receptions.
	 */

	uint8_t CurrentChannel;

	/**
	 * The maximum number of back-offs the CSMA-CA algorithm will attempt
	 * before declaring a CSMA_CA failure.
	 */
	uint8_t MaxCSMABackoffs;

	/**
	 * The minimum value of the backoff exponent BE in the CSMA-CA
	 * algorithm.
	 */
	uint8_t MinBE;

	/**
	 * Indicates if the node is a PAN coordinator or not.
	 */
	bool PrivatePanCoordinator;

	/**
	 * Default value of transmit power of transceiver
	 * using IEEE defined format of phyTransmitPower.
	 */
	uint8_t TransmitPower;

	/**
	 * Current channel page.
	 */
	uint8_t CurrentPage;

	/**
	 * Duration of the synchronization header (SHR) in symbols for the
	 * current PHY.
	 */
	uint8_t SHRDuration;

	/**
	 * Number of symbols per octet for the current PHY.
	 */
	uint8_t SymbolsPerOctet;

	/**
	 * The maximum value of the backoff exponent BE in the CSMA-CA
	 * algorithm.
	 */
	uint8_t MaxBE;

	/**
	 * The maximum number of retries allowed after a transmission failure.
	 */
	uint8_t MaxFrameRetries;
    
#ifdef PROMISCUOUS_MODE

	/**
	 * Promiscuous Mode
	 */
	bool PromiscuousMode;
#endif

} tal_pib_t;


/* parameter types in transceiver */
typedef struct phy_config_param_tag {
    bool antDiversity;
    bool antSelect;
    uint8_t antCtrl;
    bool extPACtrl;
    bool aackPromMode;
    int8_t txPwr;
    uint8_t rxSens;
    bool rxRPC;
    bool rxSafeMode;
    bool rxAutoAck;
    bool rxReservedFrame;
    bool reservedFrameFiltering;
}phy_config_param_t;

typedef struct phy_info_tag{
    tal_state_t tal_state;
    tal_trx_status_t tal_trx_status;
    bool tal_rx_on_required;  
    uint8_t last_frame_length;
    volatile bool tal_awake_end_flag;
    phy_config_param_t phy_config_param;
    uint32_t phyVersion;
    uint8_t last_pkt_ed_level;
    uint8_t last_pkt_lqi;
}phy_info_t;

#define RX_PRIORITY_ARB_SET 0x01
#define TX_PRIORITY_ARB_SET 0x01

#define DELAY_OK 0x00
#define TIME_SENSITIVE 0x01




#define TRX_SLP_TR_HIGH()               SLP_TR_Set()

#define TRX_SLP_TR_LOW()                SLP_TR_Clear()

#define TRX_RST_HIGH()                  _RST_Set()

#define TRX_RST_LOW()                  _RST_Clear()

#define TRX_SEL_HIGH()                  SPI_SS_Set()

#define TRX_SEL_LOW()                   SPI_SS_Clear()


/*
 * Default value of custom TAL PIB channel page
 */
#define PHY_CURRENT_PAGE_DEFAULT                      (0x00)

/*
 * Default value of maximum number of symbols in a frame
 */
#define PHY_MAX_FRAME_DURATION_DEFAULT                (MAX_FRAME_DURATION)

/*
 * Default value of duration of the synchronization header (SHR) in symbols
 * for the current PHY
 */
#define PHY_SHR_DURATION_DEFAULT                      (NO_OF_SYMBOLS_PREAMBLE_SFD)

/*
 * Default value of number of symbols per octet for the current PHY
 */
#define PHY_SYMBOLS_PER_OCTET_DEFAULT                 (SYMBOLS_PER_OCTET)

/*
 * Default value of maximum backoff exponent used while performing csma ca
 */
#define PHY_MAXBE_DEFAULT                             (0x05)

/*
 * Default value of PIB attribute macMaxFrameRetries
 */
#define PHY_MAXFRAMERETRIES_DEFAULT                   (0x03)

/*
 * Default value of maximum csma ca backoffs
 */
#define PHY_MAX_CSMA_BACKOFFS_DEFAULT                  (0x04)

/*
 * Default value of minimum backoff exponent used while performing csma ca
 */
#define PHY_MINBE_DEFAULT                              (0x03)

/*
 * Value of a broadcast PAN ID
 */
#define PHY_PANID_BC_DEFAULT                           (0xFFFF)

/*
 * Default value of short address
 */
#define PHY_SHORT_ADDRESS_DEFAULT                      (0xFFFF)

/*
 * Default value of current channel in TAL
 */
#define PHY_CURRENT_CHANNEL_DEFAULT                    (0x0B)

/*
 * Default value of promiscuous mode in TAL
 */
#define PHY_PIB_PROMISCUOUS_MODE_DEFAULT               (false)

#ifndef CUSTOM_DEFAULT_TX_PWR

/*
 * Default value of transmit power of transceiver: Use highest tx power
 */
#define PHY_TRANSMIT_POWER_DEFAULT          (TX_PWR_TOLERANCE | 0x04U)
#endif

/*
 * Default value CCA mode
 */
#define PHY_CCA_MODE_DEFAULT                          (TRX_CCA_MODE1)


/*
 * Default value beacon order set to 15
 */
#define TAL_BEACON_ORDER_DEFAULT            (15)

/*
 * Default value supeframe order set to 15
 */
#define TAL_SUPERFRAME_ORDER_DEFAULT        (15)

/*
 * Default value of BeaconTxTime
 */
#define TAL_BEACON_TX_TIME_DEFAULT          (0x00000000)

/*
 * Default value of BatteryLifeExtension.
 */
#define PHY_BATTERY_LIFE_EXTENSION_DEFAULT  (false)

/*
 * Default value of PAN Coordiantor custom TAL PIB
 */
#define PHY_PAN_COORDINATOR_DEFAULT         (false)

#ifndef ANTENNA_DEFAULT
#define ANTENNA_DEFAULT                 (ANT_CTRL_1)
#endif

#ifdef ENABLE_QUEUE_CAPACITY
#define PHY_INCOMING_FRAME_QUEUE_CAPACITY   (255)
#endif  /* ENABLE_QUEUE_CAPACITY */

#define NUMBER_OF_PHY_TIMERS                (1)


/* === EXTERNALS =========================================================== */

/* Global TAL variables */
extern tal_pib_t tal_pib;
extern PHY_FrameInfo_t *mac_frame_ptr;
extern queue_t tal_incoming_frame_queue;
extern uint8_t *tal_frame_to_tx;
extern buffer_t *tal_rx_buffer;
extern phy_info_t phy_info;

/* === MACROS ============================================================== */

/**
 * Conversion of number of PSDU octets to duration in microseconds
 */
#ifdef HIGH_DATA_RATE_SUPPORT
#define TAL_PSDU_US_PER_OCTET(octets) \
	( \
		tal_pib.CurrentPage == 0 ? ((uint16_t)(octets) * 32) : \
		( \
			tal_pib.CurrentPage == 2 ? ((uint16_t)(octets) * 16) : \
			( \
				tal_pib.CurrentPage == \
				16 ? ((uint16_t)(octets) * \
				8) : ((uint16_t)(octets) * 4) \
			) \
		) \
	)
#else   /* #ifdef not HIGH_DATA_RATE_SUPPORT */
#define TAL_PSDU_US_PER_OCTET(octets)       ((uint16_t)(octets) * 32)
#endif

#define TRX_IRQ_DEFAULT     TRX_IRQ_3_TRX_END


/* === PROTOTYPES ========================================================== */

/*
 * Prototypes from tal.c
 */

/**
 * \brief Sets transceiver state
 *
 * \param trx_cmd needs to be one of the trx commands
 *
 * \return current trx state
 * \ingroup group_tal_state_machine_233
 */
tal_trx_status_t set_trx_state(trx_cmd_t trx_cmd);

/*
 * Prototypes from tal_init.c
 */
void trx_config(void);

tal_trx_status_t tal_get_trx_status(void);

/*
 * Prototypes from tal_ed.c
 */


/**
 * \brief Scan done
 *
 * This function updates the max_ed_level and invokes the callback function
 * tal_ed_end_cb().
 *
 * \ingroup group_tal_ed
 */
void ed_scan_done(void);
void trx_ed_irq_handler_cb(void);
void tal_trx_wakeup(void);
void trx_delay_micros(uint32_t us);
void trx_delay_millis(uint32_t ms);
void trx_delay_loop(void *hw, uint32_t cycles);
void trx_irq_flag_clear(void);
PHY_Retval_t tal_dump_registers(uint16_t start_addr, uint16_t end_addr,
		uint8_t *value);
PHY_Retval_t tal_set_frequency_regs(uint8_t cc_band, uint8_t cc_number);

PHY_Retval_t tal_set_frequency(float frequency);

PHY_Retval_t tal_calculate_frequency(uint8_t cc_band, uint8_t cc_number,
		float *freq);

#endif /* TAL_INTERNAL_H */
