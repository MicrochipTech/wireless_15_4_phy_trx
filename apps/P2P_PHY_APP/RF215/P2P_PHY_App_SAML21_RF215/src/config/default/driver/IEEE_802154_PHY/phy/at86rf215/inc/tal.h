/**
 * @file tal.h
 *
 * @brief This file contains TAL API function declarations
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
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
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef TAL_H
#define TAL_H

/* === INCLUDES ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include "config/default/driver/IEEE_802154_PHY/pal/inc/pal.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_config.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_generic.h"
#include "config/default/driver/IEEE_802154_PHY/resources/buffer/inc/bmm.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/stack_config.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_types.h"
#include "config/default/driver/IEEE_802154_PHY/resources/queue/inc/qmm.h"
//#if (TAL_TYPE == AT86RF215)
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_rf215.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/ieee_154g.h"
//#endif

/**
 * \defgroup group_tal  Transceiver Abstraction Layer
 * The Transceiver Abstraction Layer (TAL) implements the transceiver specific
 * functionalities and
 * provides interfaces to the upper layers (like IEEE 802.15.4 MAC )and  uses
 * the services of PAL.
 *
 */

/* === TYPES =============================================================== */

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

#if defined(BEACON_SUPPORT)

	/**
	 * Holds the time at which last beacon was transmitted or received.
	 */
	uint32_t BeaconTxTime;
#endif  /* BEACON_SUPPORT */

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
#if (TAL_TYPE == AT86RF215)
	uint16_t CurrentChannel;
#else
	uint8_t CurrentChannel;
#endif

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
#if (TAL_TYPE == AT86RF215)
	int8_t TransmitPower;
#else
	uint8_t TransmitPower;
#endif

#if defined(BEACON_SUPPORT)

	/**
	 * Indication of whether battery life extension is enabled or not.
	 */
	bool BattLifeExt;

	/**
	 * Beacon order
	 */
	uint8_t BeaconOrder;

	/**
	 * Superframe order
	 */
	uint8_t SuperFrameOrder;
#endif  /* BEACON_SUPPORT */

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
#if (TAL_TYPE == AT86RF215)

	/**
	 * Current number of frames received during backoff periods; valid
	 * duriing
	 * ongoing transmission only.
	 */
	uint8_t NumRxFramesDuringBackoff;

	/**
	 * Maximum number of frames that are allowed to be received during CSMA
	 * backoff periods for a tx transaction.
	 */
	uint8_t MaxNumRxFramesDuringBackoff;

	/**
	 * PHY mode
	 */
	phy_t phy;

#if (defined SUPPORT_FSK) || (defined SUPPORT_OQPSK)

	/**
	 * Reduce power consumption mode; effective for FSK and MR-OQPSK
	 */
	bool RPCEnabled;
#endif

	/**
	 * The maximum number of symbols in a frame:
	 * = phySHRDuration + ceiling([aMaxPHYPacketSize + 1] x
	 * phySymbolsPerOctet)
	 */
	uint16_t MaxFrameDuration_sym;

	/**
	 * The duration of the synchronization header (SHR) in symbols for the
	 * current
	 * PHY.
	 */
	/* uint16_t SHRDuration_sym; */

	/**
	 * The number of symbols per octet for the current PHY.
	 */
	/* uint8_t SymbolsPerOctet; */

	/**
	 * The duration for CCA, specified in symbols.
	 */
	uint16_t CCADuration_sym;

	/**
	 * This parameter determines how to calculate
	 * the time required to perform CCA detection.
	 */
	bool CCATimeMethod;

#ifdef SUPPORT_OQPSK

	/**
	 * MR-O-QPSK rate mode
	 */
	oqpsk_rate_mode_t OQPSKRateMode;
#endif

#ifdef SUPPORT_LEGACY_OQPSK

	/**
	 * Proprietary high rate mode for legacy O-QPSK
	 */
	bool HighRateEnabled;
#endif

#ifdef SUPPORT_FSK

	/**
	 * A value of TRUE indicates that FEC is turned on. A value of FALSE
	 * indicates
	 * that FEC is turned off. This attribute is only valid for the MR-FSK
	 * PHY.
	 */
	bool FSKFECEnabled;

	/**
	 * A value of TRUE indicates that interleaving is enabled for RSC. A
	 * value of
	 * FALSE indicates that interleaving is disabled for RSC. This attribute
	 * is
	 * only valid for the MR-FSK PHY.
	 */
	bool FSKFECInterleavingRSC;

	/**
	 * A value of zero indicates that a nonrecursive and nonsystematic code
	 *(NRNSC)
	 * is employed. A value of one indicates that a recursive and systematic
	 * code
	 * (RSC) is employed. This attribute is only valid for the MR-FSK PHY.
	 */
	bool FSKFECScheme;

	/**
	 * The number of 1-octet patterns in the preamble.
	 * This attribute is only valid for the MR-FSK PHY.
	 */
	uint16_t FSKPreambleLength;

	/**
	 * Minimum FSK preamble length used for RPC.
	 * This attribute is only valid for the MR-FSK PHY.
	 */
	uint16_t FSKPreambleLengthMin;

	/**
	 * Determines which group of SFDs is used.
	 * This attribute is only valid for the MR-FSK PHY.
	 */
	bool MRFSKSFD;

	/**
	 * A value of FALSE indicates that data whitening of the PSDU is
	 * disabled.
	 * A value of TRUE indicates that data whitening of the PSDU is enabled.
	 * This attribute is only valid for the MR-FSK PHY.
	 */
	bool FSKScramblePSDU;
#ifdef SUPPORT_MODE_SWITCH

	/**
	 * A value of TRUE instructs the PHY entity to send a mode switch PPDU
	 * first and then a
	 * following PPDU that contains the PSDU using the associated mode
	 * switch parameters.
	 * This attribute is only valid for the MR-FSK PHY.
	 */
	bool ModeSwitchEnabled;

	/**
	 * The settling delay, in us, between the end of the final symbol of the
	 * PPDU
	 * initiating the mode switch and the start of the PPDU transmitted
	 * using the
	 * new PHY mode.
	 */
	uint16_t ModeSwitchSettlingDelay;

	/**
	 * PHY for the new mode following the mode switch PPDU
	 */
	new_phy_t ModeSwitchNewMode;

	/**
	 * Receive duration of the new mode after mode switch packet reception.
	 * Unit: microseconds
	 */
	uint32_t ModeSwitchDuration;
#endif /* #ifdef SUPPORT_MODE_SWITCH */
#ifdef SUPPORT_FSK_RAW_MODE

	/**
	 * Feature to enable FSK raw mode handling
	 */
	bool FSKRawModeEnabled;

	/**
	 * Expected Rx frame length in raw mode
	 */
	uint16_t FSKRawModeRxLength;
#endif
#endif

	/**
	 * The list of channel numbers supported when phyCurrentPage = 7 or 8.
	 */
	/* uint8_t SUNChannelsSupported; */

	/**
	 * The maximum channel number supported by the device.
	 * This attribute is only valid if phyCurrentPage equals 7 or 8.
	 */
	/* uint16_t MaxSUNChannelSupported; */

	/**
	 * The number of SUN channel page entries supported by the device.
	 */
	/* uint8_t NumSUNPageEntriesSupported; */

	/**
	 * Each entry in the list contains the description of a frequency band,
	 * modulation scheme, and particular PHY mode implemented by the device.
	 */
	/* uint8_t SUNPageEntriesSupported; */

	/**
	 * Defines the current frequency band, modulation scheme, and particular
	 * PHY
	 * mode when phyCurrentPage = 7 or 8.
	 */
	uint8_t CurrentSUNPageEntry;

	/**
	 * The number of GenericPHYDescriptor entries supported by the device
	 */
	/* uint8_t SUNNumGenericPHYDescriptors; */

	/**
	 * A table of GenericPHYDescriptor entries, where each entry is used to
	 * define
	 * a channel page 10 PHY mode.
	 */
	/* uint8_t SUNGenericPHYDescriptors; */

	/**
	 * The number of current entries in phyModeSwitchParameterEntries.
	 */
	/* uint8_t NumModeSwitchParameterEntries; */

	/**
	 * An array of up to four rows, where each row consists of a set of
	 * ModeSwitchDescriptor entries. This attribute is only valid for the
	 * MR-FSK
	 * PHY.
	 */
	/* uint8_t ModeSwitchParameterEntries; */

#ifdef SUPPORT_OFDM

	/**
	 * A value of zero indicates an interleaving depth of one symbol.
	 * A value of one indicates an interleaving depth of the number of
	 * symbols
	 * equal to the frequency domain spreading factor (SF).
	 * This attribute is only valid for the MR-OFDM PHY.
	 */
	bool OFDMInterleaving;

	/**
	 * OFDM MCS value
	 */
	ofdm_mcs_t OFDMMCS;
#endif

	/**
	 * The duration of the PHR, in symbols, for the current PHY.
	 * This attribute is only valid for the MR-OFDM PHY and MR-O-QPSK PHY.
	 */
	/* uint8_t PHRDuration_sym; */

	/**
	 * The time required to perform CCA detection; in us
	 */
	uint16_t CCADuration_us;

	/**
	 * The type of the FCS. A value of zero indicates a 4-octet FCS. A value
	 * of
	 * one indicates a 2-octet FCS. This attribute is only valid for SUN
	 * PHYs.
	 */
	bool FCSType;

	/**
	 * Length of the FCS field. If FCSType is 0, length field is 4 octets.
	 */
	uint8_t FCSLen;

	/**
	 * Symbol duration for the current PHY in us - read-only
	 */
	uint16_t SymbolDuration_us;

	/**
	 * ED scan duration for the current PHY in us - read-only
	 */
	uint8_t ScanDuration_us;

	/**
	 * ACK timing in us - read-only
	 */
	uint16_t ACKTiming;

	/**
	 * ACK wait duration in us - read-only
	 */
	uint16_t ACKWaitDuration;

#ifdef MEASURE_ON_AIR_DURATION

	/**
	 * ACK duration in us - read-only
	 */
	uint16_t ACKDuration_us;
#endif

	/**
	 * Maximum PHY packet size
	 */
	uint16_t MaxPHYPacketSize;

	/**
	 * CCA Threshold, register value
	 */
	int8_t CCAThreshold;

	/**
	 * Duration of an PSDU octet in us
	 */
	uint16_t OctetDuration_us;

#ifdef MEASURE_ON_AIR_DURATION

	/**
	 * Duration of PHY header (SHR + PHR) in us
	 */
	uint16_t PhyHeaderDuration_us;

	/**
	 * On the air duration measured in us
	 */
	uint32_t OnAirDuration;
#endif
#ifdef SUPPORT_ACK_RATE_MODE_ADAPTION

	/**
	 * Adapt data rate of the incoming frame to use for ACK transmission
	 */
	bool AdaptDataRateForACK;
#endif
#ifdef SUPPORT_FRAME_FILTER_CONFIGURATION

	/**
	 * Frame filter, frames types
	 */
	uint8_t frame_types;

	/**
	 * Frame filter, frames versions
	 */
	uint8_t frame_versions;
#endif
#endif /* #if (TAL_TYPE == AT86RF215) */
#ifdef MEASURE_TIME_OF_FLIGHT

	/**
	 * Time of flight
	 */
	uint32_t TimeOfFlight;
#endif

	/**
	 * Settling duration of the AGC
	 */
	uint16_t agc_settle_dur;
} tal_pib_t;

/* Portable channel type */
typedef uint16_t channel_t;
/**
 * @brief Globally used frame information structure
 *
 * @ingroup apiMacTypes
 */
//typedef struct frame_info_tag
//{
//#if (TAL_TYPE == AT86RF215)
//	/** Trx id of transceiver handling frame */
//	trx_id_t trx_id;
//#endif
//	/** Pointer to buffer header of frame */
//	buffer_t *buffer_header;
//
//
//#ifdef GTS_SUPPORT
//	queue_t *gts_queue;
//#endif /* GTS_SUPPORT */
//#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP)
//
//	/** Timestamp information of frame
//	 * The time stamping is only required for beaconing networks
//	 * or if time stamping is explicitly enabled.
//	 */
//	uint32_t time_stamp;
//#endif  /* #if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP) */
//#if (TAL_TYPE == AT86RF215)
//	/** MPDU length - does not include CRC length */
//	uint16_t len_no_crc;
//#endif
//	/** Pointer to MPDU */
//	uint8_t *mpdu;
//} PHY_FrameInfo_t;

/**
 * Sleep Mode supported by transceiver
 */
//typedef enum sleep_mode_tag {
//	SLEEP_MODE_1
//#ifdef ENABLE_DEEP_SLEEP
//	,
//	DEEP_SLEEP_MODE
//#endif
//} sleep_mode_t;

//typedef enum phy_trx_state_tag{
//    /* Transceiver to be configured to Transceiver OFF state*/
//    PHY_STATE_TRX_OFF,
//    /* Transceiver to be configured to Receiver ON state */
//    PHY_STATE_RX_ON
//}PHY_TrxState_t;

/**
 * CSMA Mode supported by transceiver
 */
//typedef enum csma_mode_tag {
//	NO_CSMA_NO_IFS,
//	NO_CSMA_WITH_IFS,
//	CSMA_UNSLOTTED,
//	CSMA_SLOTTED
//} PHY_CSMAMode_t;

//typedef enum retval_tag
//{
//	PHY_SUCCESS                 = 0x00, /**< Success defined by 802.15.4 */
//
//#ifdef ENABLE_RTB
//	RTB_SUCCESS                 = 0x10, /**< Success of ranging procedure */
//	RTB_RANGING_IN_PROGRESS     = 0x11, /**< Ranging procedure is already in
//	                                     * progress */
//	RTB_REJECT                  = 0x12, /**< Ranging procedure is rejected
//	                                    **/
//	RTB_UNSUPPORTED_ATTRIBUTE   = 0x13, /**< Unsupported attribute for RTB
//	                                    **/
//	RTB_INVALID_PARAMETER       = 0x14, /**< Unsupported parameter for RTB
//	                                    **/
//	RTB_OUT_OF_BUFFERS          = 0x15, /**< No further buffers available
//	                                     * for RTB ranging measurement */
//	RTB_UNSUPPORTED_RANGING     = 0x16, /**< Ranging is currently not
//	                                     * supported */
//	RTB_UNSUPPORTED_METHOD      = 0x17, /**< Requested Ranging method is
//	                                     * currently not supported at
//	                                     * reflector */
//	RTB_TIMEOUT                 = 0x18, /**< Timeout since requested Ranging
//	                                     * response frame is not received */
//#endif
//
//	PHY_TRX_ASLEEP              = 0x81, /**< Transceiver is currently
//	                                     * sleeping */
//	PHY_TRX_AWAKE               = 0x82, /**< Transceiver is currently awake
//	                                    **/
//	PHY_FAILURE                     = 0x85, /**< General failure condition */
//	PHY_BUSY                    = 0x86, /**< TAL busy condition */
//	PHY_FRAME_PENDING           = 0x87, /**< Frame pending at TAL */
//	QUEUE_FULL                  = 0x8C, /**< Requested queue is full */
//
//	PHY_CHANNEL_ACCESS_FAILURE  = 0xE1, /**< Channel access failure defined
//	                                     * by 802.15.4 */
//
//	PHY_INVALID_PARAMETER       = 0xE8, /**< Invalid Parameter defined by
//	                                     * 802.15.4 */
//	PHY_NO_ACK                  = 0xE9, /**< No ACK defined by 802.15.4 */
//
//	PHY_UNSUPPORTED_ATTRIBUTE   = 0xF4, /**< Unsupported attribute defined
//	                                     * by 802.15.4 */
//
//	/* The CCA attempt has detected a busy channel.*/
//    PHY_CHANNEL_BUSY            = 0x8A,
//	/* The CCA attempt has detected an idle channel.*/	
//    PHY_CHANNEL_IDLE            = 0x8B,
//	/* 15.4 subsytem request is aborted due to BT subsystem High priority request */                                 
//    PHY_RF_REQ_ABORTED          = 0x83, 
//	/* RF is not available for 802.15.4 subsytem*/
//    PHY_RF_UNAVAILABLE          = 0x84, 
//}PHY_Retval_t;
/* === EXTERNALS =========================================================== */

#if (defined SW_CONTROLLED_CSMA) && (defined TX_OCTET_COUNTER)

/**
 * Counter of transmitted octets
 */
extern uint32_t tal_tx_octet_cnt;
#endif
#define FLASH_DECLARE(x)  const x
#define FLASH_EXTERN(x) extern const x
#define PGM_READ_BYTE(x) *(x)
#define PGM_READ_WORD(x) *(x)
#define PGM_READ_DWORD(x) *(x)
#define MEMCPY_ENDIAN memcpy
#define PGM_READ_BLOCK(dst, src, len) memcpy((dst), (src), (len))
/* Converting of values from little endian to CPU endian. */
#define LE16_TO_CPU_ENDIAN(x)   (x)
#define LE32_TO_CPU_ENDIAN(x)   (x)
#define LE64_TO_CPU_ENDIAN(x)   (x)

/* Converting of constants from little endian to CPU endian. */
#define CLE16_TO_CPU_ENDIAN(x)  (x)
#define CLE32_TO_CPU_ENDIAN(x)  (x)
#define CLE64_TO_CPU_ENDIAN(x)  (x)

/* Converting of constants from CPU endian to little endian. */
#define CCPU_ENDIAN_TO_LE16(x)  (x)
#define CCPU_ENDIAN_TO_LE32(x)  (x)
#define CCPU_ENDIAN_TO_LE64(x)  (x)

#define ADDR_COPY_DST_SRC_16(dst, src)  ((dst) = (src))
#define ADDR_COPY_DST_SRC_64(dst, src)  ((dst) = (src))
/**
 * @brief Converts a 64-Bit value into  a 8 Byte array
 *
 * @param[in] value 64-Bit value
 * @param[out] data Pointer to the 8 Byte array to be updated with 64-Bit value
 * @ingroup apiPalApi
 */
//static inline void convert_64_bit_to_byte_array(uint64_t value, uint8_t *data)
//{
//    uint8_t val_index = 0;
//
//    while (val_index < 8)
//    {
//        data[val_index++] = value & 0xFF;
//        value = value >> 8;
//    }
//}

/**
 * @brief Converts a 16-Bit value into  a 2 Byte array
 *
 * @param[in] value 16-Bit value
 * @param[out] data Pointer to the 2 Byte array to be updated with 16-Bit value
 * @ingroup apiPalApi
 */
//static inline void convert_16_bit_to_byte_array(uint16_t value, uint8_t *data)
//{
//    data[0] = value & 0xFF;
//    data[1] = (value >> 8) & 0xFF;
//}

/* Converts a 16-Bit value into a 2 Byte array */
static inline void convert_spec_16_bit_to_byte_array(uint16_t value, uint8_t *data)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
}

/* Converts a 16-Bit value into a 2 Byte array */
static inline void convert_16_bit_to_byte_address(uint16_t value, uint8_t *data)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
}

/*
 * @brief Converts a 2 Byte array into a 16-Bit value
 *
 * @param data Specifies the pointer to the 2 Byte array
 *
 * @return 16-Bit value
 * @ingroup apiPalApi
 */
static inline uint16_t convert_byte_array_to_16_bit(uint8_t *data)
{
    return (data[0] | ((uint16_t)data[1] << 8));
}

/* Converts a 8 Byte array into a 32-Bit value */
static inline uint32_t convert_byte_array_to_32_bit(uint8_t *data)
{
	union
	{
		uint32_t u32;
		uint8_t u8[8];
	}long_addr;
	uint8_t index;
	for (index = 0; index < 4; index++)
	{
		long_addr.u8[index] = *data++;
	}
	return long_addr.u32;
}

/**
 * @brief Converts a 8 Byte array into a 64-Bit value
 *
 * @param data Specifies the pointer to the 8 Byte array
 *
 * @return 64-Bit value
 * @ingroup apiPalApi
 */
static inline uint64_t convert_byte_array_to_64_bit(uint8_t *data)
{
    union
    {
        uint64_t u64;
        uint8_t u8[8];
    } long_addr;

    uint8_t val_index;

    for (val_index = 0; val_index < 8; val_index++)
    {
        long_addr.u8[val_index] = *data++;
    }

    return long_addr.u64;
}
#if (TAL_TYPE == AT86RF215)
#   define NUM_TRX                      2
#else
#   define NUM_TRX                      1
#endif

#ifdef MULTI_TRX_SUPPORT
extern tal_pib_t tal_pib[NUM_TRX];
#else
extern tal_pib_t tal_pib;
#endif

/* === MACROS ============================================================== */
#ifdef ENABLE_QUEUE_CAPACITY
#define TAL_QUEUE_CAPACITY 200
#endif
/* RF bands: */

/**
 * 868 / 910 MHz (channels 0 through 10)
 * using BPSK
 */
#define BAND_900                            (0)

/**
 * 2.4 GHz (channels 11 through 26)
 */
#define BAND_2400                           (1)

/**
 * 450 / 900 MHz and 2.4 GHz - Multiple band support
 */
#define BAND_MULTIPLE                       (2)

#if (TAL_TYPE == AT86RF230A) || (TAL_TYPE == AT86RF230B) || \
	(TAL_TYPE == AT86RF231) || (TAL_TYPE == AT86RF232) || \
	(TAL_TYPE == ATMEGARFA1) || (TAL_TYPE == AT86RF233) || \
	(TAL_TYPE == ATMEGARFR2)

/** RF band */
#define RF_BAND                             BAND_2400
#elif (TAL_TYPE == AT86RF212) || (TAL_TYPE == AT86RF212B)
#define RF_BAND                             BAND_900
#elif (TAL_TYPE == AT86RF215)
#define RF_BAND                             BAND_MULTIPLE
#else
#error "Missing RF_BAND define"
#endif

/*
 * Channel numbers and channel masks for scanning.
 */
#if (RF_BAND == BAND_2400)
/** Minimum channel */
#define MIN_CHANNEL                 (11)
/** Maximum channel */
#define MAX_CHANNEL                 (26)
/** Valid channel masks for scanning */
#define VALID_CHANNEL_MASK          (0x07FFF800UL)
#else   /* 900 MHz */
#define MIN_CHANNEL                 (0)
#define MAX_CHANNEL                 (10)
#define VALID_CHANNEL_MASK          (0x000007FFUL)
#endif

#if (RF_BAND == BAND_2400)

/*
 * 4 bits form one symbol since O-QPSK is used
 */
/** Symbols per octet */
#define SYMBOLS_PER_OCTET                   (2)
/** Number of symbols included in the preamble */
#define NO_SYMBOLS_PREAMBLE                 (8)
/** Number of symbols included in the SFD field */
#define NO_SYMBOLS_SFD                      (2)

#elif (RF_BAND == BAND_900)

/*
 * Depending on the channel page either
 * 1 bit forms one symbol (BPSK in channel page 0) or
 * 4 bit form one symbol (O-QPSK in channel page 2).
 */

/**
 * Symbols per octet
 */
#define SYMBOLS_PER_OCTET                   (tal_pib.CurrentPage == 0 ? 8 : 2)

/**
 * Number of symbols included in the preamble
 */
#define NO_SYMBOLS_PREAMBLE                 (tal_pib.CurrentPage == 0 ? 32 : 8)

/**
 * Number of symbols included in the SFD field
 */
#define NO_SYMBOLS_SFD                      (tal_pib.CurrentPage == 0 ? 8 : 2)

#elif (RF_BAND == BAND_MULTIPLE)
#   ifdef ACTIVE_TRX
#       define SYMBOLS_PER_OCTET \
	((ACTIVE_TRX == RF24) ? 8 : \
	(tal_pib[ACTIVE_TRX].CurrentPage == 0 ? 8 : 2))
#       define NO_SYMBOLS_PREAMBLE \
	((ACTIVE_TRX == RF24) ? 32 : \
	(tal_pib[ACTIVE_TRX].CurrentPage == 0 ? 32 : 8))
#       define NO_SYMBOLS_SFD \
	((ACTIVE_TRX == RF24) ? 8 : \
	(tal_pib[ACTIVE_TRX].CurrentPage == 0 ? 8 : 2))
#   endif   /* #ifdef ACTIVE_TRX */

#else   /* "MAC-2003" */

/**
 * 1 bit forms one symbol since BPSK is used
 */
/* Symbols per octet */
#define SYMBOLS_PER_OCTET                   (8)
/* Number of symbols included in the preamble */
#define NO_SYMBOLS_PREAMBLE                 (32)
/* Number of symbols included in the SFD field */
#define NO_SYMBOLS_SFD                      (8)

#error "Unsupported RF band"
#endif

/**
 * Number of symbols forming the synchronization header (SHR) for the current
 * PHY.
 * This value is the base for the PHY PIB attribute phySHRDuration.
 */
#define NO_OF_SYMBOLS_PREAMBLE_SFD          (NO_SYMBOLS_PREAMBLE + \
	NO_SYMBOLS_SFD)

/**
 * Maximum number of symbols in a frame for the current PHY.
 * This value is the base for the PHY PIB attribute phyMaxFrameDuration.
 */
#define MAX_FRAME_DURATION \
	(NO_OF_SYMBOLS_PREAMBLE_SFD + \
	(aMaxPHYPacketSize + 1) * SYMBOLS_PER_OCTET)

/**
 * The maximum time in symbols for a 32 bit timer
 */
#define MAX_SYMBOL_TIME                     (0x0FFFFFFF)

/**
 * Symbol mask for ignoring most significant nibble
 */
#define SYMBOL_MASK                         (0x0FFFFFFF)

/* Custom attribute used by TAL */

/**
 * Attribute id of mac_i_pan_coordinator PIB
 */
#define mac_i_pan_coordinator               (0x0B)

#ifdef MULTI_TRX_SUPPORT
#   define TAL_CONVERT_SYMBOLS_TO_US(trx_id, symbols)   ((uint32_t)(symbols) * \
	tal_get_symbol_duration_us(trx_id))
#else

/**
 * Conversion of symbols to microseconds
 */
#   if (RF_BAND == BAND_2400)
#       define TAL_CONVERT_SYMBOLS_TO_US(symbols)      ((uint32_t)(symbols) << \
	4)
#   else   /* (RF_BAND == BAND_900) */
#       define TAL_CONVERT_SYMBOLS_TO_US(symbols) \
	(tal_pib.CurrentPage == 0 ? \
	(tal_pib.CurrentChannel == \
	0 ? ((uint32_t)(symbols) * 50) : ((uint32_t)(symbols) *	\
	25)) : \
	(tal_pib.CurrentChannel == \
	0 ? ((uint32_t)(symbols) * 40) : ((uint32_t)(symbols) << \
	4)) \
	)
#   endif  /* #if (RF_BAND == BAND_2400) */
#endif  /* #ifndef MULTI_TRX_SUPPORT */

#ifndef MULTI_TRX_SUPPORT

/**
 * Conversion of microseconds to symbols
 */
#   if (RF_BAND == BAND_2400)
#       define TAL_CONVERT_US_TO_SYMBOLS(time)         ((time) >> 4)
#   else   /* (RF_BAND == BAND_900) */
#       define TAL_CONVERT_US_TO_SYMBOLS(time) \
	(tal_pib.CurrentPage == 0 ? \
	(tal_pib.CurrentChannel == 0 ? ((time) / 50) : ((time) / 25)) :	\
	(tal_pib.CurrentChannel == 0 ? ((time) / 40) : ((time) >> 4)) \
	)
#   endif  /* #if (RF_BAND == BAND_2400) */
#endif  /* #ifndef MULTI_TRX_SUPPORT */

/*
 * Beacon Interval formula: BI = aBaseSuperframeDuration 2^BO\f$0
 * where \f$0 <= BO <= 14. Note: Beacon interval calculated is in
 * symbols.
 */

/**
 * Beacon Interval time in symbols
 */
#define TAL_GET_BEACON_INTERVAL_TIME(BO) \
	((1UL * aBaseSuperframeDuration) << (BO))

/*
 * Superframe Duration formula: \f$BI = aBaseSuperframeDuration 2^SO\f$
 * where \f$0 <= SO <= BO\f$
 */

/**
 * Superframe Duration time in symbols
 */
#define TAL_GET_SUPERFRAME_DURATION_TIME(SO) \
	((1UL * aBaseSuperframeDuration) << (SO))

#if (TAL_TYPE == ATMEGARFA1)

/*
 * Confirmation of a register write access. This operation is needed for most
 * TRX registers to finish a register write access. It is only needed when TRX
 * is in one of the PLL states. See also datasheet, section "Register access".
 */
#define CONF_REG_WRITE()   do {	\
		trx_reg_write(RG_PART_NUM, PART_NUM); \
} \
	while (0)
#endif /* TAL_TYPE == ATMEGA128RFA1 */

/**
 * Get bit mask from sub register definition
 */
#define TAL_BIT_MASK(ADDR, MASK, POS)   MASK

/**
 * Get bit position from sub register definition
 */
#define TAL_BIT_POS(ADDR, MASK, POS)    POS

/* === PROTOTYPES ========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

//#ifdef MULTI_TRX_SUPPORT
//void PHY_ConfigTrxId(trx_id_t trx_id);
//#endif

#if (TAL_TYPE == AT86RF215)
PHY_Retval_t trx_reset(trx_id_t trx_id);
#endif   /**
          * @brief TAL task handling
          *
          * This function
          * - Checks and allocates the receive buffer.
          * - Processes the TAL incoming frame queue.
          * - Implements the TAL state machine.
          * @ingroup apiTalApi
          */
void tal_task(void);
void PHY_TaskHandler(void);

/**
 * @brief Initializes the TAL
 *
 * This function is called to initialize the TAL. The transceiver is
 * initialized, the TAL PIBs are set to their default values, and the TAL state
 * machine is set to TAL_IDLE state.
 *
 * @return PHY_SUCCESS  if the transceiver state is changed to PHY_TRX_OFF and the
 *                 current device part number and version number are correct;
 *         PHY_FAILURE otherwise
 * @ingroup apiTalApi
 */
//PHY_Retval_t tal_init(void);
PHY_Retval_t PHY_Init(void);

#ifndef MULTI_TRX_SUPPORT

/**
 * @brief Resets TAL state machine and sets the default PIB values if requested
 *
 * @param set_default_pib Defines whether PIB values need to be set
 *                        to its default values
 *
 * @return PHY_SUCCESS  if the transceiver state is changed to PHY_TRX_OFF
 *         PHY_FAILURE otherwise
 * @ingroup apiTalApi
 */

PHY_Retval_t PHY_Reset(bool setDefaultPib);

// #if (MAC_SCAN_ED_REQUEST_CONFIRM == 1) || defined(DOXYGEN)

/**
 * @brief Starts ED Scan
 *
 * This function starts an ED Scan for the scan duration specified by the
 * MAC layer.
 *
 * @param scan_duration Specifies the ED scan duration in symbols
 *
 * @return PHY_SUCCESS - ED scan duration timer started successfully
 *         PHY_BUSY - TAL is busy servicing the previous request from MAC
 *         PHY_TRX_ASLEEP - Transceiver is currently sleeping
 *         PHY_FAILURE otherwise
 * @ingroup apiTalApi
 */

PHY_Retval_t PHY_EdStart(uint8_t scanDuration);
/**
 * User call back function for finished ED Scan
 *
 * @param energy_level Measured energy level during ED Scan
 * @ingroup apiTalApi
 */

void PHY_EdEndCallback(uint8_t energyLevel);
// #endif /* (MAC_SCAN_ED_REQUEST_CONFIRM == 1) */

/**
 * @brief Gets a TAL PIB attribute
 *
 * This function is called to retrieve the transceiver information base
 * attributes.
 *
 * @param[in] attribute TAL infobase attribute ID
 * @param[out] value TAL infobase attribute value
 *
 * @return PHY_UNSUPPORTED_ATTRIBUTE if the TAL infobase attribute is not found
 *         PHY_SUCCESS otherwise
 * @ingroup apiTalApi
 */
PHY_Retval_t PHY_PibGet(uint8_t attribute, uint8_t *value);
/**
 * @brief Sets a TAL PIB attribute
 *
 * This function is called to set the transceiver information base
 * attributes.
 *
 * @param attribute TAL infobase attribute ID
 * @param value TAL infobase attribute value to be set
 *
 * @return
 *      - @ref PHY_UNSUPPORTED_ATTRIBUTE if the TAL info base attribute is not
 * found
 *      - @ref PHY_BUSY if the TAL is not in TAL_IDLE state. An exception is
 *         macBeaconTxTime which can be accepted by TAL even if TAL is not
 *         in TAL_IDLE state.
 *      - @ref PHY_SUCCESS if the attempt to set the PIB attribute was
 * successful
 *      - @ref PHY_TRX_ASLEEP if trx is in SLEEP mode and access to trx is
 * required
 * @ingroup apiTalApi
 */

PHY_Retval_t PHY_PibSet(uint8_t attribute, PibValue_t *value);
/**
 * @brief Switches receiver on or off
 *
 * This function switches the receiver on (PHY_STATE_RX_ON) or off (PHY_STATE_TRX_OFF).
 *
 * @param state New state of receiver
 *
 * @return
 *      - @ref PHY_BUSY if the TAL state machine cannot switch receiver on or
 * off,
 *      - @ref PHY_STATE_TRX_OFF if receiver has been switched off, or
 *      - @ref PHY_STATE_RX_ON otherwise.
 * @ingroup apiTalApi
 */

PHY_TrxStatus_t PHY_RxEnable(PHY_TrxState_t state);
/**
 * User call back function for frame reception
 *
 * @param rx_frame Pointer to received frame structure of type PHY_FrameInfo_t
 *                 or to received frame array
 * @ingroup apiTalApi
 */

void PHY_RxFrameCallback(PHY_FrameInfo_t *rxFrame);
#ifdef ENABLE_RTB

/**
 * User call back function for frame reception in case RTB is used
 *
 * @param rx_frame Pointer to received frame structure of type PHY_FrameInfo_t
 *                 or to received frame array
 * @ingroup apiRtbApi
 */

void PHY_RtbRxFrameCallback(PHY_FrameInfo_t *rxFrame);
#endif  /* ENABLE_RTB */

// #if (TAL_TYPE != AT86RF215)
// #if (defined BEACON_SUPPORT)

// /**
//  * @brief Beacon frame transmission
//  *
//  * @param tx_frame Pointer to the PHY_FrameInfo_t structure
//  * @ingroup apiTalApi
//  */
// void tal_tx_beacon(PHY_FrameInfo_t *tx_frame);
// void PHY_TxBeacon(PHY_FrameInfo_t *tx_frame);
// #endif /* ((MAC_START_REQUEST_CONFIRM == 1) && (defined BEACON_SUPPORT)) */
// #endif /* #if (TAL_TYPE != AT86RF215) */

/**
 * @brief Requests to TAL to transmit frame
 *
 * This function is called by the MAC to deliver a frame to the TAL
 * to be transmitted by the transceiver.
 *
 * @param tx_frame Pointer to the PHY_FrameInfo_t structure or
 *                 to frame array to be transmitted
 * @param csma_mode Indicates mode of csma-ca to be performed for this frame
 * @param perform_frame_retry Indicates whether to retries are to be performed
 * for
 *                            this frame
 *
 * @return PHY_SUCCESS  if the TAL has accepted the data from the MAC for frame
 *                 transmission
 *         PHY_BUSY if the TAL is busy servicing the previous MAC request
 * @ingroup apiTalApi
 */

PHY_Retval_t PHY_TxFrame(PHY_FrameInfo_t *txFrame, PHY_CSMAMode_t csmaMode,
		bool performFrameRetry);
/**
 * User call back function for frame transmission
 *
 * @param status Status of frame transmission attempt
 * @param frame Pointer to frame structure of type PHY_FrameInfo_t
 * @ingroup apiTalApi
 */

void PHY_TxDoneCallback(PHY_Retval_t status, PHY_FrameInfo_t *frame);

/**
 * @brief Sets the transceiver to sleep
 *
 * This function sets the transceiver to sleep state.
 *
 * @param mode Defines sleep mode of transceiver SLEEP or PHY_STATE_TRX_OFF)
 *
 * @return   PHY_BUSY - The transceiver is busy in TX or RX
 *           PHY_SUCCESS - The transceiver is put to sleep
 *           PHY_TRX_ASLEEP - The transceiver is already asleep
 *           PHY_INVALID_PARAMETER - The specified sleep mode is not supported
 * @ingroup apiTalApi
 */
PHY_Retval_t PHY_TrxSleep(PHY_SleepMode_t mode);
/**
 * @brief Wakes up the transceiver from sleep
 *
 * This function awakes the transceiver from sleep state.
 *
 * @return   PHY_TRX_AWAKE - The transceiver is already awake
 *           PHY_SUCCESS - The transceiver is woken up from sleep
 *           PHY_FAILURE - The transceiver did not wake-up from sleep
 * @ingroup apiTalApi
 */

PHY_Retval_t PHY_TrxWakeup(void);
#else
#include "tal_multi_trx.h"
#endif /* MULTI_TRX_SUPPORT */

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_ConfigTxPwr(bool type, int8_t pwrValue)

  Summary:
    Configures the TX Power in Transceiver

  Description:
    This function is used to configure the Transmit power of the transceiver
 
  Precondition:
    PHY_Init() should have been called before calling this function

  Parameters:
    type        -  PWR_REGISTER_VALUE or PWR_DBM_VALUE
    pwrValue    -  Index of the power register value (0-15) or Power value in dBm
                   If LPA is enabled - Pmax - +5.5 dBm to Pmin - (-14)dbm
                   If LPA&MPA enabled - Pmax - +12 dBm to Pmin - (-16)dbm     

  Returns:
    PHY_SUCCESS  - If pwrValue bit is configured correctly
    PHY_FAILURE  - Otherwise

  Example:
    <code>
    bool pwrType = PWR_REGISTER_VALUE;
    uint8_t pwrIndex = 0x00;
    bool configStatus = false;

    if (PHY_SUCCESS == PHY_ConfigTxPwr(pwrType, int8_t (pwrIndex)))
    {
        configStatus = true;
    }

    int8_t pwrDbm = -17;
    pwrType = PWR_DBM_VALUE;
    if (PHY_SUCCESS == PHY_ConfigTxPwr(pwrType, int8_t (pwrDbm)))
    {
        configStatus = true;
    }
 
    uint8_t pwrReg;
    PHY_GetTrxConfig(TX_PWR, &pwrReg);   
    </code>

  Remarks:
    None .
*/
PHY_Retval_t PHY_ConfigTxPwr(bool type, int8_t pwrValue);

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_ConfigRxSensitivity(uint8_t pdtLevel)

  Summary:
    Configures receiver sensitivity level. This is used to desensitize 
    the receiver

  Description:
    This function is used to reduce the sensitivity of the receiver. 
    The input pdtLevel(Power Detect Level) desensitize the receiver such that 
    frames with an RSSI level below the pdtLevel threshold level (if pdtLevel > 0) 
    are not received. For a pdtLevel > 0 value the threshold level can be 
    calculated according to the following formula: 
            PRF[dBm] > RSSIBASE_VAL[dBm] + 3[dB] x (pdtLevel - 1)
 
  Precondition:
    PHY_Init() should have been called before calling this function

  Parameters:
    pdtLevel    -   0 to 15 levels of rx sensitivity(RX_PDT_LEVEL)
                      

  Returns:
    PHY_SUCCESS -  If pdtLevel bits are configured correctly
    PHY_FAILURE -  otherwise

  Example:
    <code>
    uint8_t pdtLevel =  0x03;
 
    PHY_ConfigRxSensitivity(pdtLevel);
    PHY_GetTrxConfig(RX_SENS, &pdtLevel); 
     
    </code>

  Remarks:
    None 
*/
PHY_Retval_t PHY_ConfigRxSensitivity(uint8_t pdtLevel);

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_ConfigRxPromiscuousMode(bool promCtrl)

  Summary:
    Configures RX promiscuous mode

  Description:
    This function is used to enable the RX promiscuous mode. The TRX will receive
    all frames even with FCS failure, PHY layer will discard the CRC invalid packet 
    and TRX will not acknowledge even ack is requested by the received 
    packet(auto ack is disabled in this mode). 
 
  Precondition:
    PHY_Init() should have been called before calling this function

  Parameters:
    promCtrl  - true  -  To enable promiscuous mode
                false -  To disable promiscuous mode

  Returns:
    PHY_SUCCESS -  If promCtrl bits are configured correctly
    PHY_FAILURE -  otherwise

  Example:
    <code>
    bool promCtrl =  true;
 
    PHY_ConfigRxPromiscuousMode(promCtrl);
    
    PHY_GetTrxConfig(AACK_PROMSCS_MODE, &promCtrl); 
     
    </code>

  Remarks:
    None 
*/
PHY_Retval_t PHY_ConfigRxPromiscuousMode(bool promCtrl);

/*
 * \brief to configure the reduced power consumption mode
 *
 * \param rpc_mode_sel value to be written in the TRX_RPC bits
 *
 * \return PHY_SUCCESS if the register is written correctly
 *         PHY_FAILURE otherwise
 */
#if (defined SUPPORT_FSK) || (defined SUPPORT_OQPSK)
PHY_Retval_t PHY_ConfigRxRPCMode(uint8_t rxRPCEnable);
#endif
/**
 * \brief the automatic acknowledgment from Transceiver after packet reception
 *
 * \param enableAACK true  to enable the automatic 
 *                         acknowledgment after reception
 *
 * \return PHY_SUCCESS  if configured correctly
 *         PHY_FAILURE      otherwise
 */


PHY_Retval_t PHY_ConfigAutoAck(bool enableAACK);

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_GetTrxConfig(PHY_ConfigParam_t parameter, uint8_t *paramValue)

  Summary:
    To read a current setting of particular transceiver parameter

  Description:
    The function is used to read the current of particular parameter. 
    The following parameters can be read from TRX,
        ANT_DIVERSITY     
        ANT_SELECT        
        ANT_CTRL          
        AACK_PROMSCS_MODE 
        TX_PWR            
        RX_SENS           
        RX_RPC                
        RX_AUTO_ACK       
        RX_RESERVED_FRAME 
        FILTER_RESERVED_FRAME 
 
  Precondition:
    PHY_Init() should have been called before calling this function.
 
  Parameters:
    parameter   - Type of the parameter to be read
    paramValue  - Pointer to the current parameter value
 
  Returns:
    PHY_Retval_t - PHY_INVALID_PARAMETER If the parameter is invalid
                 - PHY_SUCCESS otherwise
  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    bool promCtrl = true;
 
    PHY_GetTrxConfig(AACK_PROMSCS_MODE, (uint8_t *)&promCtrl); 
    </code>

  Remarks:
    None 
*/
PHY_Retval_t PHY_GetTrxConfig(PHY_ConfigParam_t parameter, uint8_t *paramValue);

// *****************************************************************************
// *****************************************************************************
// Section: PHY Utility Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    PHY_TrxStatus_t PHY_GetTrxStatus(void)

  Summary:
    Returns the current status of the Transceiver

  Description:
    This function gets the status of the transceiver
 
  Precondition:
    PHY_Init() should have been called before calling this function

  Parameters:
    None     

  Returns:
    PHY_TRX_OFF - The transceiver is in PHY_TRX_OFF state
    PHY_RX_ON - The transceiver is in receive state
    PHY_TX_ON - The transceiver is in Transmit state
    PHY_BUSY_RX - The transceiver currently receiving the packet
    PHY_BUSY_TX - The transceiver is currently transmitting the packet
    PHY_TRX_SLEEP - The transceiver is in sleep state
    PHY_DEEP_SLEEP - The transceiver is in Deep sleep state

  Example:
    <code>
    PHY_TrxStatus_t trxStatus;
    trxStatus = PHY_GetTrxStatus();
     
    </code>

  Remarks:
    None .
*/
PHY_TrxStatus_t PHY_GetTrxStatus(void);

// *****************************************************************************
/*
  Function:
    int8_t PHY_GetRSSIBaseVal(void)

  Summary:
    Get RSSI base value of TRX

  Description:
    This function is called to get the base RSSI value for respective
    radios
 
  Precondition:
    PHY_Init() should have been called before calling this function

  Parameters:
    None 

  Returns:
    32-bit PHY SW version value

  Example:
    <code>
    int8_t trxBaseRSSI;
 
    trxBaseRSSI = PHY_GetRSSIBaseVal();
    
    </code>

  Remarks:
    None 
*/
int8_t PHY_GetRSSIBaseVal(void);

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_ConvertTxPwrRegValToDbm(uint8_t regValue, int8_t *dbmValue)

  Summary:
    To convert the Tx Power Register index value to dbm Value

  Description:
    This function is used to convert Tx Power Register index value to dbm Value
 
  Precondition:
    PHY_Init() should have been called before calling this function

  Parameters:
    regVaue  - Index of the Power register value (Range 0-15)
    dbmValue - Corresponding dbm value to the Pwr register value  

  Returns:
    PHY_SUCCESS -  If reg value can be converted into dBm value
    PHY_FAILURE -  If regVaue is holding the invalid value

  Example:
    <code>
    uint8_t pwrRegIndex = 0x04;
    int8_t pwrDbm;
 
    PHY_ConvertTxPwrRegValToDbm(pwrRegIndex, &pwrDbm);
    
    </code>

  Remarks:
    None 
*/
PHY_Retval_t PHY_ConvertTxPwrRegValToDbm(uint8_t regValue, int8_t *dbmValue);

/*
 * \brief to read a particular range of transceiver registers
 *
 * \param reg_addr address of the transceiver register to be written
 * \param value value to be written in the register
 *
 * \return PHY_SUCCESS if the register is written correctly
 *         PHY_INVALID_PARAMETER if the reg_addr is out of range
 */
PHY_Retval_t PHY_ReadGrpOfReg(uint16_t start_addr,uint16_t end_addr);
void EIC_interrupt_cb(uintptr_t context);
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TAL_H */
/* EOF */
