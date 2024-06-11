/**
 * @file tal_multitrx_interface.c
 *
 * @brief This file implements the TAL state machine and provides general
 *        functionality used by the TAL.
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
#include <stdlib.h>
#include "config/default/driver/IEEE_802154_PHY/pal/inc/pal.h"
//#include "return_val.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/ieee_const.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_pib.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_config.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/stack_config.h"
#include "config/default/driver/IEEE_802154_PHY/resources/buffer/inc/bmm.h"
#include "config/default/driver/IEEE_802154_PHY/resources/queue/inc/qmm.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_internal.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_multi_trx.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal_helper_2.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/trx_access_2.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tfa.h"
volatile trx_id_t trx_id = RF24;
/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === GLOBALS ============================================================= */
#ifdef MULTI_TRX_SUPPORT
static PHY_Retval_t set_fsk(trx_id_t trx_id);
static PHY_Retval_t set_ofdm(trx_id_t trx_id);
static PHY_Retval_t set_oqpsk(trx_id_t trx_id);
static PHY_Retval_t set_leg_oqpsk(trx_id_t trx_id);
static PHY_Retval_t set_mod(trx_id_t trx_id, modulation_t mod);
#endif

void PHY_ConfigTrxId(trx_id_t trxid)
{
    trx_id = trxid;
}


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
PHY_Retval_t PHY_Reset(bool setDefaultPib)
{
    return (tal_reset(trx_id, setDefaultPib));
}

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
PHY_Retval_t PHY_EdStart(uint8_t scanDuration)
{
    return (tal_ed_start(trx_id, scanDuration));
}
/**
 * User call back function for finished ED Scan
 *
 * @param energy_level Measured energy level during ED Scan
 * @ingroup apiTalApi
 */
void tal_ed_end_cb(trx_id_t trx_id, uint8_t energyLevel)
{
    (void)trx_id;
    PHY_EdEndCallback(energyLevel);
}
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

PHY_Retval_t PHY_PibGet(uint8_t attribute, uint8_t *value)
{
    return (tal_pib_get(trx_id, attribute, value));
}
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

PHY_Retval_t PHY_PibSet(uint8_t attribute, PibValue_t *value)
{
    return (tal_pib_set(trx_id, attribute, value));
}
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

PHY_TrxStatus_t PHY_RxEnable(PHY_TrxState_t state)
{
    PHY_TrxStatus_t retVal;
    retVal = tal_rx_enable(trx_id, state);
    return retVal;
}
/**
 * User call back function for frame reception
 *
 * @param rx_frame Pointer to received frame structure of type PHY_FrameInfo_t
 *                 or to received frame array
 * @ingroup apiTalApi
 */

void tal_rx_frame_cb(trx_id_t trx_id, PHY_FrameInfo_t *rx_frame)
{
    PHY_RxFrameCallback(rx_frame);
}

#ifdef ENABLE_RTB

/**
 * User call back function for frame reception in case RTB is used
 *
 * @param rx_frame Pointer to received frame structure of type PHY_FrameInfo_t
 *                 or to received frame array
 * @ingroup apiRtbApi
 */
void rtb_rx_frame_cb(PHY_FrameInfo_t *rx_frame)
{
    PHY_RtbRxFrameCallback(PHY_FrameInfo_t *rxFrame);
}

#endif  /* ENABLE_RTB */


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

PHY_Retval_t PHY_TxFrame(PHY_FrameInfo_t *txFrame, PHY_CSMAMode_t csmaMode,bool performFrameRetry)
{
    return (tal_tx_frame(trx_id, txFrame, csmaMode, performFrameRetry));
}
/**
 * User call back function for frame transmission
 *
 * @param status Status of frame transmission attempt
 * @param frame Pointer to frame structure of type PHY_FrameInfo_t
 * @ingroup apiTalApi
 */

void tal_tx_frame_done_cb(trx_id_t trx_id, PHY_Retval_t status,	PHY_FrameInfo_t *frame)
{
    PHY_TxDoneCallback(status, frame);
}

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

PHY_Retval_t PHY_TrxSleep(PHY_SleepMode_t mode)
{
    return (tal_trx_sleep(trx_id));
}
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

PHY_Retval_t PHY_TrxWakeup(void)
{
    return (tal_trx_wakeup(trx_id));
}

/**
 * @brief Perform a CCA
 *
 * This function performs a CCA request.
 *
 * @return PHY_Retval_t PHY_IDLE or PHY_BUSY
 *
 * @ingroup apiTfaApi
 */
PHY_Retval_t PHY_CCAPerform(void)
{
    return (tfa_cca_perform(trx_id));
}

/**
 * @brief Perform a single ED measurement
 *
 * @return ed_value Result of the measurement (transceiver's register value)
 *         If the build switch TRX_REG_RAW_VALUE is defined, the transceiver's
 *         register value is returned.
 *
 * @ingroup apiTfaApi
 */
//uint8_t PHY_EdSample(void)
//{
//    return (tfa_ed_sample(trx_id));
//}
/**
 * @brief Starts continuous transmission on current channel
 *
 * @param trx_id Identifier of the transceiver
 * @param tx_mode Transmission mode
 * @param random_content Use random content if true
 */
void PHY_StartContinuousTransmit(PHY_ContinuousTxMode_t txMode, bool randomContent)
{
    tfa_continuous_tx_start(trx_id, txMode);
}
/**
 * @brief Stops CW transmission
 */
void PHY_StopContinuousTransmit(void)
{
    tfa_continuous_tx_stop(trx_id);
}

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
PHY_Retval_t PHY_ConfigTxPwr(bool type, int8_t pwrValue)
{
    return (tal_set_tx_pwr(trx_id, type, pwrValue));
}

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
PHY_Retval_t PHY_ConfigRxSensitivity(uint8_t pdtLevel)
{
    return (tal_set_rx_sensitivity_level(trx_id, pdtLevel));
}

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
PHY_Retval_t PHY_ConfigRxPromiscuousMode(bool promCtrl)
{
    return (tal_rxaack_prom_mode_ctrl(trx_id, promCtrl));
}

/*
 * \brief to configure the reduced power consumption mode
 *
 * \param rpc_mode_sel value to be written in the TRX_RPC bits
 *
 * \return PHY_SUCCESS if the register is written correctly
 *         PHY_FAILURE otherwise
 */
#if (defined SUPPORT_FSK) || (defined SUPPORT_OQPSK)
PHY_Retval_t PHY_ConfigRxRPCMode(uint8_t rxRPCEnable)
{
    PibValue_t pibVal;
    pibVal.pib_value_bool = rxRPCEnable;
    return (tal_pib_set(trx_id, phyRPCEnabled, &pibVal));
}
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


PHY_Retval_t PHY_ConfigAutoAck(bool enableAACK)
{
    uint16_t reg_offset = RF_BASE_ADDR_OFFSET * trx_id;
	/* Enable AACK */
	trx_reg_write(reg_offset + RG_BBC0_AMCS, AMCS_AACK_MASK);
    if(trx_id == RF09)
    {

	     /*check the configuration */
	    if (trx_reg_bit_read(SR_BBC0_AMCS_AACK) == (uint8_t)enableAACK) 
        {
		    return PHY_SUCCESS;
	    } else {
		    return PHY_FAILURE;
	    }
    }
    else
    {
        /*check the configuration */
	    if (trx_reg_bit_read(SR_BBC1_AMCS_AACK) == (uint8_t)enableAACK) 
        {
		    return PHY_SUCCESS;
	    } else {
		    return PHY_FAILURE;
	    }

    }

}

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
PHY_Retval_t PHY_GetTrxConfig(PHY_ConfigParam_t parameter, uint8_t *paramValue)
{
    return (tal_get_curr_trx_config(trx_id, parameter, paramValue));
}

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
PHY_TrxStatus_t PHY_GetTrxStatus(void)
{
    return (tal_get_trx_status(trx_id));
}

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
int8_t PHY_GetRSSIBaseVal(void)
{
    //Deepthi
    return 0;
}

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
PHY_Retval_t PHY_ConvertTxPwrRegValToDbm(uint8_t regValue, int8_t *dbmValue)
{
    return (tal_convert_reg_value_to_dBm(regValue, dbmValue));
}

/*
 * \brief to read a particular range of transceiver registers
 *
 * \param reg_addr address of the transceiver register to be written
 * \param value value to be written in the register
 *
 * \return PHY_SUCCESS if the register is written correctly
 *         MAC_INVALID_PARAMETER if the reg_addr is out of range
 */
PHY_Retval_t PHY_ReadGrpOfReg(uint16_t start_addr,uint16_t end_addr)
{
//    SYS_CONSOLE_PRINT("\r\n PHY_ReadGrpOfReg");
   uint8_t reg_val[256] = {NULL};
   int16_t num_of_reg_to_read = 0;
   num_of_reg_to_read = ((end_addr - start_addr));
   if(tal_dump_registers(trx_id, start_addr, end_addr, (reg_val + 1)) == PHY_SUCCESS)
   {
       reg_val[0] = num_of_reg_to_read + 1;
   }
       SYS_CONSOLE_PRINT("\r\n reg values at address %x is:",start_addr);
       SYS_CONSOLE_PRINT("%d\n",reg_val[1]);
//       if(reg_val[1] != 0)
//       {
//           	uint8_t temp; //modified by Deepthi
//		temp = trx_reg_read( RG_RF09_IRQS);
//		temp = trx_reg_read( RG_BBC0_IRQS);
//		temp = trx_reg_read( RG_RF24_IRQS);
//		temp = trx_reg_read( RG_BBC1_IRQS);
//        SYS_CONSOLE_PRINT("\r\n irq status registers cleared\r\n");
//       }
       PAL_TimerDelay(100000);
}

uint32_t PHY_GetSWVersion(void)
{
    return PHY_VERSION_VALUE;
}

void PHY_SetMod(trx_id_t trxid)
{
#ifdef MULTI_TRX_SUPPORT
    /* Configure PHY for sub-1GHz and 2.4GHz */
    if(trxid == RF09)
    {
        set_mod(RF09, MOD_SUB1);
    }
    else
    {
//        set_mod(RF24, MOD_2_4G);   
    }
#endif
}
#ifdef MULTI_TRX_SUPPORT
/**
 * @brief Set modulation
 *
 * @param trx_id    Transceiver identifier
 * @param mod       Modulation
 *
 * @return          PHY_SUCCESS or FAILURE
 */
static PHY_Retval_t set_mod(trx_id_t trx_id, modulation_t mod)
{
    PHY_Retval_t status;

    switch (mod)
    {
        case FSK:
            status = set_fsk(trx_id);
            break;

        case OFDM:
            status = set_ofdm(trx_id);
            break;

        case OQPSK:
            status = set_oqpsk(trx_id);
            break;

        case LEG_OQPSK:
            status = set_leg_oqpsk(trx_id);
            break;

        default:
            status = PHY_FAILURE;
    }

    return status;
}
#endif /* #ifdef MULTI_TRX_SUPPORT */


#ifdef MULTI_TRX_SUPPORT
/**
 * @brief Set FSK modulation
 *
 * @param trx_id    Transceiver identifier
 *
 * @return          PHY_SUCCESS or FAILURE
 */
static PHY_Retval_t set_fsk(trx_id_t trx_id)
{
    phy_t phy;
    PHY_Retval_t status;
    int pwr;

    phy.modulation = FSK;
    phy.phy_mode.fsk.data_rate = FSK_DATA_RATE_50;
    phy.phy_mode.fsk.mod_idx = MOD_IDX_1_0;
    phy.phy_mode.fsk.mod_type = F4FSK;
    if (trx_id == RF09)
    {
        phy.freq_band = US_915;
        phy.ch_spacing = FSK_915_MOD1_CH_SPAC;
        phy.freq_f0 = FSK_915_MOD1_F0;
        pwr = 14;
    }
    else // RF24
    {
        phy.freq_band = WORLD_2450;
        phy.ch_spacing = FSK_2450_MOD1_CH_SPAC;
        phy.freq_f0 = FSK_2450_MOD1_F0;
        pwr = 12;
    }

    /* Set preamble length */
    uint16_t len = 8;
    status = tal_pib_set(trx_id, phyFSKPreambleLength, (PibValue_t *)&len);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set FEC */
    bool fec = false;
    status = tal_pib_set(trx_id, phyFSKFECEnabled, (PibValue_t *)&fec);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Data whitening */
    bool dw = true;
    status = tal_pib_set(trx_id, phyFSKScramblePSDU, (PibValue_t *)&dw);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* CRC */
    uint16_t crc_type = FCS_TYPE_4_OCTETS;
    status = tal_pib_set(trx_id, macFCSType, (PibValue_t *)&crc_type);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set modulation / PHY configuration */
    status = tal_pib_set(trx_id, phySetting, (PibValue_t *)&phy);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set channel */
    channel_t ch = 0;
    status = tal_pib_set(trx_id, phyCurrentChannel, (PibValue_t *)&ch);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set transmit power */
    status = tal_pib_set(trx_id, phyTransmitPower, (PibValue_t *)&pwr);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Enable RPC */
    bool rpc = true;
    status = tal_pib_set(trx_id, phyRPCEnabled, (PibValue_t *)&rpc);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    return PHY_SUCCESS;
}
#endif /* #ifdef MULTI_TRX_SUPPORT */


#ifdef MULTI_TRX_SUPPORT
/**
 * @brief Set OFDM modulation
 *
 * @param trx_id    Transceiver identifier
 *
 * @return          PHY_SUCCESS or FAILURE
 */
static PHY_Retval_t set_ofdm(trx_id_t trx_id)
{
    phy_t phy;
    PHY_Retval_t status;

    phy.modulation = OFDM;
    phy.phy_mode.ofdm.option = OFDM_OPT_1;
    if (trx_id == RF09)
    {
        phy.freq_band = US_915;
        phy.ch_spacing = OFDM_915_OPT1_CH_SPAC;
        phy.freq_f0 = OFDM_915_OPT1_F0;
    }
    else // RF24
    {
        phy.freq_band = WORLD_2450;
        phy.ch_spacing = OFDM_2450_OPT1_CH_SPAC;
        phy.freq_f0 = OFDM_2450_OPT1_F0;
    }

    /* Set interleaving */
    bool interl = false;
    status = tal_pib_set(trx_id, phyOFDMInterleaving, (PibValue_t *)&interl);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set data rate / MCS */
    ofdm_mcs_t mcs = MCS3;
    status = tal_pib_set(trx_id, phyOFDMMCS, (PibValue_t *)&mcs);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* CRC */
    uint16_t crc_type = FCS_TYPE_4_OCTETS;
    status = tal_pib_set(trx_id, macFCSType, (PibValue_t *)&crc_type);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set modulation / PHY configuration */
    status = tal_pib_set(trx_id, phySetting, (PibValue_t *)&phy);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set channel */
    channel_t ch = 0;
    status = tal_pib_set(trx_id, phyCurrentChannel, (PibValue_t *)&ch);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set transmit power */
    int pwr = 14;
    status = tal_pib_set(trx_id, phyTransmitPower, (PibValue_t *)&pwr);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    return PHY_SUCCESS;
}
#endif /* #ifdef MULTI_TRX_SUPPORT */


#ifdef MULTI_TRX_SUPPORT
/**
 * @brief Set MR-OQPSK modulation
 *
 * @param trx_id    Transceiver identifier
 *
 * @return          PHY_SUCCESS or FAILURE
 */
static PHY_Retval_t set_oqpsk(trx_id_t trx_id)
{
    phy_t phy;
    PHY_Retval_t status;

    phy.modulation = OQPSK;
    phy.phy_mode.oqpsk.chip_rate = CHIP_RATE_100;
    if (trx_id == RF09)
    {
        phy.freq_band = US_915;
        phy.ch_spacing = OQPSK_915_CH_SPAC;
        phy.freq_f0 = OQPSK_915_F0;
    }
    else
    {
        phy.freq_band = WORLD_2450;
        phy.ch_spacing = OQPSK_2450_CH_SPAC;
        phy.freq_f0 = OQPSK_2450_F0;
    }

    /* Set data rate / rate mode */
    oqpsk_rate_mode_t rate = OQPSK_RATE_MOD_0;
    status = tal_pib_set(trx_id, phyOQPSKRateMode, (PibValue_t *)&rate);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* CRC */
    uint16_t crc_type = FCS_TYPE_4_OCTETS;
    status = tal_pib_set(trx_id, macFCSType, (PibValue_t *)&crc_type);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set modulation / PHY configuration */
    status = tal_pib_set(trx_id, phySetting, (PibValue_t *)&phy);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set channel */
    channel_t ch = 0;
    status = tal_pib_set(trx_id, phyCurrentChannel, (PibValue_t *)&ch);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set transmit power */
    int pwr = 14;
    status = tal_pib_set(trx_id, phyTransmitPower, (PibValue_t *)&pwr);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Enable RPC */
    bool rpc = true;
    status = tal_pib_set(trx_id, phyRPCEnabled, (PibValue_t *)&rpc);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    return PHY_SUCCESS;
}
#endif /* #ifdef MULTI_TRX_SUPPORT */


#ifdef MULTI_TRX_SUPPORT
/**
 * @brief Set legacy OQPSK modulation
 *
 * @param trx_id    Transceiver identifier
 *
 * @return          PHY_SUCCESS or FAILURE
 */
static PHY_Retval_t set_leg_oqpsk(trx_id_t trx_id)
{
    phy_t phy;
    PHY_Retval_t status;
    channel_t ch;

    phy.modulation = LEG_OQPSK;
    if (trx_id == RF09)
    {
        phy.phy_mode.leg_oqpsk.chip_rate = CHIP_RATE_1000;
        phy.freq_band = US_915;
        phy.ch_spacing = LEG_915_CH_SPAC;
        phy.freq_f0 = LEG_915_F0 - LEG_915_CH_SPAC;
        ch = 1;
    }
    else // RF24
    {
        phy.phy_mode.leg_oqpsk.chip_rate = CHIP_RATE_2000;
        phy.freq_band = WORLD_2450;
        phy.ch_spacing = LEG_2450_CH_SPAC;
        phy.freq_f0 = LEG_2450_F0 - (11 * LEG_2450_CH_SPAC);
        ch = 11;
    }

    /* Set modulation / PHY configuration */
    status = tal_pib_set(trx_id, phySetting, (PibValue_t *)&phy);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set channel */
    status = tal_pib_set(trx_id, phyCurrentChannel, (PibValue_t *)&ch);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    /* Set transmit power */
    int pwr = 14;
    status = tal_pib_set(trx_id, phyTransmitPower, (PibValue_t *)&pwr);
    if (status != PHY_SUCCESS)
    {
        return status;
    }

    return PHY_SUCCESS;
}
#endif /* #ifdef MULTI_TRX_SUPPORT */
