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



#include "../../../pal/inc/pal.h"
#include "../../../phy/inc/phy.h"
#include "../../at86rf/inc/at86rf.h"
#include "../../../phy/inc/phy_config.h"
#include "../../at86rf/inc/phy_internal.h"
#include "../../../phy/inc/ieee_phy_const.h"

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* === GLOBALS ============================================================= */
extern const int8_t tx_pwr_table[16];

/* === PROTOTYPES ========================================================== */
extern uint8_t convert_phyTransmitPower_to_reg_value(
		uint8_t phyTransmitPower_value);

/* === IMPLEMENTATION ====================================================== */

/**
 * \brief Enable/Disable the external RF front end control
 *
 * \param pa_ext_sw_ctrl true  if external rf front end control has to be
 *                              enabled
 *
 * \return PHY_SUCCESS  if PA_EXT_EN bit is configured correctly
 *         PHY_FAILURE      otherwise
 */


PHY_Retval_t PHY_ConfigTxPwr(bool type, int8_t pwrValue)
{
	uint64_t temp_var;
    int8_t tx_pwr_dbm = 0;	
    	/* modify the register for tx_pwr and set the tal_pib accordingly */
	if (true == type) {
		if (PHY_SUCCESS ==
				PHY_ConvertTxPwrRegValToDbm((uint8_t)pwrValue,
				&tx_pwr_dbm)) {
			temp_var = CONV_DBM_TO_phyTransmitPower(tx_pwr_dbm);
			(void)PHY_PibSet(phyTransmitPower, (PibValue_t *)&temp_var);

			/* To make sure that TX_PWR register is updated with the
			 * value whatever user povided.Otherwise lowest dBm
			 * power
			 * (highest reg value will be taken)
			 */
			trx_reg_bit_write(SR_TX_PWR, (uint8_t)pwrValue);

			return PHY_SUCCESS;
		} else {
			/* return invalid parameter if out of range */
			return PHY_INVALID_PARAMETER;
		}
	} else {
		temp_var = CONV_DBM_TO_phyTransmitPower(pwrValue);
		(void)PHY_PibSet(phyTransmitPower, (PibValue_t *)&temp_var);
	}

	

	uint8_t reg_value = convert_phyTransmitPower_to_reg_value(
			tal_pib.TransmitPower);
	/* check the value written in the transceiver register */
	uint8_t temp = trx_reg_bit_read(SR_TX_PWR);
    
        
	if (temp == reg_value) {
        phy_info.phy_config_param.txPwr = pwrValue;
		return PHY_SUCCESS;
	} else {
		return PHY_FAILURE;
	}
}



/*
 * \brief Configures antenna diversity and selects antenna
 *
 * \param div_ctrl  true/false to enable/disable antenna diversity
 * \param ant_ctrl  0 or 3 when antenna diversity is enabled
 *                  1 or 2 to select antenna 1 or antenna 2
 * \return The value set in the TX_PWR bits
 */

PHY_Retval_t  PHY_ConfigAntennaDiversity(bool divCtrl, uint8_t antCtrl)
{
	PHY_Retval_t return_var = PHY_FAILURE;
	if (true == divCtrl) {
		/* do the configurations if diversity has to be enabled */
		trx_reg_bit_write(SR_ANT_CTRL, ANT_CTRL_0);
		trx_reg_bit_write(SR_ANT_DIV_EN, ANT_DIV_ENABLE);
		trx_reg_bit_write(SR_PDT_THRES, THRES_ANT_DIV_ENABLE);
		trx_reg_bit_write(SR_ANT_EXT_SW_EN, ANT_EXT_SW_ENABLE);

		/* check the values written in transceiver registers */
		if ((trx_reg_bit_read(SR_ANT_CTRL) == ANT_CTRL_0) &&
				(trx_reg_bit_read(SR_ANT_DIV_EN) ==
				ANT_DIV_ENABLE) && \
				(trx_reg_bit_read(SR_ANT_EXT_SW_EN) ==
				ANT_EXT_SW_ENABLE)) {
            

			if ((trx_reg_bit_read(SR_PDT_THRES) ==
					THRES_ANT_DIV_ENABLE)){
                
                phy_info.phy_config_param.antDiversity = divCtrl;
                phy_info.phy_config_param.antCtrl = antCtrl;

			return_var = PHY_SUCCESS;
            }
		} else {
			return_var = PHY_FAILURE;
		}
	} else {
		/* do the configurations if diversity has to be disabled */
		trx_reg_bit_write(SR_ANT_DIV_EN, ANT_DIV_DISABLE);
		trx_reg_bit_write(SR_PDT_THRES, THRES_ANT_DIV_DISABLE);
		trx_reg_bit_write(SR_ANT_EXT_SW_EN, ANT_EXT_SW_ENABLE);
		if (antCtrl == ANT_CTRL_1) {
			/* Enable A1/X2 */
			trx_reg_bit_write(SR_ANT_CTRL, ANT_CTRL_1);
            phy_info.phy_config_param.antSelect = 0;
		} else if (antCtrl == ANT_CTRL_2) {
			/* Enable A2/X3 */
			trx_reg_bit_write(SR_ANT_CTRL, ANT_CTRL_2);
            phy_info.phy_config_param.antSelect = 1;
		} else if (antCtrl == ANT_CTRL_0 || antCtrl == ANT_CTRL_3) {
			trx_reg_bit_write(SR_ANT_CTRL, ANT_CTRL_0);
			antCtrl = 0;
		} else {
			return_var = PHY_INVALID_PARAMETER;
		}
		/* check the values written in transceiver registers */
		if ((trx_reg_bit_read(SR_ANT_CTRL) == antCtrl) &&
				(trx_reg_bit_read(SR_ANT_DIV_EN) ==
				ANT_DIV_DISABLE) && \
				(trx_reg_bit_read(SR_ANT_EXT_SW_EN) ==
				ANT_EXT_SW_DISABLE)) {

			if ((trx_reg_bit_read(SR_PDT_THRES) ==
					THRES_ANT_DIV_ENABLE))
            {
                phy_info.phy_config_param.antDiversity = divCtrl;
                phy_info.phy_config_param.antCtrl = antCtrl;
                
                return_var = PHY_SUCCESS;
            }
		} else {
			return_var = PHY_FAILURE;
		}
	}
	return return_var;
}



/*
 * \brief Configures the frequency to be set in transceiver
 *
 * \param frequency  frequency value to be set
 * \return PHY_SUCCESS if frequency is configured correctly
 *                 MAC_INVALID_PARAMETER if out of range or incorrect values are
 * given
 *                 PHY_FAILURE if frequency registers are not configured properly
 */


PHY_Retval_t tal_set_frequency(float frequency)
{
	double epsilon = 0.000000001;
	double dummy = 0.0;
	uint8_t cc_number = 0;
	uint8_t cc_band = 0;
	tal_trx_status_t previous_trx_status = TRX_OFF;
	/* frequency has to selected by CHANNEL register bits in PHY_CC_CCA*/
	if ((abs((double)frequency - dummy)) < epsilon) {
		cc_band = 0;
		cc_number = 0;
	}
	/* return invalid parameter if input frequency is out of range */
	else if ((frequency < MIN_ISM_FREQUENCY_MHZ) ||
			(frequency > MAX_ISM_FREQUENCY_MHZ)) {
		return PHY_INVALID_PARAMETER;
	}
	/* Choose CC_BAND & CC_NUMBER reg values for the input frequency */
	else if (frequency < MID_ISM_FREQUENCY_MHZ) {
		/* required frequency maps to band 8 */
		cc_band = CC_BAND_8;
		cc_number = (uint8_t)((frequency - BASE_ISM_FREQUENCY_MHZ) * 2.0f);
	} else {
		/* required frequency maps to band 9 */
		cc_band = CC_BAND_9;
		cc_number = (uint8_t)((frequency -  MID_ISM_FREQUENCY_MHZ) * 2.0f);
	}
    
	if (phy_info.tal_trx_status != TRX_OFF) {
		previous_trx_status = RX_AACK_ON; /* any other than TRX_OFF
		                                   * state */
		do {
			/* set TRX_OFF until it could be set;
			 * trx might be busy */
		} while (set_trx_state(CMD_TRX_OFF) != TRX_OFF);
	}

	trx_reg_bit_write(SR_CC_BAND, cc_band);
	trx_reg_write(RG_CC_CTRL_0, cc_number);

	/* Re-store previous trx state */
	if (previous_trx_status != TRX_OFF) {
		/* Set to default state */
		(void)set_trx_state(CMD_RX_AACK_ON);
	}

	/* check the values written in transceiver registers */
	if (trx_reg_bit_read(SR_CC_BAND) != cc_band || \
			trx_reg_read(RG_CC_CTRL_0) != cc_number) {
		return PHY_FAILURE;
	} else {
		return PHY_SUCCESS;
	}
}



/**
 * \brief to set the frequency based on CC_BAND and CC_NUMBER Registers
 *
 * \param cc_band band to be selected in cc_band register bits
 * \param cc_number offset frequency to be selected in cc_number register bits
 * \return PHY_SUCCESS if frequency is configured correctly
 *                 MAC_INVALID_PARAMETER if out of range or incorrect values are
 * given
 *                 PHY_FAILURE if frequency registers are not configured properly
 */

PHY_Retval_t tal_set_frequency_regs(uint8_t cc_band, uint8_t cc_number)
{
	tal_trx_status_t previous_trx_status = TRX_OFF;


	/* check cc band and cc number fit in the range*/
	if ((cc_band != CC_BAND_8) && (cc_band != CC_BAND_9)) {
		return PHY_INVALID_PARAMETER;
	} else if ((cc_band == CC_BAND_8) &&
			(cc_number <  MIN_CC_BAND_8_OFFSET)) {
		return PHY_INVALID_PARAMETER;
	} else if ((cc_band == CC_BAND_9) &&
			(cc_number > MIN_CC_BAND_9_OFFSET)) {
		return PHY_INVALID_PARAMETER;
	} else{
        /*DO NOTHING*/
    }

	/*
	 * Set trx to trx_off to avoid interruption in ongoing
	 * transaction
	 */
	if (phy_info.tal_trx_status != TRX_OFF) {
		previous_trx_status = RX_AACK_ON; /* any other than TRX_OFF
		                                   * state */
		do {
			/* set TRX_OFF until it could be set;
			 * trx might be busy */
		} while (set_trx_state(CMD_TRX_OFF) != TRX_OFF);
	}

	trx_reg_bit_write(SR_CC_BAND, cc_band);
	trx_reg_write(RG_CC_CTRL_0, cc_number);

	/* Re-store previous trx state */
	if (previous_trx_status != TRX_OFF) {
		/* Set to default state */
		(void)set_trx_state(CMD_RX_AACK_ON);
	}

	/* check the values written in transceiver registers */
	if (trx_reg_bit_read(SR_CC_BAND) != cc_band || \
			trx_reg_read(RG_CC_CTRL_0) != cc_number) {
		return PHY_FAILURE;
	} else {
		return PHY_SUCCESS;
	}
}



/*
 * \brief Calculate the frequency based on CC_BAND and CC_NUMBER Registers
 *
 * \param CC_BAND and CC_NUMBER register values to calculate the frequency
 * \param *freq pointer where the calculated frequency value should be stored
 *
 * \return  PHY_SUCCESS if frequency is configured correctly
 *          MAC_INVALID_PARAMETER if out of range or incorrect values are given
 *          PHY_FAILURE if frequency registers are not configured properly
 */

PHY_Retval_t tal_calculate_frequency(uint8_t cc_band, uint8_t cc_number,
		float *freq)
{

	/* check cc band and cc number fit in the range*/
	if ((cc_band != CC_BAND_8) && (cc_band != CC_BAND_9)) {
		return PHY_INVALID_PARAMETER;
	} else if ((cc_band == CC_BAND_8) &&
			(cc_number <  MIN_CC_BAND_8_OFFSET)) {
		return PHY_INVALID_PARAMETER;
	} else if ((cc_band == CC_BAND_9) &&
			(cc_number > MIN_CC_BAND_9_OFFSET)) {
		return PHY_INVALID_PARAMETER;
	} else{
        /*DO NOTHING*/
    }

	*freq = (float)((cc_band == CC_BAND_8) ?
			(BASE_ISM_FREQUENCY_MHZ + 0.5 * ((float)cc_number)) :
			(MID_ISM_FREQUENCY_MHZ + 0.5 * ((float)cc_number)));

	return PHY_SUCCESS;
}



/*
 * \brief Configures receiver sensitivity level
 *
 * \param pdt_level  0 to 15 levels of rx sensitivity
 * \param PHY_SUCCESS if sensitivity level is configured correctly
 *        MAC_INVALID_PARAMETER pdt_level is out of range
 *        PHY_FAILURE otherwise
 */

PHY_Retval_t PHY_ConfigRxSensitivity(uint8_t pdtLevel)
{
	uint8_t temp;
	/* return invalid parameter if sensitivity level is out of range*/
	if (pdtLevel > MAX_PDT_LEVEL) {
		return PHY_INVALID_PARAMETER;
	}
	/* configure sensitivity level*/
	trx_reg_bit_write(SR_RX_PDT_LEVEL, pdtLevel);

	temp = trx_reg_bit_read(SR_RX_PDT_LEVEL);
	if (temp == pdtLevel) {
        phy_info.phy_config_param.rxSens = pdtLevel;
		return PHY_SUCCESS;
	} else {
		return PHY_FAILURE;
	}
}



/*
 * \brief Configures promiscous mode in rx_aack_on mode
 *
 * \param prom_ctrl  true/false to enable/disable prom mode
 *
 * \param PHY_SUCCESS if rxaack_prom_mode is configured correctly
 *        PHY_FAILURE otherwise
 */

PHY_Retval_t PHY_ConfigRxPromiscuousMode(bool promCtrl)
{
	bool temp;
	/* configure promiscuous mode */
	trx_reg_bit_write(SR_AACK_PROM_MODE, (uint8_t)promCtrl);
	temp = (bool)trx_reg_bit_read(SR_AACK_PROM_MODE);
	if (temp == promCtrl) {
        phy_info.phy_config_param.aackPromMode = promCtrl;
		return PHY_SUCCESS;
	} else {
		return PHY_FAILURE;
	}
}



/*
 * \brief to get the current status of the transceiver
 *
 * \return status of the transceiver
 */
tal_trx_status_t tal_get_trx_status(void)
{
    if (phy_info.tal_trx_status == TRX_SLEEP) 
    {
        return (TRX_SLEEP);
    }
	tal_trx_status_t trx_status;
	/* Read the status from trx_status bits */
	trx_status = (tal_trx_status_t)trx_reg_bit_read(SR_TRX_STATUS);
	return trx_status;
}



/*
 * \brief to read a current setting particular transceiver parameter
 * \param parameter type of the parameter to be read
 * \param *param_value pointer to the location where the current parameter value
 * need to be
 *              stored
 * \return MAC_INVALID_PARAMETER if the parameter is invalid
 *         PHY_SUCCESS otherwise
 */

PHY_Retval_t PHY_GetTrxConfig(PHY_ConfigParam_t parameter, uint8_t *paramValue)
{
	switch (parameter) {

	case ANT_DIVERSITY:
		*paramValue = trx_reg_bit_read(SR_ANT_DIV_EN);
		break;

	case ANT_SELECT_:
		*paramValue = trx_reg_bit_read(SR_ANT_SEL);
		break;

	case ANT_CTRL_:
		*paramValue = trx_reg_bit_read(SR_ANT_CTRL);
		break;

	case AACK_PROMSCS_MODE:
		*paramValue = trx_reg_bit_read(SR_AACK_PROM_MODE);
		break;
      
	case CC_BAND:
		*paramValue = trx_reg_bit_read(SR_CC_BAND);
		break;

	case CC_NUMBER:
		*paramValue = trx_reg_read(RG_CC_CTRL_0);
		break;
      
	case TX_PWR:
		*paramValue = trx_reg_bit_read(SR_TX_PWR);
		break;
        
    case RX_SENS: 
        *paramValue = trx_reg_bit_read(SR_RX_PDT_LEVEL); 
        break;
           
    case RX_RPC:
        *paramValue = trx_reg_bit_read(SR_RX_RPC_EN); 
         break;
        
    case RX_AUTO_ACK:
        *paramValue = trx_reg_bit_read(SR_AACK_DIS_ACK); 
        break;
        
    case RX_RESERVED_FRAME:
        *paramValue = trx_reg_bit_read(SR_AACK_UPLD_RES_FT); 
        break;
        
    case FILTER_RESERVED_FRAME:
        *paramValue = trx_reg_bit_read(SR_AACK_FLTR_RES_FT); 
        break;

	default:
		return PHY_INVALID_PARAMETER;
		break;
	}
	return PHY_SUCCESS;
}


/*
 * \brief to configure the reduced power consumption mode
 *
 * \param rpc_mode_sel value to be written in the TRX_RPC bits
 *
 * \return PHY_SUCCESS if the register is written correctly
 *         PHY_FAILURE otherwise
 */

PHY_Retval_t PHY_ConfigRxRPCMode(uint8_t rxRPCEnable)
{
    bool rxRPC = false;
    if(rxRPCEnable != 0U)
    {rxRPC = true;}
        
	/*configure the rpc modes*/
	trx_reg_bit_write(SR_RX_RPC_EN, (uint8_t)rxRPC);
	/*check whether the configuration is done properly*/
	if (trx_reg_bit_read(SR_RX_RPC_EN) == (uint8_t)rxRPC) {
        phy_info.phy_config_param.rxRPC = rxRPC;
		return PHY_SUCCESS;
	} else {
		return PHY_FAILURE;
	}
    
}

/*
 * \brief Converts a register value to a dBm value
 *
 * \param reg_value TransmitPower register value,
 *
 * \param *dbm_value pointer to dbm_value
 *
 * \return PHY_SUCCESS or PHY_FAILURE based on conversion is done or not
 */

PHY_Retval_t PHY_ConvertTxPwrRegValToDbm(uint8_t regValue, int8_t *dbmValue)
{
	if (regValue < sizeof(tx_pwr_table)) {
		*dbmValue = (int8_t)*(&tx_pwr_table[regValue]);
		return PHY_SUCCESS;
	} else {
		return PHY_FAILURE;
	}
}



/*
 * \brief This function is called to get the base RSSI value for respective
 * radios
 *
 * \return value of the base RSSI value
 */
int8_t PHY_GetRSSIBaseVal(void)
{
	return (RSSI_BASE_VAL_DBM);
}

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
	trx_reg_bit_write(SR_AACK_DIS_ACK, (uint8_t)enableAACK); 
	/*check the configuration */
	if (trx_reg_bit_read(SR_AACK_DIS_ACK) == (uint8_t)enableAACK) {
        phy_info.phy_config_param.rxAutoAck = enableAACK;
		return PHY_SUCCESS;
	} else {
		return PHY_FAILURE;
	}
}


PHY_Retval_t PHY_ConfigReservedFrameFiltering(bool recReservedFrame, bool bypassFrameFilter )
{
	trx_reg_bit_write(SR_AACK_UPLD_RES_FT, (uint8_t)recReservedFrame); 
    
	trx_reg_bit_write(SR_AACK_FLTR_RES_FT, (uint8_t)bypassFrameFilter); 
    
    /*check the configuration */
	if ((trx_reg_bit_read(SR_AACK_UPLD_RES_FT) == (uint8_t)recReservedFrame) &&
         (trx_reg_bit_read(SR_AACK_FLTR_RES_FT) == (uint8_t)bypassFrameFilter)) 
    {
        phy_info.phy_config_param.reservedFrameFiltering = bypassFrameFilter;
        phy_info.phy_config_param.rxReservedFrame = recReservedFrame; 
		return PHY_SUCCESS;
	} else {
		return PHY_FAILURE;
	}
    
    
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

PHY_Retval_t tal_dump_registers(uint16_t start_addr, uint16_t end_addr,
		uint8_t *value)
{
	uint16_t addr; 
	int8_t length;

	/*check start and end address, return invalid parameter if out of range
	**/

	if (start_addr > 0x3FU || end_addr > 0x3FU) { 
		return PHY_INVALID_PARAMETER;
	}

    int16_t temp_length = (end_addr - start_addr);    
	length = (int8_t)(temp_length); 
	if (length < 0) {
		/* return invalid parameter if start and end addresses are not
		 * in order*/
		return PHY_INVALID_PARAMETER;
	} else {
		/* Read and store the values in input address*/
		for (addr = start_addr; addr <= end_addr; addr++) {
			*value = trx_reg_read((uint8_t)addr); 
			value++;
		}
		return PHY_SUCCESS;
	}
}


/* EOF */
