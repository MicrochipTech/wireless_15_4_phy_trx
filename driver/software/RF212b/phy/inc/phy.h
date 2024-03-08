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



#ifndef PHY_H
#define PHY_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "phy_config.h"
#include "../../resources/buffer/inc/bmm.h"
#include "../../resources/queue/inc/qmm.h"
#include "ieee_phy_const.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END
        
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
        
// *****************************************************************************
/* PIB value types

  Summary:
    Union of different PHY Pib value types

  Description:
    PibValue_t is used as the data type to set/get the different PHY Information 
    Base value 

  Remarks:
    None
*/

typedef union {
	/** PIB Attribute Bool */
	bool pib_value_bool;
	/** PIB Attribute 8-bit */
	uint8_t pib_value_8bit;
	/** PIB Attribute 16-bit */
	uint16_t pib_value_16bit;
	/** PIB Attribute 32-bit */
	uint32_t pib_value_32bit;
	/** PIB Attribute 64-bit */
	uint64_t pib_value_64bit;
} PibValue_t;

// *****************************************************************************
/* PHY Configuration Parameters
 
   Summary:
    PHY Configuration parameters supported by transceiver
 
   Description:
    Following are the list of configuration parameters which can be read from the 
    transceiver
 
   Remarks:
    None
 */
 
typedef enum param_tag {
    /** Antenna Diversity */
	ANT_DIVERSITY     = 0x00,
    /** Antenna Configured - ANTENNA_1/ANTENNA_2*/
	ANT_SELECT_        = 0x01,
    /** Antenna Control */
	ANT_CTRL_          = 0x02,
    /** Promiscuous Mode*/
	AACK_PROMSCS_MODE = 0x03,
	CC_BAND           = 0x04,
	CC_NUMBER         = 0x05,
    /** Tx Power Configured*/
	TX_PWR            = 0x06,
    /** Rx Sensitivity*/
    RX_SENS           = 0x07,
    /** Automatic acknowledgement*/
    RX_AUTO_ACK       = 0x09,
    /** Reserved frame reception*/
    RX_RESERVED_FRAME = 0x0A,
    /** Filter reserved frame*/
    FILTER_RESERVED_FRAME = 0x0B,
}PHY_ConfigParam_t;

// *****************************************************************************
/* PHY Frame information structure
 
   Summary:
    PHY_FrameInfo_t holds the data to be transmitted or the data being received 
    by the transceiver. 
 
   Description:
    None
 
   Remarks:
    None 
 */
 
typedef struct frame_info_tag
{
	/** Pointer to buffer header of frame */
	buffer_t *buffer_header;
	/** Pointer to MPDU */
	uint8_t *mpdu;
} PHY_FrameInfo_t;

// *****************************************************************************
/* PHY Sleep Modes
 
   Summary:
    Sleep Modes supported by transceiver
 
   Description:
    List of sleep modes supported by the transceiver. If, TRX is set to SLEEP_MODE_1,
    the TRX register contents are retained. If, TRX is set to DEEP_SLEEP_Mode 
    state the register contents are cleared
 
   Remarks:
    None 
 */
 
typedef enum sleep_mode_tag {
	SLEEP_MODE_1,
} PHY_SleepMode_t;

// *****************************************************************************
/* PHY CSMA Modes
 
   Summary:
   List of carrier sense multiple access with collision avoidance 
   supported by PHY Layer
 
   Description:
    When Transmit function is called with PHYCSMAMode of 
    NO_CSMA_NO_IFS    - Immediate Tx and SIFS(Short InterFrameSpacing) between 
                       subsequent frames
    NO_CSMA_WITH_IFS  - Immediate Tx and LIFS (Long InterFrameSpacing) between
                       subsequent frames
    CSMA_UNSLOTTED    - Hardware CSMA will be performed before packet transmission 
                        with number of retries configured 
    CSMA_SLOTTED      - Hardware CSMA will be performed - Used with Beacon 
                        Enabled network - Currently not supported by PHY 
   Remarks:
    None 
 */
 
typedef enum csma_mode_tag {
	NO_CSMA_NO_IFS,
	NO_CSMA_WITH_IFS,
	CSMA_UNSLOTTED,
	CSMA_SLOTTED
} PHY_CSMAMode_t;

// *****************************************************************************
/** Transceiver commands */
typedef enum tfa_pib_tag {
	TFA_PIB_RX_SENS         = 0
} PHY_tfa_pib_t;

/* PHY Continuous Transmission test Modes
 
   Summary:
    List of Continuous Transmission Test Modes supported by transceiver
 
   Description:
    CW_MODE - Continuous Wave mode to transmit the signal at Fc +&- 0.5MHz frequency 
    PRBS_MODE  - PRBS mode to Pseudorandom Binary Sequence frame continuously
 
   Remarks:
    None 
 */
 
typedef enum continuous_tx_mode_tag {
	/* Continuous Wave mode to transmit 
	 * the signal at Fc +&- 0.5MHz frequency */
	CW_MODE = 0,
	/* PRBS mode to Pseudorandom Binary Sequence frame continuously */
	PRBS_MODE = 1,
	CW_MODE_2 = 2,
	CW_ALL_ZEROS_MODE = 3
} PHY_ContinuousTxMode_t;


// *****************************************************************************
/* PHY Return Values
 
   Summary:
    List of return status for the PHY functions
 
   Description:
    None
   Remarks:
    None 
 */
 
typedef enum phy_return_value_tag {
	/* General Success condition*/
    PHY_SUCCESS                 = 0x00, 
	/* Transceiver is currently sleeping */
    PHY_TRX_ASLEEP              = 0x81,
	/* Transceiver is currently awake */
	PHY_TRX_AWAKE               = 0x82, 
	/* General failure condition */
	PHY_FAILURE                 = 0x85,
	/* PHY busy condition */	
	PHY_BUSY                    = 0x86, 
	/* Frame pending at PHY */
	PHY_FRAME_PENDING           = 0x87,
	/*A parameter in the set/get request is either not supported or is out ofthe valid range*/	
    PHY_INVALID_PARAMETER       = 0x88,
	/*A SET/GET request was issued with the identifier of a PIB attribute that is not supported	*/
    PHY_UNSUPPORTED_ATTRIBUTE   = 0x89, 
	/* The CCA attempt has detected a busy channel.*/
    PHY_CHANNEL_BUSY            = 0x8A,
	/* The CCA attempt has detected an idle channel.*/	
    PHY_CHANNEL_IDLE            = 0x8B,
	/* TRX received no ack for the previously sent packet*/	
    PHY_NO_ACK                  = 0x8C, 
	/* Transmit is failed due to Channel access failure*/
    PHY_CHANNEL_ACCESS_FAILURE  = 0x8D  
            
}PHY_Retval_t;

 // *****************************************************************************
/* PHY Transceiver State Values
 
   Summary:
    Enumeration for Transceiver States that can be set 
 
   Description:
    None
   Remarks:
    None 
 */
 
typedef enum phy_trx_state_tag{
	/* Transceiver to be configured to Transceiver OFF state*/
	PHY_STATE_TRX_OFF,
	/* Transceiver to be configured to Receiver ON state */
	PHY_STATE_RX_ON
}PHY_TrxState_t;

 // *****************************************************************************
/* PHY Transceiver Status Values
 
   Summary:
    Enumeration for current state of the Transceiver
   Description:
    None
   Remarks:
    None 
 */

typedef enum phy_trx_status_tag{
	/* Transceiver is in Transceiver OFF state*/
    PHY_TRX_OFF = 0x08,
	/* Transceiver is in Receiver ON state */
    PHY_RX_ON   = 0x16,
	/* Transceiver is in Transmit ON state */
    PHY_TX_ON   = 0x19,
	/* Transceiver is currently receiving the packet*/
    PHY_BUSY_RX = 0x11,
	/* Transceiver is currently transmitting the packet */
    PHY_BUSY_TX = 0x12,
	/* Transceiver is in sleep state */
    PHY_TRX_SLEEP  = 0x0F,
	/* Transceiver is in Deep sleep state */
    PHY_TRX_DEEP_SLEEP = 0x20
}PHY_TrxStatus_t;

// *****************************************************************************
// *****************************************************************************
// Section: Macros
// *****************************************************************************
#define TFA_PIB_RX_SENS_DEF                 (RSSI_BASE_VAL_BPSK_600_DBM)
// *****************************************************************************

// *****************************************************************************
/* Custom PHY PIB attribute ID 
 
   Summary:
    Seting this attribute enables the device as PAN Coordinator 
   Description:
    if only source addressing fields are included in a data or MAC command frame, 
	the frame shall be accepted only if the device is the PAN coordinator and 
	the source PAN identifier matches macPANId, for details refer to 
	IEEE 802.15.4-2006, Section 7.5.6.2 (third-level filter rule six
   Remarks:
    None 
 */

#define mac_i_pan_coordinator                         (0x0B)

// *****************************************************************************
/* Macro to convert Symbols to Microsecond 
 
   Summary:
    This macro function converts the given symbol value to microseconds 
   Description:
    None
   Remarks:
    None 
 */
#define PHY_CONVERT_SYMBOLS_TO_US(symbols)            \
	(tal_pib.CurrentPage == 0 ? \
	(tal_pib.CurrentChannel == \
	0 ? ((uint32_t)(symbols) * 50) : ((uint32_t)(symbols) *	\
	25)) : \
	(tal_pib.CurrentChannel == \
	0 ? ((uint32_t)(symbols) * 40) : ((uint32_t)(symbols) << \
	4)) \
	)

// *****************************************************************************
/* Macro to convert Microsecond to symbols
 
   Summary:
    This macro function converts the given time in microseconds to symbols 
   Description:
    None
   Remarks:
    None 
 */
#define PHY_CONVERT_US_TO_SYMBOLS(time)               \
	(tal_pib.CurrentPage == 0 ? \
	(tal_pib.CurrentChannel == 0 ? ((time) / 50) : ((time) / 25)) :	\
	(tal_pib.CurrentChannel == 0 ? ((time) / 40) : ((time) >> 4)) \
	)
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Release Version Macros
// *****************************************************************************
// *****************************************************************************


/* Major Number
 
   Summary:
    This macro holds the stack Major number 
   Description:
	None
   Remarks:
    None 
 */
#define MAJOR_NUM                 "1"


/* Minor Number
 
   Summary:
    This macro holds the stack Minor number 
   Description:
	None
   Remarks:
    None 
 */
#define MINOR_NUM                 "0"

/* Patch Number
 
   Summary:
    This macro holds the stack patch number 
   Description:
	None
   Remarks:
    None 
 */
#define PATCH_NUM                 "0"


/* PHY Version
 
   Summary:
    This macro holds the PHY SW version as a String 
   Description:
	None
   Remarks:
    None 
 */
#if (defined RC_NUM)
#define PHY_VER   "802.15.4-PHY v" MAJOR_NUM"." MINOR_NUM"." PATCH_NUM"-rc." RC_NUM
#else
#define PHY_VER   "802.15.4-PHY v" MAJOR_NUM"." MINOR_NUM"." PATCH_NUM
#endif

/* Release version information in 32-bit bitfield 
 
| bit pos | field name      | meaning                        |
|---------|-----------------|------------------------------  |
| 0-13    | reserved        | NA                             |
| 14-17   | build itreation | running version of this release|
| 18-19   | qualifier       | 00 - reserved                  |
|         |                 | 01 - Production (P)            |
|         |                 | 10 - Engineering (E)           |
|         |                 | 11 - reserved                  |
| 20-23   | stack minor     | minor version                  |
| 24-27   | stack major     | major version                  |
| 28-31   | reserved        | NA                             |


Example:
  802.15.4-PHY v1.0.0 is represented as 0x01040000

|0000       |0001        | 0000        | 01        | 0000           | 00000000000000|
|-----------|------------|-------------|-----------|----------------|---------------|
|Reserved   | Stack Major| Stack Minor | Qualifier | Build Iteration| Reserved      |
*/

 
/* PHY Software Version Information in 32-bit bitfield
 
   Summary:
    This macro holds PHY Software Version Information in 32-bit bitfield
   Description:
	None
   Remarks:
    None 
*/
#define PHY_VERSION_VALUE      (0x01040000)

// *****************************************************************************
// *****************************************************************************
// Section: PHY PIB Attribute List
// *****************************************************************************
// *****************************************************************************

// ***************************************************************************** 
/* PHY Information Base (PIB) Attribute list

| PIB Attribute       |AccessType| Type    | DefaultValue     | Range           |
|---------------------|----------|---------|------------------|-----------------|
| phyCurrentChannel   | Get/Set  | uint8_t | 01               | 00 - 10         |
| phyChannelsSupported| Get      | uint32_t| 0x000007FF       | NA              |                    
| phyCurrentPage      | Get/Set  | uint8_t | 0                |0,2,5,16,17,18,19|
| phyTransmitPower    | Get/Set  | uint8_t |                  |                 |
| phyCCAMode          | Get/Set  | uint8_t | 1                | 0 - 3           |
| macIeeeAddress      | Get/Set  | uint64_t| All 0's          | NA              |
| macShortAddress     | Get/Set  | uint16_t| 0xFFFF           | 0x0000 - 0xFFFF |
| macPANId            | Get/Set  | uint16_t| 0xFFFF           | 0x0000 - 0xFFFF |
| macMinBE            | Get/Set  | uint8_t | 3                | 0 - 3           |
| macMaxBE            | Get/Set  | uint8_t | 5                | 3 - 8           |
| macMaxCSMABackoffs  | Get/Set  | uint8_t | 4                | 0 - 5           |
| macMaxFrameRetries  | Get/Set  | uint8_t | 3                | 0 - 7           |
| macPromiscuousMode  | Get/Set  | bool    | 0                | 0 or 1          |
| phySHRDuration      | Get      | uint8_t |pg0-40sym/10Symbols| NA              |
| phySymbolsPerOctet  | Get      | uint8_t |pg0-8sym else 2Symbol| NA              |
| phyMaxFrameDuration | Get      | uint16_t|                  | NA              |
| macIpanCoordinator  | Get/Set  | bool    | 0                | 0 or 1          |
 */

// *****************************************************************************
// *****************************************************************************
// Section: PHY Task Handler Funtions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************    
/*
  Function:
    void PHY_TaskHandler ( void )

  Summary:
    PHY Task Handling routine

  Description:
    This function
   - Checks and allocates the receive buffer.
   - Processes the PHY incoming frame queue.
   - Implements the PHY state machine.

  Precondition:
    PHY_Init should be called before calling this function

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    PHY_TaskHandler();
    </code>

  Remarks:
    This routine must be called from the RTOS Task function incase of any 
    RTOS is used.
*/
void PHY_TaskHandler(void);

// *****************************************************************************    
/*
  Function:
    void TAL_TaskHandler ( void )

  Summary:
    TAL Task Handling routine

  Description:
    This function handles the transceiver interrupts

  Precondition:
    PHY_Init should be called before calling this function

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    TAL_TaskHandler();
    </code>

  Remarks:
    This routine must be called from the RTOS Task function incase of any 
    RTOS is used.
*/
void TAL_TaskHandler(void);

// *****************************************************************************
// *****************************************************************************
// Section: PHY Initialization and Reset Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_Init( void )

  Summary:
    Initialization of PHY Layer

  Description:
    This function is called to initialize the PHY layer. The transceiver is
    initialized and it will be in  PHY_STATE_TRX_OFF, the PHY PIBs are set to 
    their default values. PAL layer is initialized

  Precondition:
    SYS_Load_Cal(WSS_ENABLE_ZB) function of device support library should be 
    called before calling this function. 

  Parameters:
    None.

  Returns:
    PHY_SUCCESS - If the transceiver state is changed to TRX_OFF
    PHY_FAILURE - Otherwise

  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
 
    retVal = PHY_Init();
    if (PHY_SUCCESS =! retVal)
    {
        while(1);
    }
    </code>

  Remarks:
    This routine must be called before any of the PHY function is called
*/
PHY_Retval_t PHY_Init(void);

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_Reset( bool setDefaultPibs )

  Summary:
    Resets the PHY Layer

  Description:
    This function is called to Resets PHY state machine 
    and sets the default PIB values if requested

  Precondition:
    PHY_Init() should have been called before calling this function 

  Parameters:
    setDefaultPibs - Defines whether PIB values need to be set
                     to its default values

  Returns:
    PHY_SUCCESS - If the transceiver state is changed to TRX_OFF and PIBs are set 
                  to their default value 
    PHY_FAILURE - Otherwise

  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    bool setDefault = false;
 
    retVal = PHY_Reset(setDefault);
    if (PHY_SUCCESS =! retVal)
    {
        while(1);
    }
    </code>

  Remarks:
    None
*/

PHY_Retval_t PHY_Reset(bool set_default_pib);

// *****************************************************************************
// *****************************************************************************
// Section: PHY Tranmission Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_TxFrame(PHY_FrameInfo_t *txFrame, PHY_CSMAMode_t csmaMode,
                             bool performFrameRetry)

  Summary:
    Request to PHY to transmit frame

  Description:
    This function is called by the upper layer (MAC/Application) to deliver a 
    frame to the PHY to be transmitted by the transceiver.
  
  Precondition:
    PHY_Init() should have been called before calling this function

  Parameters:
    txFrame     - Pointer to the PHY_FrameInfo_t structure or
                  to frame array to be transmitted
                txFrame->mpdu - Pointer to the PHY Payload. mpdu[0] should hold 
						the length of the payload(N) + 1 (for length field length)
                txFrame->mpdu[1-N] - Hold the phyPayload
  
    csmaMode    - Indicates mode of csma-ca to be performed for this frame
                NO_CSMA_NO_IFS    - Immediate Tx and SIFS(Short InterFrameSpacing) 
                                    between subsequent frames
                NO_CSMA_WITH_IFS  - Immediate Tx and LIFS (Long InterFrameSpacing) 
                                    between subsequent frames
                CSMA_UNSLOTTED    - Hardware CSMA will be performed before packet 
                                    transmission with number of retries configured 
                CSMA_SLOTTED      - Hardware CSMA will be performed - Used with 
                                    Beacon Enabled network - Currently not supported 
                                    by PHY
    performFrameRetry - Indicates whether to retries are to be performed
                        for this frame
                        true - SW retry will be performed for macMaxFrameRetries
                               value
                        false- SW retry is disabled       

  Returns:
    PHY_SUCCESS -  If PHY has accepted the data from the MAC for frame
                   transmission
    PHY_BUSY    -  If PHY is busy servicing the previous MAC request

  Example:
    <code>
    uint8_t txBuffer[LARGE_BUFFER_SIZE];
    uint8_t txData[] = "Wireless!!!";
    PHY_CSMAMode_t csmaMode = CSMA_UNSLOTTED;
    bool performRetry = true;
    PHY_FrameInfo_t txFrame;
    
    txFrame.mpdu = txBuffer;
    txBuffer[0] = sizeof(txData);
    memcpy((uint8_t *)&txBuffer[1], txData, sizeof(txData));
 
    if(PHY_SUCCESS == PHY_TxFrame(&txFrame, csmaMode, performRetry))
    {
        Frame transmitted successfully
    }   
    </code>

  Remarks:
    None
*/
PHY_Retval_t PHY_TxFrame(PHY_FrameInfo_t *txFrame, PHY_CSMAMode_t csmaMode,
		bool performFrameRetry);
		
// *****************************************************************************
/*
  Function:
    void PHY_TxDoneCallback(PHY_Retval_t status, PHY_FrameInfo_t *frame)

  Summary:
    User callback function for the transmission of a frame

  Description:
    This callback function SHOULD be defined by the upper layer(Application/MAC)
    for getting the status of last transmitted packet.
  
  Precondition:
    This is a Asynchronous function call for the transmission of a frame

  Parameters:
    status      - Status of frame transmission attempt
                  PHY_SUCCESS        - The transaction was responded to by a valid ACK, 
                                       or, if no ACK is requested, after a successful
                                       frame transmission.
                  PHY_FRAME_PENDING  - Equivalent to SUCCESS and indicating that 
                                       the ?Frame Pending? bit of the received 
                                       acknowledgment frame was set.
                  PHY_CHANNEL_ACCESS_FAILURE - Channel is still busy after attempting 
                                               MAX_CSMA_RETRIES of CSMA-CA.
                  PHY_NO_ACK         - No acknowledgement frames were received 
                                       during all retry attempts.
                  PHY_FAILURE        - Transaction not yet finished.
                  PHY_RF_REQ_ABORTED - RF is busy performing Higher priority BLE task 
                                       and the transmission is aborted
                  PHY_RF_UNAVAILABLE - RF is currently unavailable for 15.4 subsystem
 
    frame       - Pointer to the PHY_FrameInfo_t structure or
                  to frame array to be transmitted
                txFrame->mpdu - Pointer to the PHY Payload. mpdu[0] should hold 
						the length of the payload(N) + 1 (for length field length)
                txFrame->mpdu[1-N] - Hold the phyPayload     

  Returns:
    None

  Example:
    <code>
    void PHY_TxDoneCallback(PHY_Retval_t status, PHY_FrameInfo_t *frame)
    {
         Keep compiler happy. 
        status = status;
        frame = frame;
    }
    </code>

  Remarks:
    None
*/
void PHY_TxDoneCallback(PHY_Retval_t status, PHY_FrameInfo_t *frame);

// *****************************************************************************
// *****************************************************************************
// Section: PHY Energy Detaction Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_EdStart(uint8_t scan_duration)

  Summary:
    Starts ED Scan

  Description:
    This function starts an ED Scan for the scan duration specified by the
    upper layer. Actual ED result of Energy level on current channel will be 
    indicated by PHY_EdEndCallback(eneryLevel).
    Scan duration formula:  aBaseSuperframeDuration * (2^SD + 1) symbols
                            where SD - scanDuration parameter (0 - 14)

  Precondition:
    PHY_Init() should have been called before calling this function 

  Parameters:
    scanDuration - Specifies the ED scan duration in symbols
                   Range - (0 to 14) 

  Returns:
    PHY_SUCCESS - ED scan is started successfully
    PHY_BUSY - PHY is busy servicing the previous request
    PHY_TRX_ASLEEP - Transceiver is currently sleeping, wakeup the transceiver 
                     using PHY_TrxWakeup() function
    PHY_FAILURE otherwise

  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    uint8_t scanDurationSym = 1;
 
    retVal = PHY_EdStart(scanDurationSym);
    if (PHY_SUCCESS =! retVal)
    {
        ED scan cannot be started at this moment
    }
    </code>

  Remarks:
    PHY_EdEndCallback(energyLevel) will be called after scanning the current 
    channel for a time period of aBaseSuperframeDuration * (2^scanDuration + 1) symbols
    For scanDuration of value 0, ScanTime = 960 *(2^0 +1) 
                                          = 1920 symbols = 30720 us
*/
PHY_Retval_t PHY_EdStart(uint8_t scan_duration);

// *****************************************************************************
/*
  Function:
    void PHY_EdEndCallback(uint8_t energyLevel)

  Summary:
    User callback function for Energy detection

  Description:
    This function SHOULD be defined by the upperlayer (Application/MAC layer) 
    in order to get the energyLevel on the current channel which is being scanned 
    for a period of scanDuration symbols 

  Precondition:
    This is an Asynchronous function call for the energy scan complete 

  Parameters:
    energyLevel - Measured energy level during ED Scan
 
    With energy_level, the RF input power can be calculated as follows
    PRF[dBm] = RSSI_BASE_VAL[dBm] + 1[dB] x energy_level

  Returns:
    None

  Example:
    <code>
    void PHY_EdEndCallback(uint8_t energyLevel)
    {
        int8_t energyLeveldBm = (int8_t) (PHY_GetRSSIBaseVal() + energyLevel);
        energyLevel = energyLevel; 
    }
    </code>

  Remarks:
    None
*/
void PHY_EdEndCallback(uint8_t energyLevel);

// *****************************************************************************
// *****************************************************************************
// Section: PHY Information Base Set/Get Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_PibGet(uint8_t attribute, uint8_t *value)

  Summary:
    Gets a PHY PIB attribute

  Description:
    This function is called to retrieve the transceiver information base
    attributes. The list of PHY PIB attributes, its default values and 
    range are described in the above table. 
    For more information refer ieee_phy_const.h file

  Precondition:
    PHY_init() should have been called before calling this function.

  Parameters:
    attribute - PHY infobase attribute ID
    value     - Pointer to the PHY infobase attribute value

  Returns:
    PHY_UNSUPPORTED_ATTRIBUTE - If the PHY infobase attribute is not found
    PHY_SUCCESS - otherwise

  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    uint8_t phyChannel;
    uint8_t shortAddr;
    
     Getting Current channel
    retVal = PHY_PibGet(phyCurrentChannel, &phyChannel);
    if(PHY_SUCCESS == retVal)
    {
        printf("\r\n PHY Current Channel - %d",phyChannel ); 
    }
    
    Getting short Address 
    retVal = PHY_PibGet(macShortAddr, &shortAddr);
    if(PHY_SUCCESS == retVal)
    {
        printf("\r\n Device short addr - 0x%x",shortAddr ); 
    }
  
    </code>

  Remarks:
    None
*/
PHY_Retval_t PHY_PibGet(uint8_t attribute, uint8_t *value);

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_PibSet(uint8_t attribute, PibValue_t *value)

  Summary:
    Sets the PHY PIB attribute with value

  Description:
    This function is called to set the transceiver information base
    attributes. The list of PHY PIB attributes, its default values and 
    range are described in the above table. 
    For more information refer ieee_phy_const.h file

  Precondition:
    PHY_init() should have been called before calling this function.

  Parameters:
    attribute - PHY infobase attribute ID
    value     - Pointer to the PHY infobase attribute value to be set

  Returns:
    PHY_UNSUPPORTED_ATTRIBUTE - if the PHY info base attribute is not
                                found
    PHY_BUSY - If the PHY is not in PHY_IDLE state. An exception is
               macBeaconTxTime which can be accepted by PHY even if PHY is not
                in PHY_IDLE state.
    PHY_SUCCESS - If the attempt to set the PIB attribute was successful
    PHY_TRX_ASLEEP - If trx is in SLEEP mode and access to trx is
                     required

  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    uint8_t phyChannel = 15;
    uint8_t shortAddr = 0x1234;
    PibValue_t pibValue;
    
    Setting Current channel
    pibValue.pib_value_8bit = phyChannel;
    retVal = PHY_PibSet(phyCurrentChannel, &pibValue);
    if(PHY_SUCCESS == retVal)
    {
        Channel is configured 
    }
  
    Setting short Address 
    pibValue.pib_value_16bit = shortAddr;
    retVal = PHY_PibSet(macShortAddr, &pibValue);
    if(PHY_SUCCESS == retVal)
    {
        Short Address is configured 
    }
  
    </code>

  Remarks:
    None
*/
PHY_Retval_t PHY_PibSet(uint8_t attribute, PibValue_t *value);


// *****************************************************************************
// *****************************************************************************
// Section: PHY Reception Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    PHY_TrxStatus_t PHY_RxEnable(PHY_TrxState_t state)

  Summary:
    Switches receiver on or off

  Description:
    This function switches the receiver on (PHY_STATE_RX_ON) or off (PHY_STATE_TRX_OFF)
 
  Precondition:
    PHY_init() should have been called before calling this function.

  Parameters:
    state - State of the Transceiver to be set
            PHY_STATE_RX_ON - Transceiver will be put into Receive state
            PHY_STATE_TRX_OFF - Transceiver will be put into OFF state         

  Returns:
    PHY_TRX_OFF- If receiver has been switched off, or
    PHY_RX_ON  - otherwise.

  Example:
    <code>
    PHY_TrxStatus_t trxStatus;
    PHY_TrxState_t trxState = PHY_STATE_RX_ON;

    trxStatus = PHY_RxEnable(trxState);
    if(PHY_RX_ON == trxStatus)
    {
        TRX is in receive state 
    } 
    </code>

  Remarks:
    None
*/
PHY_TrxStatus_t PHY_RxEnable(PHY_TrxState_t state);

// *****************************************************************************
/*
  Function:
    void PHY_RxFrameCallback(PHY_FrameInfo_t *rxFrame)

  Summary:
    User callback function for the reception of a frame

  Description:
    This callback function SHOULD be defined by the upper layer(Application/MAC)
    for getting the received frame details 
 
  Precondition:
    This is a Asynchronous function call for the reception of a frame

  Parameters:
    rxFrame - Pointer to received frame structure of type PHY_FrameInfo_t
              or to received frame array
              rxFrame->buffer_header - BMM Buffer Header of the frame
              rxFrame->mpdu - Actual MPDU comprises of 
                       mpdu[0]  - Payload Length(N)
                       mpdu[1-N]- Payload
                       mpdu[N+1]- LQI of received packet
                       mpdu[N+2]- ED_LEVEL of received packet        

  Returns:
    None

  Example:
    <code>
    uint8_t rxBuffer[LARGE_BUFFER_SIZE];
    uint8_t frameLen, frameLQI, frameED;
    int8_t frameRSSI;
    void PHY_RxFrameCallback(PHY_FrameInfo_t *frame)
    {
        printf("\n--RxCallbackreceived--");
        frameLen = frame->mpdu[0];
        Copy the payload 
        memcpy(rxBuffer, (uint8_t *)&(frame->mpdu[1]), frameLen);
        Copy the LQI
        frameLQI = frame->mpdu[frameLen+LQI_LEN];
        Copy the RSSI  
        frameED = frame->mpdu[frameLen+LQI_LEN+ED_VAL_LEN];
             
        frameRSSI = (int8_t)(frameED + PHY_GetRSSIBaseVal());  

         free the buffer that was used for frame reception 
        bmm_buffer_free((buffer_t *)(frame->buffer_header));
    } 
    </code>

  Remarks:
    None
*/
void PHY_RxFrameCallback(PHY_FrameInfo_t *rxFrame);


// *****************************************************************************
// *****************************************************************************
// Section: PHY TRX Power Management Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_TrxSleep(PHY_SleepMode_t mode)

  Summary:
    Sets the transceiver to sleep

  Description:
    This function sets the transceiver to sleep or deep sleep state.
 
  Precondition:
    PHY_Init() should have been called before calling this function

  Parameters:
    mode    - Defines sleep mode of transceiver SLEEP or DEEP_SLEEP     

  Returns:
    PHY_BUSY                - The transceiver is busy in TX or RX
    PHY_SUCCESS             - The transceiver is put to sleep
    PHY_TRX_ASLEEP          - The transceiver is already asleep

  Example:
    <code>
    PHY_SleepMode_t sleepMode = SLEEP_MODE_1;
    bool trxSleepStatus = false;
 
    if (PHY_SUCCESS == PHY_TrxSleep(sleepMode))
	{
		trxSleepStatus = true;
	}  
    </code>

  Remarks:
    When TRX is put into DeepSleep, the TRX registers are reset and it will hold 
    default values, PIB values are getting written by PHY layer when Wakeup 
    function is called.User has to reconfigure the configuration parameters
    (PHY_ConfigParam_t) which are set by application. This procedure is not 
    needed for SLEEP mode as the TRX register values are retained.
*/
PHY_Retval_t PHY_TrxSleep(PHY_SleepMode_t mode);

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_TrxWakeup(void)

  Summary:
    Wakes up the transceiver from sleep

  Description:
    This function awakes the transceiver from sleep state.
 
  Precondition:
    PHY_TrxSleep() should have been called before calling this function

  Parameters:
    None     

  Returns:
    PHY_TRX_AWAKE   - The transceiver is already awake
    PHY_SUCCESS     - The transceiver is woken up from sleep
    PHY_FAILURE     - The transceiver did not wake-up from sleep

  Example:
    <code>
    PHY_SleepMode_t sleepMode = SLEEP_MODE_1;
    bool trxSleepStatus = false;
    Set Transceiver to sleep
    if (PHY_SUCCESS == PHY_TrxSleep(sleepMode))
	{
		trxSleepStatus = true;
	}
    wakeup the transceiver
    if (PHY_SUCCESS == PHY_TrxWakeup())
    {
        trxSleepStatus = false;
    }  
    </code>

  Remarks:
    When TRX is put into DeepSleep, the TRX registers are reset and it will hold 
    default values, PIB values are getting written by PHY layer when Wakeup 
    function is called.User has to reconfigure the configuration parameters
    (PHY_ConfigParam_t) which are set by application. This procedure is not 
    needed for SLEEP mode as the TRX register values are retained.
*/
PHY_Retval_t PHY_TrxWakeup(void);

/*
 * \brief Generates a 16-bit random number used as initial seed for srand()
 *
 * This function generates a 16-bit random number by means of using the
 * Random Number Generator from the transceiver.
 * The Random Number Generator generates 2-bit random values. These 2-bit
 * random values are concatenated to the required 16-bit random seed.
 *
 * The generated random 16-bit number is feed into function srand()
 * as initial seed.
 *
 * The transceiver state is initally set to RX_ON.
 * After the completion of the random seed generation, the
 * trancseiver is set to TRX_OFF.
 *
 * As a prerequisite the preamble detector must not be disabled.
 *
 * Also in case the function is called from a different state than TRX_OFF,
 * additional trx state handling is required, such as reading the original
 * value and restoring this state after finishing the sequence.
 * Since in our case the function is called from TRX_OFF, this is not required
 * here.
 */

 void tal_generate_rand_seed(void);

// *****************************************************************************
// *****************************************************************************
// Section: PHY TRX Feature Access Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    uint8_t PHY_EdSample(void)

  Summary:
    Perform a single ED measurement on current channel

  Description:
    This function is used to measure the energy level on current channel
 
  Precondition:
    PHY_Init() should have been called before calling this function.
 
  Parameters:
    None

  Returns:
    edValue -  Result of the measurement

  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    uint8_t phyChannel = 15;
    uint8_t edLevel;
    int8_t pwrDbm;
    PibValue_t pibValue;
    
    Setting Current channel
    pibValue.pib_value_8bit = phyChannel;
    retVal = PHY_PibSet(phyCurrentChannel, &pibValue);
    if(PHY_SUCCESS == retVal)
    {
        Take the Ed sample
        edLevel = PHY_EdSample();  
        Convert the energy level to input power in Dbm
        pwrDbm = (int8_t)(edLevel + PHY_GetRSSIBaseVal());
    }
    
    </code>

  Remarks:
    PHY_EdSample scans the channel for 8 symbols(128us) and returns the energy level  
*/
uint8_t PHY_EdSample(void);

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_CCAPerform(void)

  Summary:
    Perform a clear channel assessment

  Description:
    This function is used to perform a clear channel assessment on current channel.
    using configured CCA mode (can be set using PHY_PibSet of phyCCAMode). 
    This results in the status of channel is current busy or Idle.
    
    The different CCA modes supported by Transceiver are
    The CCA mode
    - CCA Mode 1: Energy above threshold. CCA shall report a busy medium
    upon detecting any energy above the ED threshold.
    - CCA Mode 2: Carrier sense only. CCA shall report a busy medium only upon
    the detection of a signal with the modulation and spreading characteristics
    of IEEE 802.15.4. This signal may be above or below the ED threshold.
    - CCA Mode 3: Carrier sense with energy above threshold. CCA shall report a
    busy medium only upon the detection of a signal with the modulation and
    spreading characteristics of IEEE 802.15.4 with energy above the ED
    threshold. 
 
  Precondition:
    PHY_Init() should have been called before calling this function.
 
  Parameters:
    None

  Returns:
    PHY_Retval_t  - PHY_CHANNEL_IDLE or PHY_CHANNEL_BUSY
                    PHY_CHANNEL_IDLE - The CCA attempt has detected an idle channel
                    PHY_CHANNEL_BUSY - The CCA attempt has detected a busy channel

  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    uint8_t phyChannel = 15;
    PibValue_t pibValue;
    bool isChIdle = false;
 
    Setting Current channel
    pibValue.pib_value_8bit = phyChannel;
    retVal = PHY_PibSet(phyCurrentChannel, &pibValue);
    if(PHY_SUCCESS == retVal)
    {
        Take the Ed sample
        retVal = PHY_CCAPerform(); 
        if (PHY_CHANNEL_IDLE == retVal)
        {
            isChIdle = true;
        } 
    }
    
    </code>

  Remarks:
    None 
*/
PHY_Retval_t PHY_CCAPerform(void);



// *****************************************************************************
/**
 * @brief Initializes the TFA
 *
 * This function is called to initialize the TFA.
 *
 * @return MAC_SUCCESS if everything went correct;
 *         FAILURE otherwise
 *
 * @ingroup apiTfaApi
 */
PHY_Retval_t tfa_init(void);

/**
 * @brief Reset the TFA
 *
 * This function is called to reset the TFA.
 *
 * @param set_default_pib Defines whether PIB values need to be set
 *                        to its default values
 *
 * @ingroup apiTfaApi
 */
void tfa_reset(bool set_default_pib);

/**
 * @brief Gets a TFA PIB attribute
 *
 * This function is called to retrieve the transceiver information base
 * attributes.
 *
 * @param[in] tfa_pib_attribute TAL infobase attribute ID
 * @param[out] value TFA infobase attribute value
 *
 * @return MAC_UNSUPPORTED_ATTRIBUTE if the TFA infobase attribute is not found
 *         MAC_SUCCESS otherwise
 *
 * @ingroup apiTfaApi
 */
PHY_Retval_t tfa_pib_get(PHY_tfa_pib_t tfa_pib_attribute, void *value);

/**
 * @brief Sets a TFA PIB attribute
 *
 * This function is called to set the transceiver information base
 * attributes.
 *
 * @param[in] tfa_pib_attribute TFA infobase attribute ID
 * @param[in] value TFA infobase attribute value to be set
 *
 * @return MAC_UNSUPPORTED_ATTRIBUTE if the TFA info base attribute is not found
 *         TAL_BUSY if the TAL is not in TAL_IDLE state.
 *         MAC_SUCCESS if the attempt to set the PIB attribute was successful
 *
 * @ingroup apiTfaApi
 */
PHY_Retval_t tfa_pib_set(PHY_tfa_pib_t tfa_pib_attribute, void *value);

/*
  Function:
    void PHY_StartContinuousTransmit(PHY_ContinuousTxMode_t txMode, 
                                        bool randomContent)

  Summary:
    Starts continuous transmission on current channel

  Description:
    This function is called to start the continuous transmission on current 
    channel.
 
  Precondition:
    PHY_Init() should have been called before calling this function.
 
  Parameters:
    txMode        - Mode of continuous transmission 
                    CW_MODE   - Continuous Wave mode to transmit 
                                the signal at Fc +&- 0.5MHz frequency
                    PRBS_MODE - PRBS mode to Pseudorandom Binary Sequence frame 
                                continuously
    randomContent - Use random content if true 

  Returns:
    None

  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    uint8_t phyChannel = 15;
    PHY_ContinuousTxMode_t txMode = CW_MODE;
    bool useRandomContent = false;
    PibValue_t pibValue;
    
    Setting Current channel
    pibValue.pib_value_8bit = phyChannel;
    retVal = PHY_PibSet(phyCurrentChannel, &pibValue);
    if(PHY_SUCCESS == retVal)
    {
        Start continuous tx in CW mode
        PHY_StartContinuousTransmit(txMode, useRandomContent);       
    }
    
    </code>

  Remarks:
    None 
*/
void PHY_StartContinuousTransmit(PHY_ContinuousTxMode_t txMode, 
                                 bool randomContent);
// *****************************************************************************
/*
  Function:
    void PHY_StopContinuousTransmit(void)

  Summary:
    Stops the continuous transmission on the current channel

  Description:
    This function is called to stop the continuous transmission
 
  Precondition:
    PHY_Init() should have been called before calling this function.
    This function will stop the continuous transmission which is started by 
    PHY_StartContinuousTransmit()function.
 
  Parameters:
    None 

  Returns:
    None

  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    uint8_t phyChannel = 15;
    PHY_ContinuousTxMode_t txMode = PRBS_MODE;
    bool useRandomContent = true;
    bool contTxStarted = false;
    PibValue_t pibValue;
    
    Setting Current channel
    pibValue.pib_value_8bit = phyChannel;
    retVal = PHY_PibSet(phyCurrentChannel, &pibValue);
    if(PHY_SUCCESS == retVal)
    {
        Start continuous tx in CW mode
        PHY_StartContinuousTransmit(txMode, useRandomContent);
        contTxStarted = true;      
    }
    
    if(contTxStarted)
    {
       Stop continuous tx
       PHY_StopContinuousTransmit();
    }   
    </code>

  Remarks:
    When continuous tx is stopped, the PHY_Reset function is called.
    User has to reconfigure the configuration parameters
    (PHY_ConfigParam_t) which are set by application.
*/
void PHY_StopContinuousTransmit(void);

// *****************************************************************************
// *****************************************************************************
// Section: PHY TRX Configuration Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    PHY_Retval_t  PHY_ConfigAntennaDiversity(bool divCtrl, uint8_t antCtrl)

  Summary:
    Configures antenna diversity and selects antenna

  Description:
    This function is used to enable the Antenna Diversity feature and 
    to select the antenna path if the feature is disabled.
    Antenna Diversity uses two antennas to select the most reliable RF signal path. 
    To ensure highly independent receive signals on both antennas, 
    the antennas should be carefully separated from each other.
    If a valid IEEE 802.15.4 frame is detected on one antenna, this antenna is 
    selected for reception. Otherwise the search is continued on the other antenna 
    and vice versa.
 
  Precondition:
    PHY_Init() should have been called before calling this function

  Parameters:
    divCtrl  - true/false to enable/disable antenna diversity algorithm
    antCtrl  - 0 or 3 when antenna diversity is enabled
               1 or 2 to select antenna 1 or antenna 2   

  Returns:
    PHY_SUCCESS -  If Antenna Diversity/ Control bits are configured correctly
    PHY_FAILURE -  otherwise

  Example:
    <code>
    bool antDiv = ANTENNA_DIVERSITY_DISABLE;
    uint8_t antennaSel =  ANTENNA_CTRL_1;
 
     Antenna Diversity is disabled and Antenna 1 is selected for rx/tx path
	PHY_ConfigAntennaDiversity(antDiv, antennaSel);
    
     To get the antenna diversity value configured in the TRX
    PHY_GetTrxConfig(ANT_DIV, &antDiv); 
     To get antenna selected for rx/tx
    PHY_GetTrxConfig(ANT_SELECT, &antennaSel);    
    </code>

  Remarks:
    None 
*/
PHY_Retval_t  PHY_ConfigAntennaDiversity(bool divCtrl, uint8_t antCtrl);

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
 
     Reduce the PDT level 
	PHY_ConfigRxSensitivity(pdtLevel);
    
     To get the PDT level configured
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
 
     Enable Promiscuous mode
	PHY_ConfigRxPromiscuousMode(promCtrl);
    
     To get the PDT level configured
    PHY_GetTrxConfig(AACK_PROMSCS_MODE, &promCtrl); 
     
    </code>

  Remarks:
    None 
*/
PHY_Retval_t PHY_ConfigRxPromiscuousMode(bool promCtrl);


// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_ConfigAutoAck(bool enableAACK)

  Summary:
    Configures TRX for auto acknowledging the reserved frame

  Description:
    The function is used to configure the automatic acknowledgment from 
    Transceiver after packet reception. 
 
  Precondition:
    PHY_Init() should have been called before calling this function.
 
  Parameters:
    nableAACK - true -  to enable the automatic 
                        acknowledgment after reception
                false - to disable the automatic 
                        acknowledgment after reception
 

  Returns:
    PHY_Retval_t - PHY_SUCCESS  If trx is configured correctly
 *                 PHY_FAILURE  otherwise
  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    bool isEnableAACK = true;
 
    retVal = PHY_ConfigAutoAck(isEnableAACK);
    if(PHY_SUCCESS == retVal)
    {
        Trx is configured to auto acknowledge for the received packet
    }   
    </code>

  Remarks:
    None 
*/
PHY_Retval_t PHY_ConfigAutoAck(bool enableAACK);

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_ConfigReservedFrameFiltering(bool recReservedFrame, 
        bool bypassFrameFilter )

  Summary:
    Configures TRX for receiving reserved frame

  Description:
    This function is used to configure the trx for receiving the reserved frame 
    type frames and to enable/disable the frame filtering . 
 
  Precondition:
    PHY_Init() should have been called before calling this function.
 
  Parameters:
    recReservedFrame    - true  to enable the reception of reserved frame types 
                          acknowledgment after reception
    bypassFrameFilter   - true to bypass the frame filtering at the hardware 
                           level like data frame as specified in IEEE specification

  Returns:
    PHY_Retval_t - PHY_SUCCESS  If trx is configured correctly
 *                 PHY_FAILURE  otherwise
  Example:
    <code>
    PHY_Retval_t retVal = PHY_FAILURE;
    bool rxResFrame = true;
    bool bypassFrameFiltering = false;
 
    retVal = PHY_ConfigReservedFrameFiltering(rxResFrame, bypassFrameFiltering);
    if(PHY_SUCCESS == retVal)
    {
        Trx is configured to receive the reserved frame and to do the frame 
         filtering as stated in IEEE Spec
    }   
    </code>

  Remarks:
    None 
*/
PHY_Retval_t PHY_ConfigReservedFrameFiltering(bool recReservedFrame, 
        bool bypassFrameFilter );

// *****************************************************************************
/*
  Function:
    PHY_Retval_t PHY_GetTrxConfig(PHY_ConfigParam_t parameter, uint8_t *paramValue)

  Summary:
    To read a current setting of particular transceiver parameter

  Description:
    The function is used to read the current of particular parameter. 
    The following parameters can be read from TRX,
         Antenna Diversity 
        ANT_DIVERSITY     
         Antenna Configured - ANTENNA_1/ANTENNA_2
        ANT_SELECT        
         Antenna Control
        ANT_CTRL          
         Promiscuous Mode
        AACK_PROMSCS_MODE 
         Tx Power Configured
        TX_PWR            
         Rx Sensitivity
        RX_SENS           
         RX Reduced Power Consumption
        RX_RPC                
         Automatic acknowledgement
        RX_AUTO_ACK       
         Reserved frame reception
        RX_RESERVED_FRAME 
         Filter reserved frame
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
 
     To get the promiscuous mode configured
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
    PHY_TRX_OFF - The transceiver is in TRX_OFF state
    PHY_RX_ON - The transceiver is in receive state
    PHY_TX_ON - The transceiver is in Transmit state
    PHY_BUSY_RX - The transceiver currently receiving the packet
    PHY_BUSY_TX - The transceiver is currently transmitting the packet
    PHY_TRX_SLEEP - The transceiver is in sleep state
    PHY_DEEP_SLEEP - The transceiver is in Deep sleep state

  Example:
    <code>
    PHY_TrxStatus_t trxStatus;
    Gets the current status of trx
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
 
     Get RSSI base value of TRX
	trxBaseRSSI = PHY_GetRSSIBaseVal();
    
    </code>

  Remarks:
    None 
*/
int8_t PHY_GetRSSIBaseVal(void);

// *****************************************************************************
/*
  Function:
    uint32_t PHY_GetSWVersion(void)

  Summary:
    To Get the current Software version of PHY

  Description:
    This function is used Get the current Software version of PHY
 
  Precondition:
    PHY_Init() should have been called before calling this function

  Parameters:
    None 

  Returns:
    32-bit PHY SW version value

  Example:
    <code>
    uint32_t phySwVersion;
 
     Get sw version of the PHY
	phySwVersion = PHY_GetSWVersion();
    
    </code>

  Remarks:
    None 
*/
uint32_t PHY_GetSWVersion(void);

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
 
     To get the dBm value corresponding to power register index
	PHY_ConvertTxPwrRegValToDbm(pwrRegIndex, &pwrDbm);
    
    </code>

  Remarks:
    None 
*/
PHY_Retval_t PHY_ConvertTxPwrRegValToDbm(uint8_t regValue, int8_t *dbmValue);

/**
 * @brief Conversion of symbols to microseconds
 */
uint32_t tal_convert_symbols_to_us_def(uint32_t symbols);

/**
 * @brief Conversion of microseconds to symbols
 */
uint32_t tal_convert_us_to_symbols_def(uint32_t time_);



#define ANT_CTRL_0              (0U) 
#define ANT_CTRL_1              (1U)
#define ANT_CTRL_2              (2U)
#define ANT_CTRL_3              (3U)

// #define ANT_EXTERNAL_SW_ENABLE        (1)
// #define ANT_EXTERNAL_SW_DISABLE       (0)
// #define ANT_AUTO_SEL                  (0)

#define MAX_PDT_LEVEL           (0x0FU)

#define REGISTER_VALUE          (0x01U)
#define DBM_VALUE               (0x00)

#define AACK_PROM_ENABLE        (0x01U)
#define AACK_PROM_DISABLE       (0x00U)


#define CC_1_START_FREQUENCY   (769.0f)
#define CC_1_END_FREQUENCY     (794.5f)
#define CC_2_START_FREQUENCY   (857.0f)
#define CC_2_END_FREQUENCY     (882.5f)
#define CC_3_START_FREQUENCY   (903.0f)
#define CC_3_END_FREQUENCY     (928.5f)
#define CC_4_START_FREQUENCY   (769.0f)
#define CC_4_END_FREQUENCY     (863.0f)
#define CC_5_START_FREQUENCY   (833.0f)
#define CC_5_END_FREQUENCY     (935.0f)
#define CC_6_START_FREQUENCY   (902.0f)
#define CC_6_END_FREQUENCY     (927.5f)

#define CC_BAND_0               (0x00U)
#define CC_BAND_1               (0x01U)
#define CC_BAND_2               (0x02U)
#define CC_BAND_3               (0x03U)
#define CC_BAND_4               (0x04U)
#define CC_BAND_5               (0x05U)
#define CC_BAND_6               (0x06U)

#define MIN_CC_BAND_4_OFFSET    (0x5EU)
#define MIN_CC_BAND_5_OFFSET    (0x66U)

#define MAX_CC_BAND              (0x06U)





//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* PHY_H */
/* EOF */
