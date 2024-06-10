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



#ifndef PHY_CONSTANTS_H
#define PHY_CONSTANTS_H

// *****************************************************************************
// *****************************************************************************
// Section: Macros
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Minimum channel
 
   Summary:
    This macro defines the Minimum channel number of Transciver
   Description:
    None
   Remarks:
    None 
 */

#define MIN_CHANNEL                 (0U)

// *****************************************************************************
/* Maximum channel
 
   Summary:
    This macro defines the Maximum channel number of Transciver
   Description:
    None
   Remarks:
    None 
 */
#define MAX_CHANNEL                 (10U)

// *****************************************************************************
/* Valid Channel Mask
 
   Summary:
    This macro defines the Valid Channel Mask used for ED Scanning
   Description:
    None
   Remarks:
    None 
 */
#define VALID_CHANNEL_MASK          (0x000007FFUL)

// *****************************************************************************
/* Symbols per Octet
 
   Summary:
    This macro defines the number of symbols per octet
   Description:
    Depending on the channel page either
     1 bit forms one symbol (BPSK in channel page 0) or
     4 bit form one symbol (O-QPSK in channel page 2).
   Remarks:
    None 
 */

#define SYMBOLS_PER_OCTET                   (tal_pib.CurrentPage == 0 ? 8 : 2)

// *****************************************************************************
/* No of Symbols included in the preamble
 
   Summary:
    This macro defines the number of symbols includes in the preamble
   Description:
    None
   Remarks:
    None 
 */

#define NO_SYMBOLS_PREAMBLE                 (tal_pib.CurrentPage == 0 ? 32U : 8U)

// *****************************************************************************
/* Number of symbols included in the SFD field
 
   Summary:
    This macro defines the number of symbols includes in SFD Field
   Description:
    None
   Remarks:
    None 
 */

#define NO_SYMBOLS_SFD                      (tal_pib.CurrentPage == 0 ? 8U : 2U)

// *****************************************************************************
/* Number of symbols forming the synchronization header (SHR) for the current
   PHY.
 
   Summary:
    This macro defines the number of symbols forming the synchronization header   
    (SHR) for the current PHY. This value is the base for the PHY PIB attribute 
	phySHRDuration.
   Description:
    None
   Remarks:
    None 
 */

#define NO_OF_SYMBOLS_PREAMBLE_SFD          (NO_SYMBOLS_PREAMBLE + \
	NO_SYMBOLS_SFD)

// *****************************************************************************
/* Maximum number of symbols in a frame for the current PHY.
 
   Summary:
    This macro defines the Maximum number of symbols in a frame for the current PHY  
    This value is the base for the PHY PIB attribute phyMaxFrameDuration.
   Description:
    None
   Remarks:
    None 
 */

#define MAX_FRAME_DURATION \
	(NO_OF_SYMBOLS_PREAMBLE_SFD + \
	(aMaxPHYPacketSize + 1U) * SYMBOLS_PER_OCTET)

// *****************************************************************************
/* Maximum Symbol Time
 
   Summary:
    The maximum time in symbols for a 32 bit timer
   Description:
    None
   Remarks:
    None 
 */

#define MAX_SYMBOL_TIME                     (0x0FFFFFFF)

// *****************************************************************************
/* Symbol Mask
 
   Summary:
    Symbol mask for ignoring most significant nibble
   Description:
    None
   Remarks:
    None 
 */

#define SYMBOL_MASK                         (0x0FFFFFFF)

// *****************************************************************************




// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************








// *****************************************************************************
/* PWR_REGISTER_VALUE
 
   Summary:
    Macro for selecting power configuration mode as Register set
   Remarks:
    None 
 */
#define PWR_REGISTER_VALUE                  (1)

// *****************************************************************************
/* PWR_DBM_VALUE
 
   Summary:
    Macro for selecting power configuration mode as Dbm set
   Remarks:
    None 
 */
#define PWR_DBM_VALUE                       (0)

// *****************************************************************************
/* PROMISCUOUS_ENABLE
 
   Summary:
    Macro for enabling promiscuous mode
   Remarks:
    None 
 */
#define PROMISCUOUS_ENABLE                  (1)

// *****************************************************************************
/* PROMISCUOUS_DISABLE
 
   Summary:
    Macro for disabling promiscuous mode
   Remarks:
    None 
 */
#define PROMISCUOUS_DISABLE                 (0)

// *****************************************************************************
/* NUMBER_OF_TOTAL_PHY_TIMERS
 
   Summary:
    Total numbers of software timer instance used by PHY Layer
   Remarks:
    None 
 */
#define NUMBER_OF_TOTAL_PHY_TIMERS        (1)

#endif /* PHY_CONSTANTS_H */

/* EOF */
