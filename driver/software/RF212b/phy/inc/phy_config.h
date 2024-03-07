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
#ifndef PHY_CONFIG_H
#define PHY_CONFIG_H

// ****************************************************************************
// ****************************************************************************
// Section: Macros
// ****************************************************************************
// ****************************************************************************

// ****************************************************************************
#define PHY_RADIO_WAKEUP_TIME_SYM       (PHY_CONVERT_US_TO_SYMBOLS( \
	SLEEP_TO_TRX_OFF_TYP_US))



#ifdef ENABLE_FTN_PLL_CALIBRATION

/*
 * PLL calibration and filter tuning timer timeout in minutes
 */
#define TAL_CALIBRATION_TIMEOUT_MIN         (5UL)

/*
 * PLL calibration and filter tuning timer timeout in us,
 */
#define TAL_CALIBRATION_TIMEOUT_US          ((TAL_CALIBRATION_TIMEOUT_MIN) * \
	(60UL) * (1000UL) * (1000UL))
#endif  /* ENABLE_FTN_PLL_CALIBRATION */

/* === TYPES =============================================================== */
#ifdef ENABLE_QUEUE_CAPACITY
#define TAL_INCOMING_FRAME_QUEUE_CAPACITY   (255)
#endif  /* ENABLE_QUEUE_CAPACITY */


#endif 
