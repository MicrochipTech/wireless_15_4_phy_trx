<#--
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
-->


#ifndef TRX_REG_ACCESS_H    /* Guard against multiple inclusion */
#define TRX_REG_ACCESS_H



/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "at86rf.h"
#include "../default/definitions.h" 
#include "../../../pal/inc/pal.h"
/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

   
/**
 * Write access command of the transceiver
 */
#define WRITE_ACCESS_COMMAND            (0xC0U)

/**
 * Read access command to the tranceiver
 */
#define READ_ACCESS_COMMAND             (0x80U)

/**
 * Frame write command of transceiver
 */
#define TRX_CMD_FW                      (0x60U)

/**
 * Frame read command of transceiver
 */
#define TRX_CMD_FR                      (0x20U)

/**
 * SRAM write command of transceiver
 */
#define TRX_CMD_SW                      (0x40U)

/**
 * SRAM read command of transceiver
 */
#define TRX_CMD_SR                      (0x00U)

#define TRX_TRIG_DELAY()  {nop(); nop(); }

void trx_reg_write(uint8_t addr, uint8_t value);

uint8_t trx_reg_read(uint8_t addr);

void trx_reg_bit_write(uint8_t addr, uint8_t mask, uint8_t pos, uint8_t value);

uint8_t trx_reg_bit_read(uint8_t addr, uint8_t mask, uint8_t pos);

void trx_frame_write(uint8_t* buf, uint8_t length);

void trx_frame_read(uint8_t* buf, uint8_t length);

void trx_sram_write(uint8_t addr, uint8_t *data, uint8_t length);

void trx_sram_read(uint8_t addr, uint8_t *data, uint8_t length);



   /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif 

/* *****************************************************************************
 End of File
 */


#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */