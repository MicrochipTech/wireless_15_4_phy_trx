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


/* === INCLUDES ============================================================ */
#include "definitions.h"
#include "../../../pal/inc/pal.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/trx_access_2.h"
#else
#include "../../at86rf/inc/phy_trx_reg_access.h"
#endif

static irq_handler_t irq_hdl_trx = NULL;

void trx_reg_write(uint8_t addr, uint8_t value)
{
    pal_trx_irq_dis();
	/* Prepare the command byte */
	addr |= WRITE_ACCESS_COMMAND;

	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();

	/* Send the Read command byte */
    while (${SELECTED_SERCOM}_IsBusy()){} 
    ${SELECTED_SERCOM}_Write(&addr, 1);
    
    /* Write the byte in the transceiver data register */
    while (${SELECTED_SERCOM}_IsBusy()){}
    ${SELECTED_SERCOM}_Write(&value, 1);

    while (${SELECTED_SERCOM}_IsBusy()){}

	/* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();
    pal_trx_irq_en();    
}


uint8_t trx_reg_read(uint8_t addr)
{
    pal_trx_irq_dis();
	uint16_t register_value;

	/* Prepare the command byte */
	addr |= READ_ACCESS_COMMAND;
    
	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();

    /* Send the Read command byte */ 
    while (${SELECTED_SERCOM}_IsBusy()){}
    ${SELECTED_SERCOM}_Write(&addr, 1);

   /* Read the byte from the transceiver data register */
    while (${SELECTED_SERCOM}_IsBusy()){}
    ${SELECTED_SERCOM}_Read(&register_value, 1);

    while(${SELECTED_SERCOM}_IsBusy()){}

	/* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();
    pal_trx_irq_en();

	return register_value;
}


void trx_reg_bit_write(uint8_t addr, uint8_t mask, uint8_t pos, uint8_t value)
{ 
  uint8_t reg;
  reg = trx_reg_read(addr);
  reg = ((reg & ~mask) | (((value) << pos) & mask));
  trx_reg_write(addr, reg);
}


uint8_t trx_reg_bit_read(uint8_t addr, uint8_t mask, uint8_t pos)
{
  uint8_t reg;
  reg = trx_reg_read(addr);
  reg = (reg & mask) >> (pos); 
  return reg;
}


void trx_frame_write(uint8_t* buf, uint8_t length)
{
    pal_trx_irq_dis();
	uint8_t temp;

	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();
	temp = TRX_CMD_FW;

    while (${SELECTED_SERCOM}_IsBusy()){}
    /* Send the command byte */
    ${SELECTED_SERCOM}_Write(&temp, 1);
    while (${SELECTED_SERCOM}_IsBusy()){}
    /* Write into the buffer */
    ${SELECTED_SERCOM}_Write(buf, length);
    
    while (${SELECTED_SERCOM}_IsBusy()){}

	/* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set(); 
    pal_trx_irq_en();

}

void trx_frame_read(uint8_t* buf, uint8_t length)
{
    pal_trx_irq_dis();
	uint16_t temp;
	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();

	temp = TRX_CMD_FR;

    while (${SELECTED_SERCOM}_IsBusy()){}
    /* Send the command byte */
    ${SELECTED_SERCOM}_Write(&temp, 1);
    while (${SELECTED_SERCOM}_IsBusy()){}

    ${SELECTED_SERCOM}_Read(buf, length);
    
    while (${SELECTED_SERCOM}_IsBusy()){}
 
	/* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();
    pal_trx_irq_en();

}

void trx_sram_write(uint8_t addr, uint8_t *data, uint8_t length)
{
    pal_trx_irq_dis();
    uint8_t temp;

	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();
    
    temp = TRX_CMD_SW;

    /* Send the command byte */
    while (${SELECTED_SERCOM}_IsBusy()){}
    ${SELECTED_SERCOM}_Write(&temp, 1);
    
	/* Send the address from which the write operation should start */    
    while (${SELECTED_SERCOM}_IsBusy()){}
    ${SELECTED_SERCOM}_Write(&addr, 1);

    while (${SELECTED_SERCOM}_IsBusy()){}
    ${SELECTED_SERCOM}_Write(data, length);
    
    while (${SELECTED_SERCOM}_IsBusy()){}
    
    /* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();
    pal_trx_irq_en();
    
}

void trx_sram_read(uint8_t addr, uint8_t *data, uint8_t length)
{
    pal_trx_irq_dis();
	uint16_t temp;

	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();
    
    temp = TRX_CMD_SR;

     /* Send the command byte */   
    while (${SELECTED_SERCOM}_IsBusy()){}
    ${SELECTED_SERCOM}_Write(&temp, 1);
        
	/* Send the address from which the read operation should start */
    while (${SELECTED_SERCOM}_IsBusy()){}
    ${SELECTED_SERCOM}_Write(&addr, 1);
    
    /* Upload the received byte in the user provided location */
    while (${SELECTED_SERCOM}_IsBusy()){}
    ${SELECTED_SERCOM}_Read(data, length);
    
    while (${SELECTED_SERCOM}_IsBusy()){}
    
    /* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();
    pal_trx_irq_en();
    
}

void trx_read(uint16_t addr, uint8_t *data, uint16_t length)
{
    uint8_t temp1 = 0U;
    uint16_t temp = 0U,temp2 = 0U;

	/*Saving the current interrupt status & disabling the global interrupt
	**/
    pal_trx_irq_dis();//	DISABLE_TRX_INTERRUPT();
	/* Prepare the command byte */
	addr |= READ_ACCESS_COMMAND;
    temp1 = addr>>8U; //rsh
    temp2 = addr<<8U; //lsh
    temp = temp2 | temp1;
    
	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();
    
    /* Send the Read command byte */ 
    while (${SELECTED_SERCOM}_IsBusy()){}
    ${SELECTED_SERCOM}_Write(&temp, 2);

    
    while(${SELECTED_SERCOM}_IsBusy()){}
    
    ${SELECTED_SERCOM}_Read(data, length);

    while(${SELECTED_SERCOM}_IsBusy()){}

	/* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();

    
	/*Restoring the interrupt status which was stored & enabling the global
	 * interrupt */

    pal_trx_irq_en();//	ENABLE_TRX_INTERRUPT();
}

void trx_write(uint16_t addr, uint8_t *data, uint16_t length)
{
        uint8_t temp1 = 0U;
    uint16_t temp = 0U,temp2 = 0U;

	/*Saving the current interrupt status & disabling the global interrupt
	**/
    pal_trx_irq_dis();//	DISABLE_TRX_INTERRUPT();
	/* Prepare the command byte */
	addr |= WRITE_ACCESS_COMMAND;
//    temp = addr>>8U;
    temp1 = addr>>8U; //rsh
    temp2 = addr<<8U; //lsh
    temp = temp2 | temp1;

	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();

	/* Send the Read command byte */
    while (${SELECTED_SERCOM}_IsBusy()){} 
    
    ${SELECTED_SERCOM}_Write(&temp, 2);
    
    /* Write the byte in the transceiver data register */
    while (${SELECTED_SERCOM}_IsBusy()){}
    
    ${SELECTED_SERCOM}_Write(data, length);

    while (${SELECTED_SERCOM}_IsBusy()){}

	/* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();

	/*Restoring the interrupt status which was stored & enabling the global
	 * interrupt */
    pal_trx_irq_en();//	ENABLE_TRX_INTERRUPT();
}

void trx_irq_init(void * trx_irq_cb)
{
	/*
	 * Set the handler function.
	 * The handler is set before enabling the interrupt to prepare for
	 * spurious
	 * interrupts, that can pop up the moment they are enabled
	 */
	irq_hdl_trx = (irq_handler_t)trx_irq_cb;
}

uint8_t trx_bit_read(uint16_t addr, uint8_t mask, uint8_t pos)
{
	uint8_t ret;
	ret = trx_reg_read(addr);
	ret &= mask;
	ret >>= pos;
	return ret;
}

void trx_bit_write(uint16_t reg_addr, uint8_t mask, uint8_t pos,uint8_t new_value)
{
	uint8_t current_reg_value;
	current_reg_value = trx_reg_read(reg_addr);
	current_reg_value &= ~mask;
	new_value <<= pos;
	new_value &= mask;
	new_value |= current_reg_value;
	trx_reg_write(reg_addr, new_value);
}
