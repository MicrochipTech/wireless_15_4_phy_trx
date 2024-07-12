/**
 * @file trx_access_2.c
 *
 * @brief Performs interface functionalities between the PHY layer and ASF
 * drivers
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
 *
 *
 */

/*
 * Copyright (c) 2015-2018, Microchip Technology Inc All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */


#if SAMD || SAMR21
#include "spi.h"
#else
//#include "spi_master.h"
#endif
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/trx_access_2.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/tal.h"
#include "config/default/driver/IEEE_802154_PHY/phy/at86rf215/inc/at86rf215.h"
static irq_handler_t irq_hdl_trx = NULL;

#if SAMD || SAMR21
struct spi_slave_inst_config slave_dev_config;
struct spi_config config;
struct spi_module master;
struct spi_slave_inst slave;
uint16_t dummy_read;
#else
//struct spi_device SPI_AT86RFX_DEVICE = {
//	/* ! Board specific select id */
//	.id = SPI_SS_PIN //AT86RFX_SPI_CS
//};
#endif

//#if SAMD || SAMR21
//void AT86RFX_ISR(void);
//
//void AT86RFX_ISR(void)
//#else
//AT86RFX_ISR()
//#endif
//
//{
//	/*Clearing the RF interrupt*/
//	trx_irq_flag_clr();
//
//	/*Calling the interrupt routines*/
//	if (irq_hdl_trx) {
//		irq_hdl_trx();
//	}
//}


void PhyReset(void)
{
	/* Ensure control lines have correct levels. */
	RST_HIGH();
	//SLP_TR_LOW();

	/* Wait typical time of timer TR1. */
	PAL_TimerDelay(330);//330us

	RST_LOW();
	PAL_TimerDelay(10);//10us
	RST_HIGH();
}

#define TAL_DEFAULT_BB_IRQ_MASK     (BB_IRQ_TXFE | BB_IRQ_RXFE)
#define TAL_DEFAULT_RF_IRQ_MASK     RF_IRQ_ALL_IRQ

/*Temporary set of definitions to test parallel PER Test in 215*/
/* todo : Implement spi using dma to reduce cpu dependency for writing 2000bytes
 * payload for spi */
#define DISABLE_TRX_INTERRUPT()     trx_reg_write( RG_BBC0_IRQM, 0); \
	trx_reg_write(BB_BASE_ADDR_OFFSET + RG_BBC0_IRQM, 0); \
	trx_reg_write( RG_RF09_IRQM, 0); \
	trx_reg_write(RF_BASE_ADDR_OFFSET + RG_RF09_IRQM, 0);

#define ENABLE_TRX_INTERRUPT()         trx_reg_write( RG_BBC0_IRQM, \
		TAL_DEFAULT_BB_IRQ_MASK); \
	trx_reg_write(BB_BASE_ADDR_OFFSET + RG_BBC0_IRQM, \
		TAL_DEFAULT_BB_IRQ_MASK); \
	trx_reg_write( RG_RF09_IRQM, TAL_DEFAULT_RF_IRQ_MASK); \
	trx_reg_write(RF_BASE_ADDR_OFFSET + RG_RF09_IRQM, \
		TAL_DEFAULT_RF_IRQ_MASK);

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
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Write(&temp, 2);

    
    while(SERCOM5_SPI_IsBusy()){}
    
    SERCOM5_SPI_Read(data, length);

    while(SERCOM5_SPI_IsBusy()){}

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
    while (SERCOM5_SPI_IsBusy()){} 
    
    SERCOM5_SPI_Write(&temp, 2);
    
    /* Write the byte in the transceiver data register */
    while (SERCOM5_SPI_IsBusy()){}
    
    SERCOM5_SPI_Write(data, length);

    while (SERCOM5_SPI_IsBusy()){}

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



void trx_reg_write(uint16_t addr, uint8_t data)
{
    uint8_t temp1 = 0U;
    uint16_t temp = 0U,temp2 = 0U;
    pal_trx_irq_dis();
	/* Prepare the command byte */
	addr |= WRITE_ACCESS_COMMAND;

    temp1 = addr>>8U; //rsh
    temp2 = addr<<8U; //lsh
    temp = temp2 | temp1;

	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();

	/* Send the Read command byte */
    while (SERCOM5_SPI_IsBusy()){} 
    SERCOM5_SPI_Write(&temp, 2);
    
    /* Write the byte in the transceiver data register */
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Write(&data, 1);

    while (SERCOM5_SPI_IsBusy()){}

	/* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();
    pal_trx_irq_en();
}

uint8_t trx_reg_read(uint16_t addr)
{

	uint8_t register_value = 0U;
    uint8_t temp1 = 0U;
    uint16_t temp = 0U,temp2 = 0U;
    pal_trx_irq_dis();
	/* Prepare the command byte */
	addr |= READ_ACCESS_COMMAND;

    temp1 = addr>>8U; //rsh
    temp2 = addr<<8U; //lsh
    temp = temp2 | temp1;
	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();
    /* Send the Read command byte */ 
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Write(&temp, 2);

   /* Read the byte from the transceiver data register */
    while (SERCOM5_SPI_IsBusy())
    {

    }

    
    SERCOM5_SPI_Read(&register_value, 1);

    while(SERCOM5_SPI_IsBusy()){}
//delayms(5);
	/* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();
pal_trx_irq_en();
	return register_value;
}


void trx_reg_bit_write(uint16_t addr, uint8_t mask, uint8_t pos, uint8_t value)
{ 
  uint8_t reg;
  reg = trx_reg_read(addr);
  reg = ((reg & ~mask) | (((value) << pos) & mask));
  trx_reg_write(addr, reg);
}


uint8_t trx_reg_bit_read(uint16_t addr, uint8_t mask, uint8_t pos)
{
  uint8_t reg;
  reg = trx_reg_read(addr);
  reg = (reg & mask) >> (pos); 
  return reg;
}




void trx_frame_write(uint8_t* buf, uint8_t length)
{
	uint8_t temp = 0U;
pal_trx_irq_dis();
	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();
	temp |= WRITE_ACCESS_COMMAND; //TRX_CMD_FW;

    while (SERCOM5_SPI_IsBusy()){}
    /* Send the command byte */
    SERCOM5_SPI_Write(&temp, 1);
    while (SERCOM5_SPI_IsBusy()){}
    /* Write into the buffer */
    SERCOM5_SPI_Write(buf, length);
    
    while (SERCOM5_SPI_IsBusy()){}

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

	temp = READ_ACCESS_COMMAND; //TRX_CMD_FR;

    while (SERCOM5_SPI_IsBusy()){}
    /* Send the command byte */
    SERCOM5_SPI_Write(&temp, 1);
    while (SERCOM5_SPI_IsBusy()){}

    SERCOM5_SPI_Read(buf, length);
    
    while (SERCOM5_SPI_IsBusy()){}
 
	/* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();
pal_trx_irq_en();
}

void trx_sram_write(uint8_t addr, uint8_t *data, uint8_t length)
{
    uint8_t temp;

	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();
    
    temp = 1; //TRX_CMD_SW;

    /* Send the command byte */
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Write(&temp, 1);
    
	/* Send the address from which the write operation should start */    
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Write(&addr, 1);

    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Write(data, length);
    
    while (SERCOM5_SPI_IsBusy()){}
    
    /* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();
    
}

void trx_sram_read(uint8_t addr, uint8_t *data, uint8_t length)
{
	uint16_t temp;

	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();
    
    temp = 1; //TRX_CMD_SR;

     /* Send the command byte */   
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Write(&temp, 1);
        
	/* Send the address from which the read operation should start */
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Write(&addr, 1);
    
    /* Upload the received byte in the user provided location */
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Read(data, length);
    
    while (SERCOM5_SPI_IsBusy()){}
    
    /* Stop the SPI transaction by setting SEL high */
    SPI_SS_Set();
    
}
