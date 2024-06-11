

/* === INCLUDES ============================================================ */
#include "definitions.h"
#include "../../../pal/inc/pal.h"
#include "../../at86rf/inc/phy_trx_reg_access.h"


void trx_reg_write(uint8_t addr, uint8_t value)
{
    pal_trx_irq_dis();
	/* Prepare the command byte */
	addr |= WRITE_ACCESS_COMMAND;

	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();

	/* Send the Read command byte */
    while (SERCOM5_SPI_IsBusy()){} 
    SERCOM5_SPI_Write(&addr, 1);
    
    /* Write the byte in the transceiver data register */
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Write(&value, 1);

    while (SERCOM5_SPI_IsBusy()){}

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
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Write(&addr, 1);

   /* Read the byte from the transceiver data register */
    while (SERCOM5_SPI_IsBusy()){}
    SERCOM5_SPI_Read(&register_value, 1);

    while(SERCOM5_SPI_IsBusy()){}

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

	temp = TRX_CMD_FR;

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
    pal_trx_irq_dis();
    uint8_t temp;

	/* Start SPI transaction by pulling SEL low */
    SPI_SS_Clear();
    
    temp = TRX_CMD_SW;

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
    pal_trx_irq_en();
    
}
