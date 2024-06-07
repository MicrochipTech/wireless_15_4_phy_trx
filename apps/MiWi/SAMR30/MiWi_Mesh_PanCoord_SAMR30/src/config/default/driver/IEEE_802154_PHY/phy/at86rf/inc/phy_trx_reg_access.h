

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