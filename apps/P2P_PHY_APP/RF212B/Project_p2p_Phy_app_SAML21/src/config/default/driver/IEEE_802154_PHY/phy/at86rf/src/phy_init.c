

/* === INCLUDES ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "../../../phy/inc/phy.h"
#include "config/default/driver/IEEE_802154_PHY/resources/buffer/inc/bmm.h"
#include "config/default/driver/IEEE_802154_PHY/resources/queue/inc/qmm.h"

#include "../../../phy/inc/ieee_phy_const.h"
#include "../../at86rf/inc/phy_pib.h"
#include "../../at86rf/inc/phy_irq_handler.h"
#include "../../at86rf/inc/phy_tx.h"
#include "../../at86rf/inc/phy_rx.h"


#include "../../../pal/inc/pal.h"

#include "../../at86rf/inc/phy_internal.h"
#include "../../../phy/inc/phy_constants.h"
#include "../../at86rf/inc/at86rf.h"
#include "../../../phy/at86rf/inc/phy_trx_reg_access.h"







/**
 * \addtogroup group_tal_init_233
 * @{
 */

/* === MACROS ============================================================== */

/* Value in us used for delay between poll attempts for transceiver access. */
#define TRX_POLL_WAIT_TIME_US       (100U)

/* Ratio between max time of TR1 / transceiver poll delay */
#define P_ON_TO_CLKM_ATTEMPTS       ((uint8_t) \
	(P_ON_TO_CLKM_AVAILABLE_MAX_US / TRX_POLL_WAIT_TIME_US))

/* Ratio between max time of TR2 / transceiver poll delay */
#define SLEEP_TO_TRX_OFF_ATTEMPTS   ((uint8_t) \
	(SLEEP_TO_TRX_OFF_MAX_US / TRX_POLL_WAIT_TIME_US))



#define VECTOR_TABLE_SIZE 128

/* === GLOBALS ============================================================= */




TimerId_t TAL_RETRY_TIMER;

/* === PROTOTYPES ========================================================== */

static PHY_Retval_t trx_init(void);
static void trx_config(void);
static PHY_Retval_t trx_reset(void);
static PHY_Retval_t internal_tal_reset(bool set_default_pib);

/**
 * \brief Initializes all timers used by the TAL module by assigning id's to
 * each of them
 */
static PHY_Retval_t tal_timer_init(void);

/**
 * \brief Stops all initialized TAL timers
 */
static void tal_timers_stop(void);



/* ! @} */

/* === IMPLEMENTATION ====================================================== */

/*
 * \brief Initializes the TAL
 *
 * This function is called to initialize the TAL. The transceiver is
 * initialized, the TAL PIBs are set to their default values, and the TAL state
 * machine is set to TAL_IDLE state.
 *
 * \return PHY_SUCCESS  if the transceiver state is changed to TRX_OFF and the
 *                 current device part number and version number are correct;
 *         PHY_FAILURE otherwise
 */
PHY_Retval_t PHY_Init(void)
{
  
	/* Init the PAL and by this means also the transceiver interface */
	if (PAL_Init() != PAL_SUCCESS) {
		return PHY_FAILURE;
	}

	if (trx_init() != PHY_SUCCESS) {
		return PHY_FAILURE;
	}
    

	if (tal_timer_init() != PHY_SUCCESS) {
		return PHY_FAILURE;
	}

	/*
	 * Do the reset stuff.
	 * Set the default PIBs.
	 * Generate random seed.
	 */
	if (internal_tal_reset(true) != PHY_SUCCESS) {
		return PHY_FAILURE;
	}

#ifndef DISABLE_IEEE_ADDR_CHECK
	/* Check if a valid IEEE address is available. */

	/*
	 * This while loop is on purpose, since just in the
	 * rare case that such an address is randomly
	 * generated again, we must repeat this.
	 */
	uint64_t invalid_ieee_address;
	(void)memset((uint8_t *)&invalid_ieee_address, 0xFF,
			sizeof(invalid_ieee_address));
	while ((tal_pib.IeeeAddress == 0x0000000000000000U) ||
			(tal_pib.IeeeAddress == invalid_ieee_address)) {
		/*
		 * In case no valid IEEE address is available, a random
		 * IEEE address will be generated to be able to run the
		 * applications for demonstration purposes.
		 * In production code this can be omitted.
		 */

		/*
		 * The proper seed for function rand() has already been
		 * generated
		 * in function tal_generate_rand_seed().
		 */
		uint8_t *ptr_pib = (uint8_t *)&(tal_pib.IeeeAddress);

		for (uint8_t i = 0; i < 8; i++) {
			*ptr_pib++ = (uint8_t)rand();

			/*
			 * Note:
			 * Even if casting the 16 bit rand value back to 8 bit,
			 * and running the loop 8 timers (instead of only 4
			 * times)
			 * may look cumbersome, it turns out that the code gets
			 * smaller using 8-bit here.
			 * And timing is not an issue at this place...
			 */
		}
	}
#endif  /* #ifndef DISABLE_IEEE_ADDR_CHECK */
	/*
	 * Configure interrupt handling.
	 * Install a handler for the transceiver interrupt.
	 */
    EIC_CallbackRegister(EIC_PIN_14, EIC_interrupt_cb, 0);
	
	pal_trx_irq_en(); /* Enable transceiver main interrupt. */


	/* Initialize the buffer management module and get a buffer to store
	 * received frames. */
	bmm_buffer_init();
	tal_rx_buffer = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

	if (tal_rx_buffer == NULL) {
		return PHY_FAILURE;
	}

    
		/* Init incoming frame queue */
#ifdef ENABLE_QUEUE_CAPACITY
	qmm_queue_init(&tal_incoming_frame_queue,
			PHY_INCOMING_FRAME_QUEUE_CAPACITY);
#else
	qmm_queue_init(&tal_incoming_frame_queue);
#endif  /* ENABLE_QUEUE_CAPACITY */
    
    tfa_init();
    
    phy_info.phyVersion = PHY_VERSION_VALUE;
    
	return PHY_SUCCESS;
} /* tal_init() */

/**
 * \brief Initializes the transceiver
 *
 * This function is called to initialize the transceiver.
 *
 * \return PHY_SUCCESS  if the transceiver state is changed to TRX_OFF and the
 *                 current device part number and version number are correct;
 *         PHY_FAILURE otherwise
 */
static PHY_Retval_t trx_init(void)
{       
	volatile tal_trx_status_t trx_status;
	uint8_t poll_counter = 0;

    
    /* Wait typical time of timer TR1. */
    trx_delay_micros(P_ON_TO_CLKM_AVAILABLE_TYP_US);
    
	/* make sure SPI is working properly */
	/*    while ((tal_trx_status_t)trx_bit_read(SR_TRX_STATUS) != P_ON); */


	/* Apply reset pulse Ensure control lines have correct levels */
	TRX_SEL_HIGH();
	TRX_RST_LOW();
	TRX_SLP_TR_LOW();
    trx_delay_micros(RST_PULSE_WIDTH_US);
	TRX_RST_HIGH();
	
	/* Wait typical time of timer TR13. */
	trx_delay_micros(30);

	trx_status = (tal_trx_status_t)trx_reg_bit_read(SR_TRX_STATUS);

	/* Dummy assignment, to avoid compiler warning */
	trx_status = trx_status;

#if !(defined FPGA_EMULATION)
	do {
		/* Wait not more than max. value of TR1. */
		if (poll_counter == P_ON_TO_CLKM_ATTEMPTS) {

            return 	PHY_FAILURE;
		}
		/* Wait a short time interval. */
        trx_delay_micros(TRX_POLL_WAIT_TIME_US);
		poll_counter++;

		/* Check if AT86RF212B is connected; omit manufacturer id check
		**/
	} while (trx_reg_read(RG_PART_NUM) != PART_NUM_AT86RF212B);
#endif  /* !defined FPGA_EMULATION */

	/* Verify that TRX_OFF can be written */

    trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_TRX_OFF);

	/* Verify that the trx has reached TRX_OFF. */
	tal_trx_status_t trx_status_check;
	trx_status_check = (tal_trx_status_t)trx_reg_bit_read(SR_TRX_STATUS);
	if (trx_status_check != TRX_OFF) {
		return PHY_FAILURE;
    }


	trx_reg_write(RG_IRQ_MASK, TRX_NO_IRQ);
	phy_info.tal_trx_status = TRX_OFF;


	return PHY_SUCCESS;
}

/**
 * \brief Internal TAL reset function
 *
 * \param set_default_pib Defines whether PIB values need to be set
 *                        to its default values
 *
 * \return PHY_SUCCESS  if the transceiver state is changed to TRX_OFF and the
 *                 current device part number and version number are correct;
 *         PHY_PHY_FAILURE otherwise
 */
static PHY_Retval_t internal_tal_reset(bool set_default_pib)
{
	if (trx_reset() != PHY_SUCCESS) {
		return PHY_FAILURE;
	}

	/*
	 * Generate a seed for the random number generator in function rand().
	 * This is required (for example) as seed for the CSMA-CA algorithm.
	 */
	tal_generate_rand_seed();


	/* Configure the transceiver register values. */
	trx_config();

	if (set_default_pib) {
		/* Set the default PIB values */
		init_tal_pib(); /* implementation can be found in 'tal_pib.c' */
	} else {
		/* nothing to do - the current TAL PIB attribute values are used
		**/
	}

	/*
	 * Write all PIB values to the transceiver
	 * that are needed by the transceiver itself.
	 */
	write_all_tal_pib_to_trx(); /* implementation can be found in
	                             *'tal_pib.c' */

	/* Reset TAL variables. */
	phy_info.tal_state = PHY_IDLE;





	phy_info.tal_rx_on_required = false;

	return PHY_SUCCESS;
}

/**
 * \brief Configures the transceiver
 *
 * This function is called to configure the transceiver after reset.
 */
void trx_config(void)
{


    
	/* Set pin driver strength */
	trx_reg_bit_write(SR_PAD_IO_CLKM, PAD_CLKM_2_MA);
    trx_reg_bit_write(SR_CLKM_SHA_SEL, CLKM_SHA_DISABLE); 

#ifndef SW_CONTROLLED_CSMA
	/*
	 * After we have initialized a proper seed for rand(),
	 * the transceiver's CSMA seed can be initialized.
	 * It needs to be assured that a seed for function rand()
	 * had been generated before.
	 */

	/*
	 * Init the SEED value of the CSMA backoff algorithm.
	 */
	uint16_t rand_value = (uint16_t)rand();
    trx_reg_write(RG_CSMA_SEED_0, (uint8_t)rand_value); 
	trx_reg_bit_write(SR_CSMA_SEED_1, (uint8_t)(rand_value >> 8));

//	/*
//	 * To make sure that the CSMA seed is properly set within the
//	 * transceiver,
//	 * put the trx to sleep briefly and wake it up again.
//	 */
//	PHY_TrxSleep(SLEEP_MODE_1);
//	PHY_TrxWakeup();
#endif


	trx_reg_bit_write(SR_AACK_SET_PD, SET_PD); /* ACKs for data requests,
	                                       * indicate pending data */
	trx_reg_bit_write(SR_RX_SAFE_MODE, RX_SAFE_MODE_ENABLE); /* Enable
	                                                     * buffer
	                                                     * protection
	                                                     * mode */
	trx_reg_bit_write(SR_IRQ_MASK_MODE, IRQ_MASK_MODE_ON); /* Enable poll
	                                                    * mode */	                                                   
	trx_reg_write(RG_IRQ_MASK, (uint8_t)TRX_IRQ_DEFAULT); /* The TRX_END
	                                              * interrupt of the
	                                              * transceiver is
	                                              * enabled. */

    

#if (ANTENNA_DIVERSITY == 1)
	trx_reg_bit_write(SR_ANT_EXT_SW_EN, ANT_EXT_SW_ENABLE); /* Enable
	                                                     * antenna
	                                                     * diversity. */
#if (ANTENNA_DEFAULT != ANT_CTRL_1)
	trx_reg_bit_write(SR_ANT_CTRL, ANTENNA_DEFAULT);
#endif  /* ANTENNA_DEFAULT */
#endif  /* ANTENNA_DIVERSITY */


#ifdef CCA_ED_THRESHOLD

	/*
	 * Set CCA ED Threshold to other value than standard register due to
	 * board specific loss (see pal_config.h). */
	trx_reg_bit_write(SR_CCA_ED_THRES, CCA_ED_THRESHOLD);
#endif

#ifndef ENABLE_RX_OVERRIDE
	trx_reg_bit_write(SR_RX_OVERRIDE, RXO_ENABLE);
#else

	/*
	 * Allow overriding of a 'weak' frame when another frame with stronger
	 * signal power arrives during the reception of this 'weak' frame.
	 */
	trx_reg_bit_write(SR_RX_OVERRIDE, ENABLE_RX_OVERRIDE);
#endif /* ENABLE_RX_OVERRIDE */
}

/**
 * \brief Reset transceiver
 *
 * \return PHY_SUCCESS  if the transceiver state is changed to TRX_OFF
 *         PHY_FAILURE otherwise
 */
static PHY_Retval_t trx_reset(void)
{
    tal_trx_status_t trx_status;
    uint8_t poll_counter = 0;
    /* trx might sleep, so wake it up */
	TRX_SLP_TR_LOW();
    trx_delay_micros(SLEEP_TO_TRX_OFF_TYP_US);
	/* Apply reset pulse */
	TRX_RST_LOW();
    trx_delay_micros(RST_PULSE_WIDTH_US);
	TRX_RST_HIGH();
    
    /* verify that trx has reached TRX_OFF */
	do {
		/* Wait a short time interval. */
        trx_delay_micros(TRX_POLL_WAIT_TIME_US);

		trx_status =(tal_trx_status_t)trx_reg_bit_read(SR_TRX_STATUS);

		/* Wait not more than max. value of TR2. */
		if (poll_counter == SLEEP_TO_TRX_OFF_ATTEMPTS) {

			return PHY_FAILURE;
		}

		poll_counter++;
	} while (trx_status != TRX_OFF);

	phy_info.tal_trx_status = TRX_OFF;


	return PHY_SUCCESS;
}

/*
 * \brief Resets TAL state machine and sets the default PIB values if requested
 *
 * \param set_default_pib Defines whether PIB values need to be set
 *                        to its default values
 *
 * \return PHY_SUCCESS  if the transceiver state is changed to TRX_OFF
 *         PHY_FAILURE otherwise
 */
PHY_Retval_t PHY_Reset(bool set_default_pib)
{
	/*
	 * Do the reset stuff.
	 * Set the default PIBs depending on the given parameter
	 * set_default_pib.
	 * Do NOT generate random seed again.
	 */
	if (internal_tal_reset(set_default_pib) != PHY_SUCCESS) {
		return PHY_FAILURE;
	}

	ENTER_CRITICAL_REGION();
	tal_timers_stop();
	LEAVE_CRITICAL_REGION();

	/* Clear TAL Incoming Frame queue and free used buffers. */
	while (tal_incoming_frame_queue.size > 0U) {
		buffer_t *frame = qmm_queue_remove(&tal_incoming_frame_queue,
				NULL);
		if (NULL != frame) {
			bmm_buffer_free(frame);
		}
	}
    tfa_reset(set_default_pib);
	/*
	 * Configure interrupt handling.
	 * Install a handler for the transceiver interrupt.
	 */
	EIC_CallbackRegister(EIC_PIN_14, EIC_interrupt_cb, 0);
	/* The pending transceiver interrupts on the microcontroller are
	 * cleared. */
	pal_trx_irq_en(); /* Enable transceiver main interrupt. */

	return PHY_SUCCESS;
}

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

void tal_generate_rand_seed(void)
{
	uint16_t seed = 0;
	uint8_t cur_random_val = 0;

	/* Ensure that PLL has locked and receive mode is reached. */
	tal_trx_status_t trx_state;
	do {
		trx_state = set_trx_state(CMD_RX_ON);
	} while (trx_state != RX_ON);

	/* Ensure that register bit RX_PDT_DIS is set to 0. */
	trx_reg_bit_write(SR_RX_PDT_DIS, RX_ENABLE);

	/*
	 * We need to disable TRX IRQs while generating random values in RX_ON,
	 * we do not want to receive frames at this point of time at all.
	 */
	EIC_InterruptDisable(EIC_PIN_14);

	/*
	 * The 16-bit random value is generated from various 2-bit random
	 * values.
	 */
	for (uint8_t i = 0; i < 8; i++) {
		/* Now we can safely read the 2-bit random number. */
		cur_random_val = trx_reg_bit_read(SR_RND_VALUE);
		seed = seed << 2;
		seed |= cur_random_val;
		trx_delay_micros(1); /* wait that the random value gets updated */
	}

	(void)set_trx_state(CMD_FORCE_TRX_OFF);

	/*
	 * Now we need to clear potential pending TRX IRQs and
	 * enable the TRX IRQs again.
	 */
	(void)trx_reg_read(RG_IRQ_STATUS);
	EIC_REGS->EIC_INTFLAG = (1UL << EIC_PIN_14);
	EIC_InterruptEnable(EIC_PIN_14);

	/* Set the seed for the random number generator. */
	srand(seed);
}

static PHY_Retval_t tal_timer_init(void)
{
    /* Timer used for Retransmission caused by Arbiter Abort failure*/
    if (PAL_SUCCESS != PAL_TimerGetId(&TAL_RETRY_TIMER)) {
		return PHY_FAILURE;
	}
#ifdef ENABLE_FTN_PLL_CALIBRATION
	if (PAL_SUCCESS != PAL_TimerGetId(&TAL_CALIBRATION)) {
		return PHY_FAILURE;
	}

#ifdef SW_CONTROLLED_CSMA
	if (PAL_SUCCESS != PAL_TimerGetId(&TAL_T_BOFF)) {
		return PHY_FAILURE;
	}
#endif

#else
#ifdef SW_CONTROLLED_CSMA
	if (PAL_SUCCESS != PAL_TimerGetId(&TAL_T_BOFF)) {
		return PHY_FAILURE;
	}
#endif
#endif  /* ENABLE_FTN_PLL_CALIBRATION */
	return PHY_SUCCESS;
}

static void tal_timers_stop(void)
{
#if (NUMBER_OF_PHY_TIMERS > 0)
    (void)PAL_TimerStop(TAL_RETRY_TIMER);

#ifdef ENABLE_FTN_PLL_CALIBRATION
	(void)PAL_TimerStop(TAL_CALIBRATION);
#ifdef SW_CONTROLLED_CSMA
	(void)PAL_TimerStop(TAL_T_BOFF);
#endif

#else
#ifdef SW_CONTROLLED_CSMA
	(void)PAL_TimerStop(TAL_T_BOFF);
#endif
#endif  /* ENABLE_FTN_PLL_CALIBRATION */
#endif /* (NUMBER_OF_TAL_TIMERS > 0)*/
}

void trx_irq_flag_clear(void)
{
    uint8_t trx_irq_cause;

	trx_irq_cause = trx_reg_read(RG_IRQ_STATUS);
    
    (void)trx_irq_cause;
}



    

/**************************/
/* EOF */
