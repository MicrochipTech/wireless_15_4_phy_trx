

/* === INCLUDES ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "../../../pal/inc/pal.h"
#include "../../../phy/inc/phy.h"
#include "../../../phy/inc/ieee_phy_const.h"
#include "../../at86rf/inc/phy_pib.h"
#include "../../at86rf/inc/phy_irq_handler.h"
#include "../../at86rf/inc/at86rf.h"
#include "config/default/driver/IEEE_802154_PHY/resources/buffer/inc/bmm.h"
#include "config/default/driver/IEEE_802154_PHY/resources/queue/inc/qmm.h"
#include "../../at86rf/inc/phy_rx.h"
#include "../../at86rf/inc/phy_tx.h"
#include "../../../phy/inc/phy_constants.h"
#include "../../../phy/at86rf/inc/phy_internal.h"
#include "../../../phy/at86rf/inc/phy_trx_reg_access.h"
#include "../../../phy/at86rf/inc/phy_internal.h"
#include"definitions.h"


/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

 /* Value used for checking proper locking of PLL during switch from
 * TRX_PFF to PLL_ON.
 */
#define PLL_LOCK_ATTEMPTS           (3)

/* === GLOBALS ============================================================= */

/*
 * PHY Information Base
 */
phy_info_t phy_info;

/*
 * TAL PIBs
 */
tal_pib_t tal_pib;

/*
 * Global TAL variables
 * These variables are only to be used by the TAL internally.
 */

/**
 * Current state of the TAL state machine.
 * \ingroup group_tal_state_machine_233
 */
//tal_state_t tal_state;

/**
 * Current state of the transceiver.
 * \ingroup group_tal_state_machine_233
 */
//tal_trx_status_t tal_trx_status;

/**
 * Indicates if the transceiver needs to switch on its receiver by tal_task(),
 * because it could not be switched on due to buffer shortage.
 * \ingroup group_tal_state_machine_233
 */
//bool tal_rx_on_required;

/**
 * Pointer to the 15.4 frame created by the TAL to be handed over
 * to the transceiver.
 */
uint8_t *tal_frame_to_tx;

/**
 * Pointer to receive buffer that can be used to upload a frame from the trx.
 */
buffer_t *tal_rx_buffer = NULL;

/**
 * Queue that contains all frames that are uploaded from the trx, but have not
 * be processed by the MCL yet.
 */
queue_t tal_incoming_frame_queue;

/**
 * Frame pointer for the frame structure provided by the MCL.
 */
PHY_FrameInfo_t *mac_frame_ptr;

/* Last frame length for IFS handling. */
//uint8_t last_frame_length;

/* Flag indicating awake end irq at successful wake-up from sleep. */
//volatile bool tal_awake_end_flag;




/* === PROTOTYPES ========================================================== */

static void switch_pll_on(void);

#ifdef ENABLE_FTN_PLL_CALIBRATION
static void handle_ftn_pll_calibration(void);

#endif  /* ENABLE_FTN_PLL_CALIBRATION */

/* === IMPLEMENTATION ====================================================== */
/* TODO:  Include other files here if needed. */
/*
 * \brief TAL task handling
 *
 * This function
 * - Checks and allocates the receive buffer.
 * - Processes the TAL incoming frame queue.
 * - Implements the TAL state machine.
 */
void PHY_TaskHandler(void)
{  
      /* Check if the receiver needs to be switched on. */
	if (phy_info.tal_rx_on_required && ((phy_info.tal_state == PHY_IDLE) || (phy_info.tal_state == PHY_TX_DONE)))
         {
		/* Check if a receive buffer has not been available before. */
		if (tal_rx_buffer == NULL) {
			tal_rx_buffer = bmm_buffer_alloc(LARGE_BUFFER_SIZE);
		}

		/* Check if buffer could be allocated */
		if (NULL != tal_rx_buffer) {
			/*
			 * Note:
			 * This flag needs to be reset BEFORE the received is
			 * switched on.
			 */
			phy_info.tal_rx_on_required = false;

#ifdef PROMISCUOUS_MODE
			if (tal_pib.PromiscuousMode) {
                do
            {
                (void)set_trx_state(CMD_RX_ON);
            }while (tal_get_trx_status() != RX_ON);
				
			} else {
				do
            {
                (void)set_trx_state(CMD_RX_AACK_ON);
            }while (tal_get_trx_status() != RX_AACK_ON);
			}

#else   /* Normal operation */
			
            do
            {
                (void)set_trx_state(CMD_RX_AACK_ON);
            }while (tal_get_trx_status() != RX_AACK_ON);
#endif
		}
	 else {
        
		/* no free buffer is available; try next time again */
		}
	}
	/*
	 * If the transceiver has received a frame and it has been placed
	 * into the queue of the TAL, the frame needs to be processed further.
	 */
	if (tal_incoming_frame_queue.size > 0U) {
		buffer_t *rx_frame;

		/* Check if there are any pending data in the
		 * incoming_frame_queue. */
		rx_frame = qmm_queue_remove(&tal_incoming_frame_queue, NULL);
		if (NULL != rx_frame) {
			process_incoming_frame(rx_frame);
		}
	}

	/* Handle the TAL state machines */
	switch (phy_info.tal_state) {


	case PHY_IDLE:
        
        break;
	/* Do nothing, but fall through... */


	case PHY_TX_DONE:
		tx_done_handling(); /* see tal_tx.c */
		break;


	case PHY_ED_DONE:
		ed_scan_done();
		break;
        

	default:
        /* Nothing to do */
		break;
  }	
} /* PHY_TaskHandler() */

/* \brief TAL Task handling 
 * 
 * \This function handles the transceiver interrupt
 *
 */

 void TAL_TaskHandler(void)
 {
	    trx_irq_handler_cb();
 }


/*
 * \brief Sets transceiver state
 *
 * \param trx_cmd needs to be one of the trx commands
 *
 * \return current trx state
 */
tal_trx_status_t set_trx_state(trx_cmd_t trx_cmd)
{
    
	if (phy_info.tal_trx_status == TRX_SLEEP) {
		/*
		 * Since the wake-up procedure relies on the Awake IRQ and
		 * the global interrupts may be disabled at this point of time,
		 * we need to make sure that the global interrupts are enabled
		 * during wake-up procedure.
		 * Once the TRX is awake, the original state of the global
		 * interrupts
		 * will be restored.
		 */
		/* Reset wake-up interrupt flag. */
		if (CMD_SLEEP == trx_cmd) {
			return TRX_SLEEP;
		}

		phy_info.tal_awake_end_flag = false;
        
		/* Set callback function for the awake interrupt. */
        tal_trx_wakeup();
		
#if (ANTENNA_DIVERSITY == 1)
		/* Enable antenna diversity. */
		trx_reg_bit_write(SR_ANT_EXT_SW_EN, ANT_EXT_SW_ENABLE);
#endif

#ifdef EXT_RF_FRONT_END_CTRL
		/* Enable RF front end control */
		trx_reg_bit_write(SR_PA_EXT_EN, PA_EXT_ENABLE);
#endif

        phy_info.tal_trx_status = TRX_OFF;
        
		if ((trx_cmd == CMD_TRX_OFF) ||
				(trx_cmd == CMD_FORCE_TRX_OFF)) {
			return TRX_OFF;
		}
	}

#ifdef ENABLE_DEEP_SLEEP
	else if (phy_info.tal_trx_status == TRX_DEEP_SLEEP) {
		if (CMD_DEEP_SLEEP == trx_cmd) {
			return TRX_DEEP_SLEEP;
		}
        
        tal_trx_wakeup();

		/* Check if trx has left deep sleep. */
		tal_trx_status_t trx_state;
		do {
			trx_state = trx_reg_read(
					RG_TRX_STATUS);
		} while (trx_state != TRX_OFF);
        
		phy_info.tal_trx_status = TRX_OFF;

		/* Using deep sleep, the transceiver's registers need to be
		 * restored. */
		trx_config();

		/*
		 * Write all PIB values to the transceiver
		 * that are needed by the transceiver itself.
		 */
		write_all_tal_pib_to_trx(); /* implementation can be found in
		                             *'tal_pib.c' */
		if ((trx_cmd == CMD_TRX_OFF) ||
				(trx_cmd == CMD_FORCE_TRX_OFF)) {
			return TRX_OFF;
		}
	}
#endif

	switch (trx_cmd) { /* requested state */
	case CMD_SLEEP:
#ifdef ENABLE_DEEP_SLEEP
	/* Fall through. */
	case CMD_DEEP_SLEEP:
#endif
        
        trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_FORCE_TRX_OFF);
#if (ANTENNA_DIVERSITY == 1)

		/*
		 *  Disable antenna diversity: to reduce the power consumption
		 * or
		 *  avoid leakage current of an external RF switch during SLEEP.
		 */
		trx_reg_bit_write(SR_ANT_EXT_SW_EN, ANT_EXT_SW_DISABLE);
#endif
#ifdef EXT_RF_FRONT_END_CTRL
		/* Disable RF front end control */
		trx_reg_bit_write(SR_PA_EXT_EN, PA_EXT_DISABLE);
#endif
        /* Clear existing interrupts */
		(void)trx_reg_read(RG_IRQ_STATUS);

#ifdef ENABLE_DEEP_SLEEP
		if (trx_cmd == CMD_DEEP_SLEEP) {
			trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_PREP_DEEP_SLEEP);
			phy_info.tal_trx_status = TRX_DEEP_SLEEP;
		} else {
			/*
			 * Enable Awake_end interrupt.
			 * This is used for save wake-up from sleep later.
			 */
			trx_reg_write(RG_IRQ_MASK, (uint8_t)TRX_IRQ_4_CCA_ED_DONE);
			phy_info.tal_trx_status = TRX_SLEEP;
		}

#else

		/*
		 * Enable Awake_end interrupt.
		 * This is used for save wake-up from sleep later.
		 */
		trx_reg_write(RG_IRQ_MASK, (uint8_t)TRX_IRQ_4_CCA_ED_DONE);
		phy_info.tal_trx_status = TRX_SLEEP;
#endif

		trx_delay_micros(1);
        TRX_SLP_TR_HIGH();
        trx_delay_micros(TRX_OFF_TO_SLEEP_TIME_CLKM_CYCLES);
        return phy_info.tal_trx_status;
		break;
        

	case CMD_TRX_OFF:
		switch (phy_info.tal_trx_status) {
		case TRX_OFF:
			break;

		default:
			trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_TRX_OFF);
			trx_delay_micros(1);
			break;
		}
		break;

	case CMD_FORCE_TRX_OFF:
		switch (phy_info.tal_trx_status) {
		case TRX_OFF:
			break;

		default:
			trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_FORCE_TRX_OFF);
			trx_delay_micros(1);
			break;
		}
		break;

	case CMD_PLL_ON:
		switch (phy_info.tal_trx_status) {
            
		case PLL_ON:
			break;

		case TRX_OFF:

			switch_pll_on();
			break;

		case RX_ON:
		case RX_AACK_ON:
		case TX_ARET_ON:
            trx_reg_write(RG_TRX_STATE, (uint8_t)TRX_OFF);
            trx_delay_micros(1);
            switch_pll_on();
			break;

		case BUSY_RX:
		case BUSY_TX:
		case BUSY_RX_AACK:
		case BUSY_TX_ARET:
			/* do nothing if trx is busy */
			break;

		default:
            /* Nothing to do */
			break;
		}
		break;

	case CMD_FORCE_PLL_ON:
		switch (phy_info.tal_trx_status) {
		case TRX_OFF:
			switch_pll_on();
			break;

		case PLL_ON:
			break;

		default:
			trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_FORCE_PLL_ON);
			break;
		}
		break;

	case CMD_RX_ON:
		switch (phy_info.tal_trx_status) {
		case RX_ON:
            break;
		case PLL_ON:
		case RX_AACK_ON:
		case TX_ARET_ON:
            trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_TRX_OFF);
            //switch_pll_on();
            //trx_delay_micros(TRX_OFF_TO_PLL_ON_TIME_US);
            do{
                trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_RX_ON);
            }while((((uint8_t)tal_get_trx_status())& 0x1fU) != ((uint8_t)RX_ON));			
			break;

		case TRX_OFF:
			//switch_pll_on();
            //trx_delay_micros(TRX_OFF_TO_PLL_ON_TIME_US);
            do{
                trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_RX_ON);
             }while((((uint8_t)tal_get_trx_status())& 0x1fU) != ((uint8_t)RX_ON));
			
			break;

		case BUSY_RX:
		case BUSY_TX:
		case BUSY_RX_AACK:
		case BUSY_TX_ARET:
			/* do nothing if trx is busy */
			break;

		default:
            /* Nothing to do */
			break;
		}
		break;

	case CMD_RX_AACK_ON:
		switch (phy_info.tal_trx_status) {
		case RX_AACK_ON:
            break;
		case TX_ARET_ON:
		case PLL_ON:
		case RX_ON:
            trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_TRX_OFF);
            //switch_pll_on();
            do{
               trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_RX_AACK_ON);
            }while((((uint8_t)tal_get_trx_status())& 0x1fU) != ((uint8_t)RX_AACK_ON));

			break;

		case TRX_OFF:
			//switch_pll_on(); /* state change from TRX_OFF to
			//                  * RX_AACK_ON can be done directly, too
			//                  **/
            //trx_delay_micros(TRX_OFF_TO_PLL_ON_TIME_US);
			do{
               trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_RX_AACK_ON);
            }while((((uint8_t)tal_get_trx_status())& 0x1fU) != ((uint8_t)RX_AACK_ON));
			
			break;

		case BUSY_RX:
        case BUSY_RX_AACK:  
        case BUSY_TX:
		case BUSY_TX_ARET:
			/* do nothing if trx is busy */
			break;

		default:
            /* Nothing to do */
			break;
		}
		break;

	case CMD_TX_ARET_ON:
		switch (phy_info.tal_trx_status) {
		case TX_ARET_ON:
			break;

		case PLL_ON:
		case RX_ON:
		case RX_AACK_ON:
            trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_TRX_OFF);
            //switch_pll_on();
            //trx_delay_micros(TRX_OFF_TO_PLL_ON_TIME_US);           
            trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_TX_ARET_ON);
			
			break;

		case TRX_OFF:
			//switch_pll_on(); /* state change from TRX_OFF to
			//                  * TX_ARET_ON can be done directly, too
			//                  **/
            //trx_delay_micros(TRX_OFF_TO_PLL_ON_TIME_US);
			trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_TX_ARET_ON);
			break;

		case BUSY_RX:
		case BUSY_TX:
		case BUSY_RX_AACK:
		case BUSY_TX_ARET:
			/* do nothing if trx is busy */
			break;

		default:
            /* Nothing to do */
			break;
		}
		break;

	default:
        /* Nothing to do */
		break;
	}
	uint8_t count = 0;

	do {
		count++;
		uint8_t temp = (0x1FU) & (trx_reg_read(RG_TRX_STATUS));
		phy_info.tal_trx_status = (tal_trx_status_t)temp;
		if (count == 100)
		{
			break;
		}
	} while (phy_info.tal_trx_status == STATE_TRANSITION_IN_PROGRESS);
    
	return phy_info.tal_trx_status;
} /* set_trx_state() */

/**
 * \brief Switches the PLL on
 * \ingroup group_tal_state_machine
 */
static void switch_pll_on(void)
{
    
	uint32_t start_time;
	uint32_t current_time;

	/* Check if trx is in TRX_OFF; only from PLL_ON the following procedure
	 * is applicable */
	if (trx_reg_bit_read(SR_TRX_STATUS) != (uint8_t)TRX_OFF) {

		return;
	}

	/* Clear all pending trx interrupts */
	(void)trx_reg_read(RG_IRQ_STATUS);
	/* Get current IRQ mask */
	uint8_t trx_irq_mask = trx_reg_read(RG_IRQ_MASK);
	/* Enable transceiver's PLL lock interrupt */
	trx_reg_write(RG_IRQ_MASK, (uint8_t)TRX_IRQ_0_PLL_LOCK);
	/* Disable trx interrupt handling */
    EIC_InterruptDisable(EIC_PIN_0); 

	/* Switch PLL on */
	trx_reg_write(RG_TRX_STATE, (uint8_t)CMD_PLL_ON);
	start_time = PAL_GetCurrentTime();

	/* Wait for transceiver interrupt: check for IRQ line */
	while ((bool)IRQ_Get() == false) {
		/* Handle errata "potential long PLL settling duration". */
		current_time = PAL_GetCurrentTime();
		if (pal_sub_time_us(current_time,
				start_time) > PLL_LOCK_DURATION_MAX_US) {
			uint8_t reg_value;

			reg_value = trx_reg_read(RG_PLL_CF);
			if ((bool)(reg_value & (0x01U))) {
				reg_value &= 0xFEU;
			} else {
				reg_value |= 0x01U;
			}

			trx_reg_write(RG_PLL_CF, reg_value);
			start_time = PAL_GetCurrentTime();
		}

		/* Wait until trx line has been raised. */
	}

	/* Clear PLL lock interrupt at trx */
	(void)trx_reg_read(RG_IRQ_STATUS);
	/* Clear MCU's interrupt flag */
    EIC_REGS->EIC_INTFLAG = (1UL << EIC_PIN_0);
	/* Enable trx interrupt handling again */
    EIC_InterruptEnable(EIC_PIN_0);
	/* Restore transceiver's interrupt mask. */
	trx_reg_write(RG_IRQ_MASK, trx_irq_mask);
}


PHY_TrxStatus_t PHY_GetTrxStatus(void)
{
    tal_trx_status_t trx_status = tal_get_trx_status();
    PHY_TrxStatus_t trx_state;
    
    switch(trx_status)
    {
        case TRX_OFF:
        case P_ON:
        case PLL_ON:
            trx_state = PHY_TRX_OFF;
            break;
            
        case RX_ON:
        case RX_AACK_ON:
            trx_state = PHY_RX_ON;
            break;
            
        case TX_ARET_ON:
            trx_state = PHY_TX_ON;
            break;
            
        case BUSY_RX:
        case BUSY_RX_AACK:
            trx_state = PHY_BUSY_RX;
            break;
            
        case BUSY_TX:
        case BUSY_TX_ARET:
            trx_state = PHY_BUSY_TX;
			break;
            
        case TRX_SLEEP:
            trx_state = PHY_TRX_SLEEP;
            break;
            
        case PREP_DEEP_SLEEP:
            trx_state = PHY_TRX_DEEP_SLEEP;
            break;
            
        default:
            trx_state = PHY_TRX_OFF;
            break;
    }
    
    return trx_state;
}


uint32_t PHY_GetSWVersion(void)
{
    uint32_t phyVersion;
    
    phyVersion = phy_info.phyVersion;
    
    return phyVersion;
}


/**
 * @brief Conversion of symbols to microseconds
 */
uint32_t tal_convert_symbols_to_us_def(uint32_t symbols)
{
	return (PHY_CONVERT_SYMBOLS_TO_US(symbols));
}

/**
 * @brief Conversion of microseconds to symbols
 */
uint32_t tal_convert_us_to_symbols_def(uint32_t time_)
{
	return (PHY_CONVERT_US_TO_SYMBOLS(time_));
}

void trx_delay_loop(void *hw, uint32_t cycles)
{
    (void) hw;
    (void) cycles;

    /* 
     * SUBS instruction - 1 cycle 
     * BHI  instruction - 2 cycles if taken; 1 if not taken
     * 
     * In most case: 3 cycles is the loop time
     */
    
    __asm__(
            ".syntax unified\n"
            "__dly:\n"
            "subs r1, r1, #1\n"
            "bhi __dly\n"
            ".syntax divided"
            );
}

void trx_delay_millis(uint32_t ms)
{
    uint32_t cycles = ((uint32_t)PAL_CPU_CLOCK_FREQUENCY / 1000UL) * ms;
    cycles /= 3UL;
    trx_delay_loop(NULL, cycles);
}

void trx_delay_micros(uint32_t us)
{
    uint32_t cycles = ((uint32_t)PAL_CPU_CLOCK_FREQUENCY / 1000000UL) * us;
    cycles /= 3UL;
    trx_delay_loop(NULL, cycles); 
}


/* EOF */
