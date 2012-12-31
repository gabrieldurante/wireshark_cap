/**
 * @file pal_irq.c
 *
 * @brief PAL IRQ functionality
 *
 * This file contains functions to initialize, enable, disable and install
 * handler for the transceiver interrupts.
 *
 * $Id: pal_irq.c 29088 2011-11-02 15:05:28Z v_prasad.anjangi $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2010, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === Includes ============================================================ */

#include <stdint.h>
#include "pal_types.h"
#include "return_val.h"
#include "pal.h"
#include "pal_irq.h"
#include "pal_boardtypes.h"
#include "pal_internal.h"

#if (BOARD_TYPE == RZ600_231)
/* === Types ============================================================== */

/* === Globals ============================================================= */

/*
 * Function pointers to store the callback function of
 * the transceiver interrupt
 */
static irq_handler_t irq_hdl_trx;


#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP)
/*
 * Function pointers to store the callback function of
 * the timestamp interrupt
 */
static irq_handler_t irq_hdl_trx_tstamp;
#endif

/* === Prototypes ========================================================== */


/* === Implementation ====================================================== */

/**
 * @brief Initializes the transceiver interrupts
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver interrupts
 *
 * @param trx_irq_cb Callback function for the given transceiver
 * interrupt
 */
void pal_trx_irq_init(FUNC_PTR trx_irq_cb)
{
    irq_hdl_trx = (irq_handler_t)trx_irq_cb;
    /*
     * Set the handler function.
     * The handler is set before enabling the interrupt to prepare for spurious
     * interrupts, that can pop up the moment they are enabled
     */
    volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[EXT_INT >> PORT_NUM];
    uint32_t mask = 1 << (EXT_INT & 0x1F);
    gpio_port->gfers = mask;
    gpio_port->imr0s = mask;
    gpio_port->imr1c = mask;
    gpio_port->ifrc = mask;
}

#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP)
/**
 * @brief Initializes the transceiver timestamp interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver timestamp interrupt
 *
 * @param trx_irq_cb Callback function for the transceiver timestamp
 * interrupt
 */
void pal_trx_irq_init_tstamp(FUNC_PTR trx_irq_cb)
{
    irq_hdl_trx_tstamp = (irq_handler_t)trx_irq_cb;
    /* Rising edge on DIG2 pin used to trigger IRQ */
    TC_CH2.ier = AVR32_TC_IER0_LDRAS_MASK ;
}
#endif  /* #if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP) */


#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP)
/**
 * @brief Function to read the timestamp
 */
void read_timestamp()
{
    /*Clearing the timestamp interrupt*/
    pal_trx_irq_flag_clr_tstamp();
    irq_hdl_trx_tstamp();
}
#endif  /* #if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP) */


/**
 * @brief ISR for transceiver's main interrupt
 */
#if (defined __GNUC__)
__attribute__((__interrupt__))
#elif (defined __ICCAVR32__)
__interrupt
#endif
void ext_int_isr(void)
{
    /*Clearing the RF interrupt*/
    pal_trx_irq_flag_clr();
    /*Calling the interrupt routines*/
    irq_hdl_trx();
}


#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
/*
 * @brief ISR for RTC overflow interrupt
 */
extern unsigned long top_value_rtc;
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
/* RTC Interrupt  */
#pragma handler = AVR32_RTC_IRQ_GROUP, 1
__interrupt
#endif
void rtc_irq(void)
{
    /*clear the interrupt flag*/
    rtc_clear_interrupt(&AVR32_RTC);

    /* Code to be written here to wake up the device if in Power-down mode*/
#ifdef WATCHDOG
    wdt_clear();
#endif
#ifndef WATCHDOG
    rtc_set_top_value(&AVR32_RTC, RTC_TOP_VALUE_1s);
#else
    rtc_set_top_value(&AVR32_RTC, top_value_rtc);
#endif
    rtc_enable_interrupt(&AVR32_RTC);
    rtc_enable(&AVR32_RTC);
}
#endif
#endif /* RZ600_231 */
/* EOF */

