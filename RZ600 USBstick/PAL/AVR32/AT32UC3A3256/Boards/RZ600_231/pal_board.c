/**
 * @file pal_board.c
 *
 * @brief PAL board specific functionality
 *
 * This file implements PAL board specific functionality for RZ600
 * which contains AVR32 AT32UC3A3256.
 *
 * $Id: pal_board.c 32382 2012-06-20 10:43:20Z yogesh.bellan $
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

#include <stdbool.h>
#include <stdlib.h>
#include "pal_types.h"
#include "pal.h"
#include "pal_boardtypes.h"
#include "pal_config.h"
#include "pal_internal.h"
#include "pal_timer.h"

#if (BOARD_TYPE == RZ600_231)

/* === Macros ============================================================== */

/* === Types =============================================================== */
/**
 * Create a table for interrupt groups.
 * Each group contains a member pointer to corresponding interrupt handler.
 */
#if defined(__GNUC__)
struct
{
    volatile __int_handler int_handler;
} int_table[AVR32_INTC_NUM_INT_GRPS];
#endif

/* === Globals ============================================================= */

#if defined(SLEEPING_TIMER) || defined(WATCHDOG) || defined(DOXYGEN)
volatile unsigned long wdt_timeout_period = WDT_MIN_VALUE_US;
volatile uint8_t WDT_PSEL;
unsigned long top_value_rtc;
#endif

/* === Prototypes ========================================================== */


/* === Implementation ======================================================= */

/**
 * @brief Provides timestamp of the last received frame
 *
 * This function provides the timestamp (in microseconds)
 * of the last received frame.
 *
 * @param[out] Timestamp in microseconds
 */
void pal_trx_read_timestamp(uint32_t *timestamp)
{
    /*
     * Everytime a transceiver interrupt is triggred, input capture register of
     * the AVR is latched. The 'sys_time' is concatenated to the ICR to
     * generate the time stamp of the received frambe.
     * 'ICR_Higher byte'   'ICR_Lower byte'
     *  ---------|--------- => 32 bit timestamp
     *   16 bits   16 bits
     */
    *timestamp  = (uint32_t)TIMER_HIGH_REGISTER << (uint32_t)16;
    *timestamp |= (uint32_t)TIMER_LOW_REGISTER;
}


/**
 * @brief Calibrates the internal RC oscillator
 *
 * This function calibrates the internal RC oscillator.
 *
 * @return True since the RC oscillator is always calibrated
 *         automatically at startup by hardware itself
 */
bool pal_calibrate_rc_osc(void)
{
    return (true);
}


/**
 * @brief Enables the GPIO for ouput
 *
 * @param pin GPIO pin to be configured for output
 */
void gpio_enable_output(uint32_t pin)
{
    volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> PORT_NUM];
    gpio_port->gpers = 1 << (pin & 0x1F);
    gpio_port->oderc = 1 << (pin & 0x1F);
    gpio_port->oders = 1 << (pin & 0x1F); // The GPIO output driver is enabled for that pin.
}

/**
 * @brief Enables the GPIO for input
 *
 * @param pin GPIO pin to be configured for input
 */
void gpio_enable_input(uint32_t pin)
{
    volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> PORT_NUM];
    gpio_port->gpers = 1 << (pin & 0x1F);
    gpio_port->oderc = 1 << (pin & 0x1F);
    gpio_port->oderc = 1 << (pin & 0x1F); /* The GPIO output driver is disabled for that pin.*/
}

/**
 * @brief Sets the GPIO pin
 *
 * @param pin GPIO pin to be set
 */
void gpio_set_gpio_pin(uint32_t pin)
{
    volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> PORT_NUM];
    gpio_port->ovrs  = 1 << (pin & 0x1F); /* Value to be driven on the I/O line: 1.*/
}

/**
 * @brief Clears the GPIO pin
 *
 * @param pin GPIO pin to be cleared
 */
void gpio_clr_gpio_pin(uint32_t pin)
{
    volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> PORT_NUM];
    gpio_port->ovrc  = 1 << (pin & 0x1F); /* Value to be driven on the I/O line: 0.*/
    gpio_port->oders = 1 << (pin & 0x1F); /* The GPIO output driver is enabled for that pin.*/
    gpio_port->gpers = 1 << (pin & 0x1F); /* The GPIO module controls that pin.*/
}

/**
 * @brief Toggles the GPIO pin
 *
 * @param pin GPIO pin to be toggled
 */
void gpio_tgl_gpio_pin(uint32_t pin)
{
    volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> PORT_NUM];
    gpio_port->ovrt  = 1 << (pin & 0x1F); /* Value to be driven on the I/O line: tgl.*/
}

/**
 * @brief Gets the status of the GPIO pin
 *
 * @param pin GPIO pin whose status to be read
 */
pin_state_t gpio_get(uint32_t pin)
{
    volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> PORT_NUM];

    /* Check whether the pin is HIGH */
    if ((gpio_port->pvr >> (pin & 0x1F)) & HIGH)
    {
        return(HIGH);
    }
    else
    {
        return(LOW);
    }
}


/**
 * @brief Sets the GPIO pin state
 *
 * @param pin GPIO pin to be made HIGH or LOW
 * @param state New pin state
 */
void pal_gpio_set(gpio_pin_type_t pin, pin_state_t state)
{
    if (SLP_TR_PIN == pin)
    {
        if (HIGH == state)
        {
            gpio_set_gpio_pin(SLP_TR);
        }
        else
        {
            gpio_clr_gpio_pin(SLP_TR);
        }
    }
    else if (RST_PIN == pin)
    {
        if (HIGH == state)
        {
            gpio_set_gpio_pin(TRX_RST);
        }
        else
        {
            gpio_clr_gpio_pin(TRX_RST);
        }
    }
}


/**
 * @brief Gets the current GPIO pin state
 *
 * @param gpio_pin GPIO pin to be read
 * @return pin_state Current GPIO pin state
 */
pin_state_t pal_gpio_get(gpio_pin_type_t gpio_pin)
{
    pin_state_t pin_state = LOW;

    if (RST_PIN == gpio_pin)
    {
        pin_state = gpio_get(TRX_RST);
    }
    else if (SLP_TR_PIN == gpio_pin)
    {
        pin_state = gpio_get(SLP_TR);
    }

    return pin_state;
}

/**
 * @brief Enables a specific module mode for a pin.
 *
 * @param pin The pin number
 * @param function The pin function
 */
static void gpio_enable_module_pin(uint32_t pin, uint32_t function)
{
    volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> PORT_NUM];
    /* Enable the correct function.*/
    switch (function)
    {
        case 0: // A function.
            gpio_port->pmr0c = 1 << (pin & 0x1F);
            gpio_port->pmr1c = 1 << (pin & 0x1F);
            break;

        case 1: // B function.
            gpio_port->pmr0s = 1 << (pin & 0x1F);
            gpio_port->pmr1c = 1 << (pin & 0x1F);
            break;

        case 2: // C function.
            gpio_port->pmr0c = 1 << (pin & 0x1F);
            gpio_port->pmr1s = 1 << (pin & 0x1F);
            break;

        case 3: // D function.
            gpio_port->pmr0s = 1 << (pin & 0x1F);
            gpio_port->pmr1s = 1 << (pin & 0x1F);
            break;

        default:
            return;
    }

    /* Disable GPIO control.*/
    gpio_port->gperc = 1 << (pin & 0x1F);
}


/**
 * @brief Initializes the GPIO pins
 *
 * This function is used to initialize the transceiver and peripheral
 * port pins from AT32UC3A3235.
 */
void gpio_init(void)
{
#if ((defined UART0) ||(defined UART1) ||(defined UART2) || (defined UART3))
    /* Set-up the UART pins to respective functions. */
    gpio_enable_module_pin(USART_RX, USART_RX_FUNCTION);
    gpio_enable_module_pin(USART_TX, USART_TX_FUNCTION);
#endif

    /* Initialize peripheral interrupt pin. */
    gpio_enable_input(EXT_INT);
    gpio_enable_module_pin(AVR32_EIC_EXTINT_8_PIN, AVR32_EIC_EXTINT_8_FUNCTION);

    /* Initialize TRX_RST, SLP_TR and SEL as GPIO. */
    gpio_enable_output(TRX_RST);
    gpio_enable_output(SLP_TR);
    gpio_set_gpio_pin(SLP_TR);
    gpio_enable_output(SEL);
    gpio_set_gpio_pin(SEL);

    /* Initialize pins for SPI pins(11 - 13) to respective functions. */
    gpio_enable_module_pin(MOSI, MOSI_FUNCTION);
    gpio_enable_module_pin(MISO, MISO_FUNCTION);
    gpio_enable_module_pin(SCK, SCK_FUNCTION);
    gpio_enable_input(AVR32_PIN_PX18);
    gpio_enable_module_pin( AVR32_TC0_A2_0_PIN, AVR32_TC0_A2_0_FUNCTION);
}

/**
 * @brief This function registers the corresponding interrupt for the group.
 *
 * The function stores pointer to the interrupt handler in int_table[group],
 * so that _get_interrupt can retrieve it when vectored.
 * And sets the corresponding interrupt group's priority level.
 *
 * @param handler Function pointer to the corresponding interrupt subroutine.
 * @param group Interrupt group.
 * @param int_level Priority level for the group.
 */
#if defined(__GNUC__)
void register_interrupt(__int_handler handler, uint32_t group, uint32_t int_level)
{
    int_table[group].int_handler = handler;
    AVR32_INTC.ipr[group] = ipr_val[int_level & (AVR32_INTC_IPR_INTLEVEL_MASK >> AVR32_INTC_IPR_INTLEVEL_OFFSET)];
}
#endif

/**
 * @brief This function gets the interrupt handler of the current event.
 *
 * @param int_level Interrupt priority level to handle.
 * @return Interrupt handler to execute.
 */
#if defined(__GNUC__)
__int_handler _get_interrupt_handler(uint32_t int_level)
{
    uint32_t int_grp = AVR32_INTC.icr[AVR32_INTC_INT3 - int_level];
    uint32_t int_req = AVR32_INTC.irr[int_grp];
    return (int_req) ? int_table[int_grp].int_handler : NULL ;
}
#endif


/**
 * @brief Initialize the interrupt system of the ATUC3A3256
 */
void    interrupt_system_init(void)
{
    /* Enable high priority interrupts */
#if defined(__GNUC__)

    /* Disable all interrupts.*/
    pal_global_irq_disable();

    /* Initialize EVBA address.*/
    Set_system_register(AVR32_EVBA, (int)&_evba );

    register_interrupt(&ext_int_isr, EXT_INT_ISR_GROUP, EXT_INT_ISR_PRIORITY);

#if (TOTAL_NUMBER_OF_TIMERS > 0)
    register_interrupt(&TC_isr, TC_CH1_ISR_GROUP, TC_CH1_ISR_PRIORITY);
    register_interrupt(&TC2_isr, TC_CH2_ISR_GROUP, TC_CH2_ISR_PRIORITY);
#endif

#ifdef SIO_HUB
    register_interrupt(&usb_general_interrupt, USB_GROUP, 0);
#endif

#ifdef TOUCH_UC3LEK
    register_interrupt(&touch_acq_done_irq, CAT_ISR_GROUP, AVR32_INTC_INT3);
#endif

#endif
}


/**
 * @brief Initialize the clock of the ATUC3A3256
 */
void clock_init(void)
{
    /*Pll to get maximum output*/
    avr32_pm_pll_t pll_conf;

    /* To set the oscillator mode */
    AVR32_PM.OSCCTRL0.mode = AVR32_PM_OSCCTRL0_MODE_CRYSTAL_G3;

    /*Start up time for the oscillator is set*/
    AVR32_PM.OSCCTRL0.startup = AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC;

    /* Enable the Osc0. */
    AVR32_PM.mcctrl = AVR32_PM_MCCTRL_OSC0EN_MASK;

    /*Waiting till the internal oscillator gets stable*/
    while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_OSC0RDY_MASK));

    /* Switch main clock to Osc0. */
    AVR32_PM.MCCTRL.mcsel = AVR32_PM_MCSEL_OSC0;

    /* Set-up the PLL, enable it and wait for lock. */
    pll_conf.pllmul   = CLOCK_MULTIPLIER;             /*To make the oscillator output to 32Mhz*/
    pll_conf.plldiv   = CLOCK_DIVIDER;                /*Assigning the division factor for PLL*/
    pll_conf.pllosc   = AVR32_PM_PLLOSC_USER;         /*Selecting the type of oscillator*/
    pll_conf.pllopt   = AVR32_PM_PLL0_PLLOPT_SIZE;    /*To make the oscillator output divided by 2 so that it gives us 32Mhz*/
    pll_conf.pllen    = AVR32_PM_PLL0_PLLEN_USER;     /*Count of the pll is configured*/
    //pll_conf.pllcount = AVR32_PM_PLL0_PLLCOUNT_USER;/*Enabling the pll*/
    AVR32_PM.PLL[PLL0_SELECT] = pll_conf;          /*Assigning the pll value to get a maximum of 32Mhz output*/

    /*Waiting till the oscillator gets stable*/
    while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_LOCK0_MASK));

    /* Set-up the bus speed: 12MHz with the HSB at 48MHz. */
    avr32_pm_cksel_t bus_conf;

    /*  To set the HSB bus for the usb features*/
    bus_conf.hsbsel = ZERO_INIT;
    bus_conf.hsbdiv = ZERO_INIT;
    bus_conf.pbasel = ZERO_INIT;
    bus_conf.pbadiv = ZERO_INIT;
    bus_conf.pbbsel = ZERO_INIT;
    bus_conf.pbbdiv = ZERO_INIT;
    AVR32_PM.CKSEL = bus_conf;

    /*Waiting till the clock gets stable*/
    while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_CKRDY_MASK));

    /* Set one wait state. */
    AVR32_FLASHC.FCR.fws = ENABLE_FCR;

    /* Enable clock. */
    AVR32_PM.MCCTRL.mcsel = AVR32_PM_MCCTRL_MCSEL_PLL0;

    /* Set up GCLK for USB. */
    avr32_pm_gcctrl_t gc_conf;

    /*Enabling the clock to the usb*/
    gc_conf.cen = USB_CLOCK_ENABLE;
    gc_conf.pllsel = ZERO_INIT;

    /*Selecting the clock control for the usb peripheral*/
    AVR32_PM.GCCTRL[AVR32_PM_GCLK_USBB] = gc_conf;

    /*Selecting pba clock to spi peripheral */
    AVR32_PM.clkmask[SPI_BUS] |= SPI_PBA_BUS_SEL;

    /*Selecting pba clock to timer peripheral */
    AVR32_PM.clkmask[TIMER_BUS] |= TIMER_PBA_BUS_SEL;

    /*Selecting pba clock to uart peripheral */
    AVR32_PM.clkmask[UART_BUS] |= USART_PBA_BUS_SEL;
}

/**
 * @brief Initialize the timer of the ATUC3A3256
 */
void timer_init_non_generic(void)
{
    /* Timer channel 0 configuration */
    /* Set up the timer channel0 to make a 1 millisecond output to feed channel 1. */
    avr32_tc_cmr_t cmr_conf = {.waveform = {
            .acpc = AVR32_TC_CMR0_ACPC_CLEAR,             /* 0x02 - CLear- RC Compare Effect on TIOA */
            .acpa = AVR32_TC_CMR0_ACPA_SET,               /* 0x01 - Set - RA Compare Effect on TIOA */
            .wave = SET,                                  /* 0x01 - Capture Mode is Disabled */
            .wavsel = AVR32_TC_CMR0_WAVSEL_UP_NO_AUTO,    /* 0x02 - UP mode with automatic trigger on RC Compare */
            .tcclks = AVR32_TC_CMR0_TCCLKS_TIMER_CLOCK4,  /* 0x01 - F_pba/32 = 0.9 Microsecond */
        }
    };

    /*AVR32_TC0.channel[0].CMR*/
    TC_CH0.CMR = cmr_conf;
    /*All interrupts are disabled for timers*/
    TC_CH0.idr = DISABLE_ALL_TIMER_INTERRUPTS;
    /*Register A value to Comapre*/
    TC_CH0.ra = ONE_MILLISECOND_DELAY;
    /*Register C value to Comapre*/
    TC_CH0.rc = ONE_MILLISECOND_OVERFLOW;
    /*Clock is Enabled for channel 0*/
    TC_CH0.CCR.clken = AVR32_TC_CCR0_CLKEN_SIZE;

    /* Timer channel 1 configuration */
    avr32_tc_cmr_t cmr_conf2 = {.waveform = {
            .bswtrg = SET,                                /* Software trigger enabled*/
            .beevt = NOT_SET,                             /* External Event Effect on TIOB*/
            .eevt = NOT_SET,                               /* TIOB is chosen as the external event signal,
                                                         it is configured as an input*/
            .eevtedg = SET,                               /* Event Selection Rising Edge*/
            .wave = SET,                                  /* 0x01 - Capture Mode is Disabled*/
            .wavsel = AVR32_TC_CMR0_WAVSEL_UP_AUTO,       /* 0x02 - UP mode with automatic trigger on RC Compare*/
            .tcclks = AVR32_TC_CMR0_TCCLKS_XC1,           /* Clock is chained from channel 0 */
        }
    };

    /*AVR32_TC0.channel[1].CMR*/
    TC_CH1.CMR = cmr_conf2 ;
    /* All interrupts are disabled for timers*/
    TC_CH1.idr = DISABLE_ALL_TIMER_INTERRUPTS;
    /*Clock is Enabled for channel 1*/
    TC_CH1.CCR.clken = AVR32_TC_CCR1_CLKEN_SIZE;


    /* Timer channel 2 configuration */
    avr32_tc_cmr_t cmr_conf3 = {.capture = {
            .ldrb = NOT_SET,
            .ldra = SET,                                  /*Loading Each Edge*/
            .wave = NOT_SET,                              /*TIOB is chosen as the external event & configured as an input*/
            .cpctrg = NOT_SET,
            .abetrg = SET,                                /*TIOA as External Trigger*/
            .etrgedg = SET,                               /*Event Selection Rising Edge*/
            .ldbdis = NOT_SET,
            .ldbstop = NOT_SET,
            .burst = NOT_SET,
            .tcclks = AVR32_TC_CMR0_TCCLKS_TIMER_CLOCK4,   /*External timer clock is selected & it gives a clock of 1 Mhz*/
        }
    };
    /*Configuring AVR32_TC1.channel[0].CMR*/
    TC_CH2.CMR = cmr_conf3 ;

    /*All interrupts are disabled for timers*/
    TC_CH2.idr = DISABLE_ALL_TIMER_INTERRUPTS;

    /*Enabling the interrupt for the capture*/
    TC_CH2.ier = AVR32_TC_IER0_LDRAS_MASK ;

    TC_CH2.CCR.clken = AVR32_TC_CCR0_CLKEN_SIZE;

    /*Chaining timer channels & External Clock Signal 1 Selection, TIOA0 is Given as i/p*/
    TC.BMR.tc1xc1s = AVR32_TC_BMR_TC1XC1S_TIOA0;

    /*Software trigger simultaneously for each of the channels*/
    TC.BCR.sync = AVR32_TC_SYNC_SIZE;

    /* Timer/Counter 1 configuration */
    avr32_tc_cmr_t cmr_conf4 = {.waveform = {
            .bswtrg = SET,                                  /*Software trigger enabled*/
            .beevt = NOT_SET,                               /*External Event Effect on TIOB*/
            .eevt = NOT_SET,                                 /*TIOB is chosen as the external event signal,
                                                          it is configured as an input*/
            .eevtedg = SET,                                 /*Event Selection Rising Edge*/
            .wave = SET,                                    /*Capture Mode is Disabled*/
            .wavsel = AVR32_TC_CMR0_WAVSEL_UP_AUTO,         /*UP mode with automatic trigger on RC Compare*/
            .tcclks = AVR32_TC_CMR0_TCCLKS_TIMER_CLOCK4,    /*External timer clock is selected & it gives a clock of 1 Mhz*/
        }
    };
    /*Configuring AVR32_TC1.channel[1].CMR*/
    TC2_CH0.CMR = cmr_conf4 ;
    /*All interrupts are disabled for timers*/
    TC2_CH0.idr = DISABLE_ALL_TIMER_INTERRUPTS;

#ifdef ENABLE_HIGH_PRIO_TMR
    avr32_tc_cmr_t cmr_conf5 = {.waveform = {
            .bswtrg = SET,                                  /*Software trigger enabled*/
            .beevt = NOT_SET,                               /*External Event Effect on TIOB*/
            .eevt = NOT_SET,                                 /*TIOB is chosen as the external event signal,
                                                          it is configured as an input*/
            .eevtedg = SET,                                 /*Event Selection Rising Edge*/
            .wave = SET,                                    /*Capture Mode is Disabled*/
            .wavsel = AVR32_TC_CMR0_WAVSEL_UP_AUTO,         /*UP mode with automatic trigger on RC Compare*/
            .tcclks = AVR32_TC_CMR0_TCCLKS_TIMER_CLOCK4,    /*External timer clock is selected & it gives a clock of 1 Mhz*/
        }
    };
    /*Configuring AVR32_TC1.channel[1].CMR*/
    TC2_CH1.CMR = cmr_conf5 ;
    /*All interrupts are disabled for timers*/
    TC2_CH1.idr = DISABLE_ALL_TIMER_INTERRUPTS;
    /*Test value to test the output compare effectively*/
    TC2_CH1.CCR.clken = AVR32_TC_CCR0_CLKEN_SIZE;
#endif

    /*Enable overflow interrupt*/
    TC2_CH0.ier   =  AVR32_TC_IER0_COVFS_MASK;

    /*Enabling the clock*/
    TC2_CH0.CCR.clken = AVR32_TC_CCR0_CLKEN_SIZE;

    /*Chaining timer channel 0 & synching them */
    TC.BCR.sync = AVR32_TC_SYNC_SIZE;

    /*Chaining timer channel 0 & synching them */
    TC2.BCR.sync = AVR32_TC_SYNC_SIZE;
}

/**
 * @brief Initialize the watchdog timer of the ATUC3A3256
 */
#if defined(WATCHDOG) || defined(DOXYGEN)
void wdt_init(void)
{
    pal_wdt_enable(wdt_timeout_period);
}
#endif

/**
 * @brief Enable the watchdog timer and set the desired time-out period
 *
 * @param Time-out period to be set for the Watchdog Timer
 */
#if defined(WATCHDOG) || defined(DOXYGEN)
void pal_wdt_enable(unsigned long us_timeout_period)
{

    /* Set the CTRL.EN bit and translate the us timeout to fit in CTRL.PSEL using
    the formula Twdt = 2pow(PSEL+1) / fRCosc */
    wdt_set_ctrl(AVR32_WDT_CTRL_EN_MASK |
                 ((32 - clz(((((Min(Max(us_timeout_period, MIN_US_TIMEOUT_PERIOD), MAX_US_TIMEOUT_PERIOD) *
                                AVR32_PM_RCOSC_FREQUENCY + 500000) / 1000000) << 1) - 1) >> 1) - 1) <<
                  AVR32_WDT_CTRL_PSEL_OFFSET));
}
#endif

/**
 * @brief Set the CTRL register of the watchdog timer of the ATUC3A3256
 */
#if defined(WATCHDOG) || defined(DOXYGEN)
void wdt_set_ctrl(unsigned long ctrl)
{
    AVR32_WDT.ctrl = ctrl | (AVR32_WDT_KEY_VALUE << AVR32_WDT_CTRL_KEY_OFFSET);
    AVR32_WDT.ctrl = ctrl | ((unsigned long) (~AVR32_WDT_KEY_VALUE << AVR32_WDT_CTRL_KEY_OFFSET) & AVR32_WDT_CTRL_KEY_MASK);
}
#endif

/**
 * @brief Clears by writing zero to the CLR register of the watchdog timer of the ATUC3A3256
 */
#if defined(WATCHDOG) || defined(DOXYGEN)
void wdt_clear(void)
{
    AVR32_WDT.clr = 0;
#ifndef SLEEPING_TIMER
    pal_timer_start(WDT_PARALLEL_TIMER, WDT_PARALLEL_TIMER_TIMEOUT_US, TIMEOUT_RELATIVE, (FUNC_PTR)wdt_clear, NULL);
#endif
}
#endif

/**
 * @brief Parallel Software timer initialization if the Sleeping timer (RTC) is
 * not enabled to reset the Watchdog timer periodically.
 */
#if defined(WATCHDOG) || defined(DOXYGEN)
void wdt_parallel_timer_init()
{
    /*Start Parallel timer for watchdog clear if sleeping timer is not running*/
    pal_timer_start(WDT_PARALLEL_TIMER, WDT_PARALLEL_TIMER_TIMEOUT_US, TIMEOUT_RELATIVE, (FUNC_PTR)wdt_clear, NULL);
}
#endif

/**
 * @brief Initialization of sleeping timer
 */
#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
void sleeping_timer_init(void)
{
#ifndef WATCHDOG
    sleeping_timer_without_wdt();
#else
    sleeping_timer_with_wdt();
#endif
}
#endif

/**
 * @brief Initialization of Sleeping timer (RTC) of the At32uc3a3256 when Watchdog Timer
 * is not enabled. The period for the Sleeping timer should be given by the user in this case.
 */
#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
void sleeping_timer_without_wdt(void)
{
    /*RTC Initialization with the frequency of 57.5KHz*/
    pal_rtc_init(&AVR32_RTC, RTC_OSC_RC);
    /*Sets the Top value specified by the user for sleeping timer(RTC)*/
    rtc_set_top_value(&AVR32_RTC, RTC_TOP_VALUE_1s);
    pal_global_irq_disable();
    /*Enable RTC Interrupt*/
    rtc_enable_interrupt(&AVR32_RTC);
    pal_global_irq_enable();
    /*Start Sleeping Timer*/
    rtc_enable(&AVR32_RTC);
}
#endif

/**
 * @brief Initialization of Sleeping timer (RTC) of the At32uc3a3256 when Watchdog Timer
 * is enabled. The period for the Sleeping timer should be given by the user in this case
 * and the period of the sleeping timer should be less than the time-out period of the
 * watchdog timer.
 * @note It is required that the period of the Sleeping Timer (RTC) is less than the Time-out period
 * of Watchdog Timer (If enabled) to avoid the unintentional reset of the device.
 */
#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
void sleeping_timer_with_wdt(void)
{
    /*Read the Watchdog PSEL value set according to the time-out period set by the user*/
    WDT_PSEL = (uint8_t) ((AVR32_WDT.ctrl & AVR32_WDT_CTRL_PSEL_MASK) >> AVR32_WDT_CTRL_PSEL_OFFSET);
    /*Calculate the Top value to be set for the RTC which should be slightly less than the Time-out period of Watchdog*/
    top_value_rtc = ((1 << WDT_PSEL) / 2) - 1;
    /*RTC Initialization with the frequency of 57.5KHz*/
    pal_rtc_init(&AVR32_RTC, RTC_OSC_RC);
    /*Sets the Top value calculated above for sleeping timer(RTC)*/
    rtc_set_top_value(&AVR32_RTC, top_value_rtc);
    pal_global_irq_disable();
    /*Enable RTC Interrupt*/
    rtc_enable_interrupt(&AVR32_RTC);
    pal_global_irq_enable();
    /*Start Sleeping Timer*/
    rtc_enable(&AVR32_RTC);
}
#endif

/**
 * @brief Check whether RTC is busy
 */
#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
int rtc_is_busy(volatile avr32_rtc_t *rtc)
{
    return (rtc->ctrl & AVR32_RTC_CTRL_BUSY_MASK) != 0;
}
#endif

/**
 * @brief Initialization of the RTC
 * @param osc_type
 */
#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
void pal_rtc_init(volatile avr32_rtc_t *rtc, unsigned char osc_type)
{
    /* Wait until the rtc CTRL register is up-to-date*/
    while (rtc_is_busy(rtc));
    /* Set the new RTC configuration*/
    rtc->ctrl = osc_type << AVR32_RTC_CTRL_CLKEN_OFFSET |
                RTC_PSEL << AVR32_RTC_CTRL_PSEL_OFFSET |
                AVR32_RTC_CTRL_CLKEN_MASK;
    /* Wait until write is done*/
    while (rtc_is_busy(rtc));
    /* Set the counter value to 0*/
    rtc_set_value(rtc, 0x00000000);
}
#endif

/**
 * @brief Sets the value in the VAL register of the RTC module
 * @param val value
 */
#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
void rtc_set_value(volatile avr32_rtc_t *rtc, unsigned long val)
{
    /* Wait until we can write into the VAL register*/
    while (rtc_is_busy(rtc));
    /*Set the new val value*/
    rtc->val = val;
    /* Wait until write is done*/
    while (rtc_is_busy(rtc));
}
#endif

/**
 * @brief Enables the RTC interrupt
 */
#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
void rtc_enable_interrupt(volatile avr32_rtc_t *rtc)
{
    rtc->ier = AVR32_RTC_IER_TOPI_MASK;
}
#endif

/**
 * @brief Sets the top value in the TOP register of the RTC module
 * @param top
 */
#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
void rtc_set_top_value(volatile avr32_rtc_t *rtc, unsigned long top)
{
    /* Wait until we can write into the VAL register*/
    while (rtc_is_busy(rtc));
    /* Set the new val value*/
    rtc->top = top;
    /* Wait until write is done*/
    while (rtc_is_busy(rtc));
}
#endif

/**
 * @brief Enables the RTC module
 */
#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
void rtc_enable(volatile avr32_rtc_t *rtc)
{
    /* Wait until the rtc CTRL register is up-to-date*/
    while (rtc_is_busy(rtc));
    /* Enable the RTC*/
    rtc->ctrl |= AVR32_RTC_CTRL_EN_MASK;
    /* Wait until write is done*/
    while (rtc_is_busy(rtc));
}
#endif


/**
 * @brief Clears the RTC interrupt
 */
#if defined(SLEEPING_TIMER) || defined(DOXYGEN)
void rtc_clear_interrupt(volatile avr32_rtc_t *rtc)
{
    if (Is_global_interrupt_enabled())
    {
        Disable_global_interrupt();
    }
    rtc->icr = AVR32_RTC_ICR_TOPI_MASK;
    rtc->isr;
    if (Is_global_interrupt_enabled())
    {
        Enable_global_interrupt();
    }
}
#endif

/**
 * @brief Initialize LEDs
 */
void pal_led_init(void)
{
    gpio_enable_output(LED0_PIN);
    gpio_enable_output(LED1_PIN);
}


/**
 * @brief Control LED status
 *
 * param[in]  led_no LED ID
 * param[in]  led_setting LED_ON, LED_OFF, LED_TOGGLE
 */
void pal_led(led_id_t led_no, led_action_t led_setting)
{
    uint8_t pin;
    switch (led_no)
    {
        case LED_0:
            pin = LED0_PIN;
            break;
        case LED_1:
            pin = LED1_PIN;
            break;
        default:
            pin = LED1_PIN;
            break;
    }

    switch (led_setting)
    {
        case LED_ON:
            gpio_clr_gpio_pin(pin);
            break;
        case LED_OFF:
            gpio_set_gpio_pin(pin);
            break;
        case LED_TOGGLE:
            gpio_tgl_gpio_pin(pin);
            break;
        default: /* do nothing */
            break;
    }
}

/**
 * @brief Initialize the button
 */
void pal_button_init(void)
{
    /* There are no buttons on RZ600*/
}

/**
 * @brief Read button
 *
 * @param button_no Button ID
 */
button_state_t pal_button_read(button_id_t button_no)
{
    /* Keep compiler happy */
    button_no = button_no;

    /* There are no buttons on RZ600*/
    return BUTTON_OFF;
}


#endif /* RZ600_231 */
/* EOF */
