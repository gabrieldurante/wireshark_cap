/**
 * @file main.c
 *
 * @brief Application Main
 *
 * This file implements main of the Wireshark Sniffer Interface.
 *
 * $Id: main.c 33742 2012-12-05 14:48:05Z yogesh.bellan $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === INCLUDES ============================================================ */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "pal.h"
#include "tal.h"
#include "app_config.h"
#include "ieee_const.h"
#include "bmm.h"
#include "sio_handler.h"

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */
#define INIT_STATE 0
#define SET_CHANNEL 1
#define SET_START   2
#define SET_STOP   3
#define CAPTURE_INPROGRESS 4
#define WSBRIDGE_START  5
#define WS_RESTART 6
#define INIT_VALUE	0
#define STOP_CAP	0x99

/* === GLOBALS ============================================================= */

uint8_t sio_rx_data[MAX_GUI_CMD_LENGTH];
/* === PROTOTYPES ========================================================== */

static void app_task(void);

static uint8_t Wireshark_Settings;


/* === IMPLEMENTATION ====================================================== */

/**
 * @brief Main function of the Sniffer application
 */
int main(void)
{

   /* Initialize the TAL layer */
    if (tal_init() != MAC_SUCCESS)
    {
        /* something went wrong during initialization*/
        pal_alert();
    }

    /* Calibrate MCU's RC oscillator */
    pal_calibrate_rc_osc();

    /* Initialize LEDs */
    pal_led_init();


    /*
     * The stack is initialized above, hence the global interrupts are enabled
     * here.
     */
    pal_global_irq_enable();

    /* Initialize the serial interface used for communication with sniffer
       GUI */
    if (pal_sio_init(SIO_CHANNEL) != MAC_SUCCESS)
    {
        /* something went wrong during initialization */
        pal_alert();
    }

    //sio_getchar();

    Wireshark_Settings = INIT_STATE ;

    /* Endless while loop */
    while (1)
    {
        pal_task();
        tal_task();
        app_task();
    }
}


/**
 * @brief Application task handler
 */
static void app_task(void)
{
    uint8_t number_of_bytes_to_be_transmitted;
	uint8_t stop_data;
   if(Wireshark_Settings == INIT_STATE)
   {
        uint8_t input_channel = sio_getchar();
        sniffer_config_select(CHANNEL_SELECT_ID,input_channel);
        Wireshark_Settings = SET_START;
   }

   if(Wireshark_Settings == SET_START)
   {
        number_of_bytes_to_be_transmitted = sio_getchar();
        if(number_of_bytes_to_be_transmitted == 0x01)
        {
           sniffer_config_select(START_CAPTURE_ID,number_of_bytes_to_be_transmitted);
           Wireshark_Settings = WS_RESTART;
           sio_rx_data[0] = INIT_VALUE;
        }
   }

   if(Wireshark_Settings == WS_RESTART)
   {
        number_of_bytes_to_be_transmitted = pal_sio_rx(SIO_CHANNEL, sio_rx_data, 1);
        if((number_of_bytes_to_be_transmitted  == 1) && (sio_rx_data[0] == 0x02))
        {
                sio_rx_data[0] = INIT_VALUE;
                Wireshark_Settings = INIT_STATE;
                stop_data = STOP_CAP;
                while((number_of_bytes_to_be_transmitted = pal_sio_tx(SIO_CHANNEL, &stop_data, 1)) !=1);
        }
   }
}


/**
 * @brief Callback that is called if data has been received by trx.
 *
 * @param rx_frame    Pointer to received data structure
  */
void tal_rx_frame_cb(frame_info_t *rx_frame)
{
    buffer_write_free = true;
    /* Handle the received frame */
    sniffer_handle_received_frame((uint8_t*)rx_frame);
    /* Free the received pointer */
    bmm_buffer_free(rx_frame->buffer_header);
}


/**
 * @brief Callback that is called once tx is done.
 *
 * @param status    Status of the transmission procedure
 * @param frame     Pointer to the transmitted frame structure
 */
void tal_tx_frame_done_cb(retval_t status, frame_info_t *frame)
{
    /* Handle the tx done callback */
    sniffer_handle_tx_done(status);
    frame = frame;
}

/**
 * @brief Transmits data through selected SIO unit
 *
 * This function transmits data through the selected SIO unit.
 *
 * @param data Pointer to the data to be transmitted is present
 * @param length Number of bytes to be transmitted
 *
 * @return Actual number of transmitted bytes
 */
uint8_t sniffer_sio_tx(uint8_t *data, uint8_t length)
{
    return(pal_sio_tx(SIO_CHANNEL, data, length));
}

void tal_ed_end_cb(uint8_t max_ed_level)
{
	max_ed_level = max_ed_level;
}

/* EOF */
