/**
 * @file wireshark.c
 *
 * @brief Wireshark Interface Firmware
 *
 * This file implements Wireshark Interface Firmware
 *
 * $Id: wireshark.c 33712 2012-12-03 11:13:35Z yogesh.bellan $
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
#include "tal_internal.h"
#include "app_config.h"
#include "ieee_const.h"
#include "bmm.h"
#include "tal_rx.h"
#include "tal_constants.h"

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

#define LED_APP_START           (LED_0)
#define LED_CAPTURING           (LED_1)

/* === GLOBALS ============================================================= */
bool buffer_write_free = true;

/* === PROTOTYPES ========================================================== */

/* === IMPLEMENTATION ====================================================== */
/**
 * @brief Callback that is called if data has been received by trx.
 *
 * @param received_frame    Pointer to received data structure
  */
void sniffer_handle_received_frame(uint8_t *received_frame)
{
  /* Local variables */
    frame_info_t *rx_frame = (frame_info_t*)received_frame;
    /* Extract payload length */
    uint8_t payload_length = *rx_frame->mpdu;
    uint8_t *rx_frame_ptr = rx_frame->mpdu;
    if ((payload_length >=5) && (payload_length <= 127))
    {
        /* Print received data to terminal program. */
        bool sio_ongoing = true;
        uint8_t sio_len_rx;
        uint8_t first_ptr=0;
        payload_length = payload_length-1;
        //tal_rx_enable(TRX_OFF);
        while (sio_ongoing)
        {
            sio_len_rx = sniffer_sio_tx(&rx_frame_ptr[first_ptr], payload_length);

            if (sio_len_rx < payload_length)
            {
                payload_length -= sio_len_rx;
                first_ptr += sio_len_rx;
                pal_task();
            }
            else
            {
                sio_ongoing = false;
                buffer_write_free = false;				
            }
        }
    }
    else
    {
        return;
    }
}


/**
 * @brief Callback that is called once tx is done.
 *
 * @param status    Status of the transmission procedure
 * @param frame     Pointer to the transmitted frame structure
 */
void sniffer_handle_tx_done(retval_t status)
{
    /* Keep compiler happy */
    status = status;
}
/**
 * @brief initialization of sniffer
 */
void sniffer_config_select(uint8_t Cmd,uint8_t Channel)
{
    bool mode = true;
    uint8_t temp_val;
    switch(Cmd)
    {
    case CHANNEL_SELECT_ID:
        temp_val = 0;   // 2 = O-QPSK
        tal_pib_set(phyCurrentPage, (pib_value_t *)&temp_val);
        tal_pib_set(phyCurrentChannel, (pib_value_t *)&Channel);
        break;
    case START_CAPTURE_ID:
        tal_pib_set(macPromiscuousMode, (pib_value_t *)&mode);
        tal_rx_enable(PHY_RX_ON);
        Channel= Channel;
        break;
    case STOP_CAPTURE_ID:
        tal_rx_enable(PHY_TRX_OFF);
        Channel= Channel;
        break;
    default:
        break;
    }
}
/* EOF */
