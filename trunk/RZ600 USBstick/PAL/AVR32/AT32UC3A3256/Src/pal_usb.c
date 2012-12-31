/**
 * @file pal_usb.c
 *
 * @brief PAL USB related functions
 *
 * This file implements USB related transmission and reception
 * functions
 *
 * $Id: pal_usb.c 32382 2012-06-20 10:43:20Z yogesh.bellan $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */
/* === Includes ============================================================= */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pal.h"
#include "return_val.h"
#include "pal_usb.h"
#include "app_config.h"
#include "usb_device_task.h"

#ifdef USB0

#if (DEBUG > 1)
# include <stdio.h>
#endif

/* === Macros =============================================================== */
#if !defined(USB_VID) || !defined(USB_PID) || !defined(USB_RELEASE) ||\
    !defined(USB_VENDOR_NAME) || !defined(USB_PRODUCT_NAME)
#  error "Define USB_VID, USB_PID, USB_RELEASE, USB_VENDOR_NAME and USB_PRODUCT_NAME in app_config.h"
#endif

/* === Includes ============================================================= */
#include "app_config.h"
#include "usb_task.h"
#include "usb_drv.h"
#include "usb_standard_request.h"
#include "uart_usb_lib.h"

/* === Globals ============================================================== */
/**
 * USB transmit buffer
 * The buffer size is defined in app_config.h
 */
static uint8_t usb_tx_buf[USB_TX_BUF_SIZE];

/**
 * USB receive buffer
 * The buffer size is defined in app_config.h
 */
static uint8_t usb_rx_buf[USB_RX_BUF_SIZE];

/**
 * Transmit buffer head
 */
static uint8_t usb_tx_buf_head;

/**
 * Transmit buffer tail
 */
static uint8_t usb_tx_buf_tail;

/**
 * Receive buffer head
 */
static uint8_t usb_rx_buf_head;

/**
 * Receive buffer tail
 */
static uint8_t usb_rx_buf_tail;

/**
 * Number of bytes in transmit buffer
 */
static uint8_t usb_tx_count;

/**
 * Number of bytes in receive buffer
 */
static uint8_t usb_rx_count;

/**
 * Number of bytes received count
 */
static volatile uint16_t sof_cnt;


/* === Implementation ======================================================= */
/**
 * @brief Initialization of the Usb
 *
 * This function is used to Initialize the Usb.
 *
 * @param none
 * @return none
 */
void sio_usb_init(void)
{
    sof_cnt = 0;
    usb_task_init();
    Usb_enable_sof_interrupt();
}


/**
 * @brief Copies the transmission data to the specified PAL USB buffer
 *
 * This function copies the transmission data to the PAL USB buffer.
 *
 * @param data Pointer to the buffer where the data to be transmitted is present
 * @param length Number of bytes to be transmitted
 *
 * @return number of bytes transmitted
 */
uint8_t sio_usb_tx(uint8_t *data, uint8_t length)
{
    uint8_t bytes_to_be_written;
    uint8_t size;
    uint8_t back;

    /*
     * Calculate available buffer space
     */
    if (usb_tx_buf_tail >= usb_tx_buf_head)
    {
        size = (USB_TX_BUF_SIZE - 1) - (usb_tx_buf_tail - usb_tx_buf_head);
    }
    else
    {
        size = (usb_tx_buf_head - 1) - usb_tx_buf_tail;
    }

    if (size < length)
    {
        /* Not enough buffer space available. Use the remaining size */
        bytes_to_be_written = size;
    }
    else
    {
        bytes_to_be_written = length;
    }

    /* Remember the number of bytes transmitted. */
    back = bytes_to_be_written;
    usb_tx_count += bytes_to_be_written;

    /* The data is copied to the transmit buffer. */
    while (bytes_to_be_written > 0)
    {
        usb_tx_buf[usb_tx_buf_tail] = *data;

        if ((USB_TX_BUF_SIZE - 1) == usb_tx_buf_tail)
        {
            /* Reached the end of buffer, revert back to beginning of buffer. */
            usb_tx_buf_tail = 0;
        }
        else
        {
            /*
             * Increment the index to point the next character to be
             * inserted.
             */
            usb_tx_buf_tail++;
        }

        bytes_to_be_written--;
        data++;
    }

    /*
     * If a transmission needs to be started, pal_usb_handler() takes
     * care about that once it is run.
     */

    return back;
}


/*
 * @brief Puts a character onto USB FIFO
 *
 * This function transmits a byte over usb.
 */
static void usb_putc(void)
{
    if (usb_tx_count > 0)
    {
        /* The number of bytes to be transmitted is decremented. */
        usb_tx_count--;

        /* Write a byte to the endpoint */
        Usb_write_endpoint_data(TX_EP, 8, usb_tx_buf[usb_tx_buf_head]);

        if ((USB_TX_BUF_SIZE - 1) == usb_tx_buf_head)
        {
            /* Reached the end of buffer, revert back to beginning of buffer */
            usb_tx_buf_head = 0;
        }
        else
        {
            usb_tx_buf_head++;
        }
    }
}


/**
 * @brief Copies the data received from USB to the user specified location
 *
 * This function copies the data received from USB to the user specified
 * location.
 *
 * @param data pointer to the buffer where the received data is to be stored
 * @param max_length maximum length of data to be received
 *
 * @return actual number of bytes received
 */
uint8_t sio_usb_rx(uint8_t *data, uint8_t max_length)
{
    uint8_t data_received = 0;

    if (usb_rx_count == 0)
    {
        /* USB receive buffer is empty. */
        return 0;
    }

    if (USB_RX_BUF_SIZE <= usb_rx_count)
    {
        /*
         * Bytes between head and tail are overwritten by new data.
         * The oldest data in buffer is the one to which the tail is
         * pointing. So reading operation should start from the tail.
         */
        usb_rx_buf_head = usb_rx_buf_tail;

        /*
         * This is a buffer overflow case. Byt still only bytes equivalent to
         * full buffer size are useful.
         */
        usb_rx_count = USB_RX_BUF_SIZE;

        /* Bytes received is more than or equal to buffer. */
        if (USB_RX_BUF_SIZE <= max_length)
        {
            /*
             * Requested receive length (max_length) is more than the
             * max size of receive buffer, but at max the full
             * buffer can be read.
             */
            max_length = USB_RX_BUF_SIZE;
        }
    }
    else
    {
        /* Bytes received is less than receive buffer maximum length. */
        if (max_length > usb_rx_count)
        {
            /*
             * Requested receive length (max_length) is more than the data
             * present in receive buffer. Hence only the number of bytes
             * present in receive buffer are read.
             */
            max_length = usb_rx_count;
        }
    }

    data_received = max_length;

    while (max_length > 0)
    {
        /* Start to copy from head. */
        *data = usb_rx_buf[usb_rx_buf_head];
        usb_rx_buf_head++;
        usb_rx_count--;
        data++;
        max_length--;
        if ((USB_RX_BUF_SIZE) == usb_rx_buf_head)
        {
            usb_rx_buf_head = 0;
        }
    }
    return data_received;
}


/*
 * @brief Gets a byte from USB FIFO
 *
 * This function receives a byte from usb.
 */
static void usb_getc(void)
{

    /* The count of characters present in receive buffer is incremented. */
    usb_rx_count++;

    /* Read a character from the endpoint. */
    usb_rx_buf[usb_rx_buf_tail] = Usb_read_endpoint_data(RX_EP, 8);

    if ((USB_RX_BUF_SIZE - 1) == usb_rx_buf_tail)
    {
        /* Reached the end of buffer, revert back to beginning of buffer. */
        usb_rx_buf_tail = 0x00;
    }
    else
    {
        usb_rx_buf_tail++;
    }
}


/**
 * @brief Services data transmission or reception on USB
 *
 * This function polls for usb for completion of reception or transmission of
 * a packet on USB.
 */
void usb_handler(void)
{
    usb_task();

    if (Is_usb_out_received(RX_EP))
    {
        uint8_t bytes_received = Usb_byte_count(RX_EP);
        Usb_reset_endpoint_fifo_access(RX_EP);
        while (bytes_received != 0)
        {
            usb_getc();
            --bytes_received;
        }

        Usb_ack_out_received_free(RX_EP);
    }

    if ((Is_usb_write_enabled(TX_EP) == true))
    {
        while (usb_tx_count > 0)
        {
            bool send_zlp = false;
            Usb_reset_endpoint_fifo_access(TX_EP);
            while ((usb_tx_count > 0) && (Is_usb_write_enabled(TX_EP) == true))
            {
                usb_putc();
            }
            if (Is_usb_write_enabled(TX_EP) == true)
            {
                send_zlp = true;
            }
            Usb_ack_in_ready_send(TX_EP);
            if (send_zlp)
            {
                while (Is_usb_write_enabled(TX_EP) == false)
                    /* wait */;
                Usb_ack_in_ready_send(TX_EP);
            }
        }
    }
}


void usb_sof_action(void)
{
    sof_cnt++;
}
#endif
/* EOF */
