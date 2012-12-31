/**
 * @file app_config.h
 *
 * @brief These are application-specific resources which are used
 *        in the application in addition to the underlaying stack.
 *
 * $Id: app_config.h 33712 2012-12-03 11:13:35Z yogesh.bellan $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* === Includes ============================================================= */

#include "stack_config.h"
#include "return_val.h"
#include <stdbool.h>

/* === Macros =============================================================== */

#if (NUMBER_OF_TOTAL_STACK_TIMERS == 0)
#define APP_FIRST_TIMER_ID          (0)
#else
#define APP_FIRST_TIMER_ID          (LAST_STACK_TIMER_ID + 1)
#endif

/* === Types ================================================================ */

/* Timer ID's used by the Application */
#if 1 /* Unused */
typedef enum
{
    /* App Timers start from APP_FIRST_TIMER_ID */
    TX_WS = (APP_FIRST_TIMER_ID)
} SHORTENUM app_timer_t;
#endif

#define NUMBER_OF_APP_TIMERS        (0)

#define TOTAL_NUMBER_OF_TIMERS      (NUMBER_OF_APP_TIMERS + NUMBER_OF_TOTAL_STACK_TIMERS)

/* Additional buffers used by the application */
#define NUMBER_OF_LARGE_APP_BUFS    (15)
#define NUMBER_OF_SMALL_APP_BUFS    (0)

#define TOTAL_NUMBER_OF_LARGE_BUFS  (NUMBER_OF_LARGE_APP_BUFS + NUMBER_OF_LARGE_STACK_BUFS)
#define TOTAL_NUMBER_OF_SMALL_BUFS  (NUMBER_OF_SMALL_APP_BUFS + NUMBER_OF_SMALL_STACK_BUFS)

#define TOTAL_NUMBER_OF_BUFS        (TOTAL_NUMBER_OF_LARGE_BUFS + TOTAL_NUMBER_OF_SMALL_BUFS)

/*
 * USB transmit buffer size
 */
#define USB_TX_BUF_SIZE           (250)//  (127+15)/*aMaxPHYPacketSize + App Header */

/*
 * USB receive buffer size
 */
#define USB_RX_BUF_SIZE            (250)// (127+15)/*aMaxPHYPacketSize + App Header */

/*
 * USB-specific definitions
 */

/*
 * USB Vendor ID (16-bit number)
 */
#define USB_VID                 0x03EB /* Atmel's USB vendor ID */

/*
 * USB Product ID (16-bit number)
 */
#ifdef FAKE_STK541
#define USB_PID                 0x2109  /* RZ USB stick product ID */
#else
#define USB_PID                 0x2115  /* RZ USB stick Sniffer product ID */
#endif

/*
 * USB Release number (BCD format, two bytes)
 */
#define USB_RELEASE             { 0x00, 0x01 } /* 01.00 */

/*
 * Maximal number of UTF-16 characters used in any of the strings
 * below.  This is only used for compilers that cannot handle the
 * initialization of flexible array members within structs.
 */
#ifdef FAKE_STK541
#define USB_STRING_SIZE         22
#else
#define USB_STRING_SIZE         24
#endif

/*
 * String representation for the USB vendor name.
 */
#define USB_VENDOR_NAME L"ATMEL"

/*
 * String representation for the USB product name.
 */
#ifdef FAKE_STK541
#define USB_PRODUCT_NAME L"STK541 USB Serial Port"
#else
#define USB_PRODUCT_NAME L"Atmel RZUSBStick Sniffer"
#endif

/*
 * UART transmit buffer size
 */
#define UART_MAX_TX_BUF_LENGTH      (10)

/*
 * UART receive buffer size
 */
#define UART_MAX_RX_BUF_LENGTH      (10)

/* Offset of IEEE address storage location within EEPROM */
#define EE_IEEE_ADDR                (0)

#ifdef PERYTON_SNIFFER
#define MAX_GUI_CMD_LENGTH          (140)
#else
#define MAX_GUI_CMD_LENGTH          (20)
#endif

#ifndef NB_MS_BEFORE_FLUSH
#define NB_MS_BEFORE_FLUSH          (50)
#endif

/* === Externals ============================================================ */
#define NUL     0x30  //0x00x
#define SOH     0x31   //0x31
#define EOT     0x34   //0x34

#define DATA_PACKET_ID          0x31
#define CHANNEL_SELECT_ID       0x32
#define START_CAPTURE_ID        0x33
#define STOP_CAPTURE_ID         0x34

/* === Prototypes =========================================================== */
void sniffer_handle_received_frame(uint8_t *received_frame);
void sniffer_config_select(uint8_t Cmd,uint8_t Channel);
void sniffer_handle_tx_done(retval_t status);
uint8_t sniffer_sio_tx(uint8_t *data, uint8_t length);
extern void clear_buffer();
extern bool buffer_write_free;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* APP_CONFIG_H */
/* EOF */
