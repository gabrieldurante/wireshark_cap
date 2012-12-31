/**
 * @file pal_usb.h
 *
 * @brief PAL usb internal functions prototypes
 *
 * This header contains the function prototypes for transmit,
 * receive functions and macros used in USB module.
 *
 * $Id: pal_usb.h 24665 2011-01-03 12:49:03Z yogesh.bellan $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2010, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> LICENSE.txt
 */

/* Prevent double inclusion */
#ifndef PAL_USB_H
#define PAL_USB_H

/* === Includes ============================================================= */


/* === Types ================================================================ */


/* === Externals ============================================================ */


/* === Macros =============================================================== */

#ifdef USB0

#define NB_MS_BEFORE_FLUSH   (50)
/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

void sio_usb_init(void);

uint8_t sio_usb_tx(uint8_t *data, uint8_t length);

uint8_t sio_usb_rx(uint8_t *data, uint8_t max_length);

void usb_handler(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* USB0 */

#endif /* PAL_USB_H */
/* EOF */
