/**
 * @file sio_handler.h
 *
 * @brief This file contains the prototypes for UART related functions.
 *
 * $Id: dual_sio_handler.h 32623 2012-07-12 14:19:18Z sschneid $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2012, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef SIO_HANDLER_H
#define SIO_HANDLER_H

/* === Includes ============================================================= */

#include <stdio.h>
#include <pal_types.h>

/* === Macros =============================================================== */

#ifdef UART0
#ifdef DUAL_UART0
#define SIO_DATA_CHANNEL      SIO_0
#else
#define SIO_DEBUG_CHANNEL     SIO_0
#endif
#endif
#ifdef UART1
#ifdef DUAL_UART1
#define SIO_DATA_CHANNEL      SIO_1
#else
#define SIO_DEBUG_CHANNEL     SIO_1
#endif
#endif
#ifdef UART2
#ifdef DUAL_UART2
#define UART2
#define SIO_DATA_CHANNEL      SIO_3
#else
#define SIO_DEBUG_CHANNEL     SIO_3
#endif
#endif
#ifdef UART3
#ifdef DUAL_UART3
#define UART3
#define SIO_DATA_CHANNEL      SIO_4
#else
#define SIO_DEBUG_CHANNEL     SIO_4
#endif
#endif
#ifdef USB0
#ifdef DUAL_USB0
#define USB0
#define SIO_DATA_CHANNEL      SIO_2
#else
#define SIO_DEBUG_CHANNEL     SIO_2
#endif
#endif

/* Function aliases allowing IAR and GCC functions use the same way */
#if ((defined __ICCAVR__) || (defined __ICCARM__) || (defined __ICCAVR32__))
#define sio_putchar(data)           _sio_putchar(data)
#define sio_getchar()               _sio_getchar()
#define sio_getchar_nowait()        _sio_getchar_nowait()
#define sio_dataputchar(data)       _sio_dataputchar(data)
#define sio_datagetchar()           _sio_datagetchar()
#define sio_datagetchar_nowait()    _sio_datagetchar_nowait()
#define fdevopen(a,b)           /* IAR does not use the fdevopen - the __write() (or __read()) must exist instead (here in file write.c) */
#else
#define sio_putchar(data)           _sio_putchar(data, NULL)
#define sio_getchar()               _sio_getchar(NULL)
#define sio_getchar_nowait()        _sio_getchar_nowait(NULL)
#define sio_dataputchar(data)       _sio_dataputchar(data, NULL)
#define sio_datagetchar()           _sio_datagetchar(NULL)
#define sio_datagetchar_nowait()    _sio_datagetchar_nowait(NULL)
#if (PAL_GENERIC_TYPE == ARM7)
#define fdevopen(a,b)
#endif
#endif

/* === Types ================================================================ */


/* === Externals ============================================================ */


/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif


#if ((defined __ICCAVR__) || (defined __ICCARM__) || (defined __ICCAVR32__))
    /* Functions which use the uart channel for user interaction or debug reasons */
    int _sio_putchar(char data);
    int _sio_getchar(void);
    int _sio_getchar_nowait(void);
    /* Functions which use the uart channel connected to another board for board2board communication*/
    int _sio_dataputchar(char data);
    int _sio_datagetchar(void);
    int _sio_datagetchar_nowait(void);
#else
    /* Functions which use the uart channel for user interaction or debug reasons */
    int _sio_putchar(char data, FILE *dummy);
    int _sio_getchar(FILE *dummy);
    int _sio_getchar_nowait(FILE *dummy);
    /* Functions which use the uart channel connected to another board for board2board communication*/
    int _sio_dataputchar(char data, FILE *dummy);
    int _sio_datagetchar(FILE *dummy);
    int _sio_datagetchar_nowait(FILE *dummy);
#endif
    /* Function which use the uart channel for user interaction or debug reasons */
    void sio_binarydbgwrite(uint8_t *d, int16_t sz);
    /* Functions which use the uart channel connected to another board for board2board communication*/
    void sio_binarywrite(uint8_t *d, int16_t sz);
    int sio_binaryread(uint8_t *d, uint8_t sz);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SIO_HANDLER_H */
/* EOF */
