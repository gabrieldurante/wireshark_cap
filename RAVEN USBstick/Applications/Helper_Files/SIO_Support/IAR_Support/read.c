/**
 * @file read.c
 *
 * @brief Implements READ function used by standard library.
 *
 * This is a template implementation of the "__read" function used by
 * the standard library.  Replace it with a system-specific
 * implementation.
 *
 * The "__read" function reads a number of bytes, at most "size" into
 * the memory area pointed to by "buffer".  It returns the number of
 * bytes read, 0 at the end of the file, or _LLIO_ERROR if failure
 * occurs.
 *
 * The template implementation below assumes that the application
 * provides the function "MyLowLevelGetchar".  It should return a
 * character value, or -1 on failure.
 *
 * $Id: read.c 32623 2012-07-12 14:19:18Z sschneid $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === Includes ============================================================ */

#include <yfuns.h>
#include <stdint.h>
#include "sio_handler.h"


_STD_BEGIN

#if ((defined __ICCAVR__) || (defined __ICCARM__) || (defined __ICCAVR32__))
/*  #pragma module_name = "?__read" */
#endif  /* ((defined __ICCAVR__) || (defined __ICCARM__) || (defined __ICCAVR32__)) */

/* === Macros ============================================================== */


/* === Globals ============================================================= */


/* === Prototypes ========================================================== */


/* === Implementation ====================================================== */

int MyLowLevelGetchar(void)
{
    return sio_getchar();
}

size_t __read(int handle, unsigned char *buffer, size_t size)
{
    int nChars = 0;

    /*
     * This template only reads from "standard in", for all other file
     * handles it returns failure.
     */
    if (handle != _LLIO_STDIN)
    {
        return _LLIO_ERROR;
    }

    for (/* Empty */; size > 0; --size)
    {
        int c = MyLowLevelGetchar();
        if (c < 0)
        {
            break;
        }

        *buffer++ = c;
        ++nChars;
    }

    return nChars;

}

_STD_END
/* EOF */
