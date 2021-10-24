/************************************************************************
 * Copyright (C)​ 2021 by Dhruv Mehta
 *
 * ​Redistribution, ​​modification ​​or ​​use ​​of ​​this ​​software ​​in​​source ​​or ​​binary
 * forms ​​is ​​permitted ​​as ​​long​​ as ​​the ​​files​​ maintain ​​this ​​copyright.
 *
 * Users ​​are​​ ​​permitted ​​to​​ modify ​​this ​​and ​​use​​ it ​​to​​ learn​​ about​​ the​​ field
 * ​​of ​​embedded​​ ​​software.
 *
 * ​Dhruv Mehta and the University of Colorado are not liable for any misuse
 *  of this material​​.
 *
 ************************************************************************/

/* @file    cbfifo.c
 *
 * @brief   Contains an implementation for a circular buffer FIFO.
 *
 * @tools   GCC compiler v8.1.0 / Visual Studio Code IDE
 * @author  Dhruv Mehta
 * @date    04/04/2021
 * @version 1.1
 *
 ************************************************************************/
#include <stdint.h>
#include "cbfifo.h"


int cbfifo_enqueue(Buffer* Cbfifo, struct buffer_data *buf)
{
    if(buf == NULL || Cbfifo == NULL)
        return -1;                              /* Input buffer is NULL */

    if((Cbfifo->wrloc == Cbfifo->rdloc) && (Cbfifo->isFull == 1))
      return -1;

    Cbfifo->Data[Cbfifo->wrloc] = *buf;        /* Write to buffer */
    Cbfifo->wrloc = (Cbfifo->wrloc + 1) & (SIZE - 1); /* Wrapped addition of wrloc */

    /* If ptrs match, buffer is full */
    if(Cbfifo->wrloc == Cbfifo->rdloc)
        Cbfifo->isFull = 1;

    return 0;
}

int cbfifo_dequeue(Buffer* Cbfifo, struct buffer_data *buf)
{
    if(buf == NULL || Cbfifo == NULL)
        return -1;                                /* Input buffer is NULL */

    if((Cbfifo->rdloc == Cbfifo->wrloc) && (Cbfifo->isFull == 0))
      return -1;

    *buf = Cbfifo->Data[Cbfifo->rdloc];        /* Read from buffer */
    Cbfifo->rdloc = (Cbfifo->rdloc + 1) & (SIZE - 1);  /* Wrapped addition of rdloc */

    if(Cbfifo->isFull == 1)
      Cbfifo->isFull = 0;

    return 0;
}

size_t cbfifo_length(Buffer* Cbfifo)
{
    return ((Cbfifo->wrloc - Cbfifo->rdloc) & (SIZE - 1)); /* Wrapped subtraction of wrloc and rdloc */
}
