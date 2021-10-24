/*
 * cbfifo.h - a fixed-size FIFO implemented via a circular buffer
 *
 * Author: Howdy Pierce, howdy.pierce@colorado.edu
 *
 */

#ifndef _CBFIFO_H_
#define _CBFIFO_H_

#include <stdlib.h>  // for size_t
#include <stdint.h>

#define SIZE (16)           /* Size of Buffer */

struct buffer_data{
  uint16_t charHandle;      /* Characteristic handle from GATTdb */
  size_t   bufferLength;    /* Length of buffer in bytes to send */
  uint8_t  buffer[5];       /* Actual buffer size. 5 bytes for htm and 2 for btn */
};

typedef struct{
struct buffer_data Data[SIZE];    /* Buffer */
uint8_t wrloc;                    /* Write Location (where to write next) */
uint8_t rdloc;                    /* Read Location (where to read from next) */
uint8_t isFull;                   /* Flag to indicate buffer full */
}Buffer;

/*
 * Enqueues data onto the FIFO, up to the limit of the available FIFO
 * capacity.
 *
 * Parameters:
 *	 fifo	  Fifo to be enqueued
 *   buf      Pointer to the data
 *   nbyte    Max number of bytes to enqueue
 *
 * Returns:
 *   The number of bytes actually enqueued, which could be 0. In case
 * of an error, returns -1.
 */
int cbfifo_enqueue(Buffer* Cbfifo, struct buffer_data *buf);


/*
 * Attempts to remove ("dequeue") up to nbyte bytes of data from the
 * FIFO. Removed data will be copied into the buffer pointed to by buf.
 *
 * Parameters:
 * 	 fifo	  Fifo to be dequeued
 *   buf      Destination for the dequeued data
 *   nbyte    Bytes of data requested
 *
 * Returns:
 *   The number of bytes actually copied, which will be between 0 and
 * nbyte.
 *
 * To further explain the behavior: If the FIFO's current length is 24
 * bytes, and the caller requests 30 bytes, cbfifo_dequeue should
 * return the 24 bytes it has, and the new FIFO length will be 0. If
 * the FIFO is empty (current length is 0 bytes), a request to dequeue
 * any number of bytes will result in a return of 0 from
 * cbfifo_dequeue.
 */
int cbfifo_dequeue(Buffer* Cbfifo, struct buffer_data *buf);


/*
 * Returns the number of bytes currently on the FIFO.
 *
 * Parameters:
 *   fifo The fifo whose length is required.
 *
 * Returns:
 *   Number of bytes currently available to be dequeued from the FIFO
 */
size_t cbfifo_length(Buffer* Cbfifo);


#endif // _CBFIFO_H_
