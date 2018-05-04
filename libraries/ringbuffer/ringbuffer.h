/*
    Ring (circular) buffer implementation.
    Author: Kalim Moghul (kfm9 at cornell dot edu)
    Invariants:
        head==tail iff buffer is empty.
        If buffer is not empty, head points at next valid element to be consumed.
        tail always points at the next empty element.
            => There is always one unused element in a full buffer.
            => length must be greater than 1
*/

#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#include <stdint.h>

typedef uint8_t rbdatatype;				//define storage type of ringbuffer

typedef struct ringbuf {
	rbdatatype *buf;					//points to data array
	uint32_t length;					//length of data array
	uint8_t head, tail;					//producer and consumer indices
} RingBuffer;

//initializes the given ringbuffer with the supplied array and its length
void RingBuffer_Init(RingBuffer *rb, rbdatatype *array, uint32_t length);

//returns boolean true if the ringbuffer is empty, false otherwise
uint8_t RingBuffer_IsEmpty(RingBuffer *rb);

//returns boolean true if the ringbuffer is full, false otherwise
uint8_t RingBuffer_IsFull(RingBuffer *rb);

//consumes an element from the buffer
//returns NULL if buffer is empty or a pointer to the array element otherwise
rbdatatype* RingBuffer_Get(RingBuffer *rb);

//puts an element into the buffer
//returns 0 if buffer is full, otherwise returns 1
uint8_t RingBuffer_Put(RingBuffer *rb, rbdatatype c);

#endif /* __RINGBUFFER_H */
