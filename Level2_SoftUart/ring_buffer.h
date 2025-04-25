#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t * b;   // pointer to the buffer
    uint16_t  sz;  // buffer size
    uint16_t head;
    uint16_t tail;
} RingBuffer;

// Initialize the buffer
void RingBuffer_Init(RingBuffer *rb, uint8_t* buf, uint16_t size);

// Add a byte to the buffer
bool RingBuffer_Put(RingBuffer *rb, uint8_t data);

// Read a byte from the buffer
bool RingBuffer_Get(RingBuffer *rb, uint8_t *data);

// Check if the buffer is empty
bool RingBuffer_IsEmpty(RingBuffer *rb);

// Check if the buffer is full
bool RingBuffer_IsFull(RingBuffer *rb);

// Get the number of bytes in the buffer
uint16_t RingBuffer_Count(RingBuffer *rb);

// Add multiple bytes to the buffer
uint16_t RingBuffer_PutBytes(RingBuffer *rb, const uint8_t *data, uint16_t length);

// Read multiple bytes from the buffer
uint16_t RingBuffer_GetBytes(RingBuffer *rb, uint8_t *data, uint16_t length);

#endif // RING_BUFFER_H