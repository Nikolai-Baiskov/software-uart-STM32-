
#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer.h"

//#define TEST_RB_ATOM

#ifdef  TEST_RB_ATOM
 #define DISIRQ   NVIC_DisableIRQ(TIM2_IRQn);
 #define ENIRQ    NVIC_EnableIRQ(TIM2_IRQn);
#else
 #define DISIRQ   ;
 #define ENIRQ    ;
#endif

void RingBuffer_Init(RingBuffer *rb, uint8_t* buf, uint16_t size) {
    DISIRQ
    rb->b = buf;
    rb->sz = size;
    rb->head = 0;
    rb->tail = 0;
    ENIRQ
}

bool RingBuffer_IsEmpty(RingBuffer *rb) {
    bool result;
    DISIRQ
    result = (rb->head == rb->tail);
    ENIRQ
    return result;
}

bool RingBuffer_IsFull(RingBuffer *rb) {
    bool result;
    DISIRQ
    result = ((rb->head + 1) % rb->sz) == rb->tail;
    ENIRQ
    return result;
}

uint16_t RingBuffer_Count(RingBuffer *rb) {
    uint16_t count;
    DISIRQ
    if (rb->head >= rb->tail)
        count = rb->head - rb->tail;
    else
        count = rb->sz - rb->tail + rb->head;
    ENIRQ
    return count;
}

bool RingBuffer_Put(RingBuffer *rb, uint8_t data) {
    bool success = false;
    DISIRQ
    uint16_t next = (rb->head + 1) % rb->sz;
    if (next != rb->tail) {
        rb->b[rb->head] = data;
        rb->head = next;
        success = true;
    }
    ENIRQ
    return success;
}

bool RingBuffer_Get(RingBuffer *rb, uint8_t *data) {
    bool success = false;
    DISIRQ
    if (rb->head != rb->tail) {
        *data = rb->b[rb->tail];
        rb->tail = (rb->tail + 1) % rb->sz;
        success = true;
    }
    ENIRQ
    return success;
}

uint16_t RingBuffer_PutBytes(RingBuffer *rb, const uint8_t *data, uint16_t length) {
    uint16_t i;
    
    for (i = 0; i < length; i++) {
        DISIRQ 
        uint16_t next = (rb->head + 1) % rb->sz;
        if (next == rb->tail) {
            ENIRQ
            break;  // Buffer is full
        }
        rb->b[rb->head] = data[i];
        rb->head = next;
        ENIRQ
    }
    
    return i;
}

uint16_t RingBuffer_GetBytes(RingBuffer *rb, uint8_t *data, uint16_t length) {
    uint16_t i;
    
    for (i = 0; i < length; i++) {
        DISIRQ 
        if (rb->head == rb->tail) {
            ENIRQ
            break;  // Buffer is empty
        }
        data[i] = rb->b[rb->tail];
        rb->tail = (rb->tail + 1) % rb->sz;
        ENIRQ
    }
    return i;
}
