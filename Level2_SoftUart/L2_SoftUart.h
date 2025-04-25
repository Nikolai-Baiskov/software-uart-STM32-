#ifndef L2_SOFTUART_H
#define L2_SOFTUART_H

#include <stdint.h>
#include <stdbool.h>


/**** Minimum allowed value of CPU ticks per 1 bit.
     Determined experimentally and depends on the baudrate:
     - higher baudrate > smaller PROC_TICKS_PER_BIT
     - heavier MCU load > higher PROC_TICKS_PER_BIT ****/

#define PROC_TICKS_PER_BIT (1000)



typedef enum 
 { none  = 0,
   odd   = 1,
   even  = 2,
   mark  = 3,
   space = 4
 } parity_t;
   

/*****  Software UART open/close  *********************/
// Opens the software UART.
// Returns:
//   0  - on success
//  -1  - baudrate is too high
//  -2  - baudrate is too low
int soft_uart_open(uint32_t baudrate, parity_t pbit);

// Closes the software UART and frees resources.
void soft_uart_close(void);

/***   Single-byte transmission and reception  *************/
// Transmits one byte.
// Returns:
//   0  - on success
//  -1  - previous byte is still being transmitted
int soft_uart_send_byte(uint8_t data);

// Receives one byte.
// Returns:
//   1  - no new byte available 
//   0  - on success
//  -1  - new byte available with patity Error

int soft_uart_receive_byte(uint8_t *data);

/*****  Working with TX and RX ring buffers  ****************/
// Initializes TX and RX ring buffers.
// Returns 0 on success, or a negative error code.
int soft_uart_init_buffers(uint8_t *tx_buffer, uint16_t tx_size, uint8_t *rx_buffer, uint16_t rx_size);

// Sends multiple bytes using the TX buffer.
// Returns number of bytes successfully buffered.
int soft_uart_send_buffer(const uint8_t *data, uint16_t length);

// Reads multiple bytes from the RX buffer.
// Returns number of bytes successfully read.
int soft_uart_receive_buffer(uint8_t *data, uint16_t length);

/******* Status checks. Has transmission or reception completed?  ***********/
bool soft_uart_is_tx_complete(void);
bool soft_uart_is_rx_complete(void);

#endif



