/********************************************************************************
  * @file    main.c 
  * @author  Nikolai
  * @version V1.0.0
  * @date    04/24/2025
  * @brief   Main program body.
  ******************************************************************************/

#include "L2_SoftUart.h"

uint8_t rec_byte = 0;

// Test buffers
uint8_t sfuart_tx [50];
uint8_t sfuart_rx [50];

//Test UART parameters
uint32_t baudrate = 19200;
parity_t parity   = none;


int main(void)
{
  
  int rc = soft_uart_open (baudrate, parity);
  if (rc < 0) return rc;

  /** Working with individual bytes **/
  uint8_t ch = '0';
  uint16_t rx_counter = 0;

  // Transmit and receive
  for (int i = 0; i < 10; i++)
  {
    soft_uart_send_byte(ch++);

    while(!soft_uart_is_tx_complete())
    {
      if (soft_uart_receive_byte(&rec_byte) == 0)
        rx_counter++;
    }
  }

  // Only receive
  while (rx_counter < 20)
  {
    if (soft_uart_receive_byte(&rec_byte) == 0)
      rx_counter++;
  }

  /*** Working via buffer ************/
  soft_uart_init_buffers(sfuart_tx, sizeof(sfuart_tx), sfuart_rx, sizeof(sfuart_rx));

  uint8_t tmp_buf[sizeof(sfuart_rx)];
  rx_counter = 0;
  uint16_t len = 0, tx_counter = 0;

  // Transmit what we receive
  while(1)
  {
    len = soft_uart_receive_buffer(tmp_buf, sizeof(tmp_buf));

    if (len > 0)
    {
      rx_counter += len;
      tx_counter += soft_uart_send_buffer(tmp_buf, len);
    }

    if (tx_counter >= 1000) break;
  }

  // Wait for the transmission to complete
  while(!soft_uart_is_tx_complete());

  soft_uart_close();

  return 0;
}

