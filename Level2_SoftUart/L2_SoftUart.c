#include <stddef.h>
#include "STM32f10x.h"
#include "stm32f10x_tim.h"
#include "ring_buffer.h"
#include "L2_SoftUart.h"


/****  Local variables for the software UART **********************************/
static RingBuffer rx_b = {NULL,0,0,0}; 
static RingBuffer tx_b = {NULL,0,0,0}; 

//Bytes to be transmitted/received and their bits counters
static uint8_t tx_byte = 0;
static uint8_t rx_byte = 0;
static uint8_t tx_bit_counter = 0; 
static uint8_t rx_bit_counter = 0;
static uint8_t tx_parity_counter = 0; 
static uint8_t rx_parity_counter = 0;

// Fully received byte
static uint8_t recived_byte = 0;

// Indicator that a byte has been received
static uint8_t recived_byte_flags = 0;
#define RX_DONE_BIT    (1<<0)
#define PARITY_OK_BIT  (1<<1)

// UART bit duration in TIM2 timer ticks
static uint32_t bitime = 0;  
static parity_t pabit = none;

// Structures for reinitializing TIM2_CH4 (RX) in interrupt
static TIM_ICInitTypeDef  tim_input;
static TIM_OCInitTypeDef  tim_output;
static uint16_t capture = 0;


/*****  Initialization and shutdown of software UART *********/
// Use pin PA2 as TIM2_CH3 - softuart TX pin 
// Use pin PA3 as TIM2_CH4 - softuart RX pin 
static void init_softuart_gpio (void)
{ 
   GPIO_InitTypeDef  GPIO_InitStructure;
   /* Enable GPIOx (x = A) clocks */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
   
   // TXD pin
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
   
   // RXD pin
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   // Reset internal buffers
   tx_b.b = NULL; tx_b.head = tx_b.tail = tx_b.sz = 0;
   rx_b.b = NULL; rx_b.head = rx_b.tail = rx_b.sz = 0;
}

// Initialize TIM2 
static int init_tim2 (uint32_t baudrate, parity_t pbit)
{  
   // Bit duration in processor ticks for given UART speed
   uint32_t procbitime = SystemCoreClock / baudrate; 
  
   // Very high baudrate
   if (procbitime < PROC_TICKS_PER_BIT) return -1;
  
   uint32_t base_freq = 0;  // Timer count frequency for UART
   
   TIM_TimeBaseInitTypeDef timer;
  
   TIM_TimeBaseStructInit(&timer);
   timer.TIM_Prescaler = SystemCoreClock / 2000000 - 1; // timer clock ~ 2 MHz
   base_freq = SystemCoreClock / (timer.TIM_Prescaler + 1);
   bitime = base_freq / baudrate; 
   
   // Very low baudrate
   if(bitime > (0xFFFF / 3) * 2 - 100) return -2; 
   
   pabit=pbit;
   
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
   TIM_DeInit(TIM2); // Reset all previous settings

   TIM_TimeBaseInit(TIM2, &timer);	

   // Initialize output (softuart TX pin) - Channel 3
   TIM_OCStructInit(&tim_output);
   TIM_OC3Init(TIM2, &tim_output);
   TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable); // We can change compare register any time
   
   // Initialize input (softuart RX pin) - Channel 4
   TIM_ICStructInit(&tim_input);
   tim_input.TIM_Channel = TIM_Channel_4;
   tim_input.TIM_ICPolarity = TIM_ICPolarity_Falling;
   TIM_ICInit(TIM2, &tim_input); // Capture start bit moment on RX pin
   TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);
   
   // Enable interrupt for channel 4 (receiver)
   TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE); 
   
   // Start timer
   TIM_Cmd(TIM2, ENABLE);
   
   return 0;
} 

int soft_uart_open (uint32_t baudrate, parity_t pbit )
{     
   NVIC_InitTypeDef NVIC_InitStructure;
   
   int rc = init_tim2(baudrate, pbit);
   if (rc < 0 ) return rc;
   
   // Enable TIM2 global interrupt
   NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                  
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
   init_softuart_gpio();
   return 0;
}

void soft_uart_close ()
{ 
   // Disable TIM2 interrupt
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                  
   NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
   NVIC_Init(&NVIC_InitStructure);
   
   // Disable TIM2
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
   
   // Set TX pin to input mode 
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/*****  Working with single bytes for TX/RX *************/
// Use PA2 as TIM2_CH3 - softuart TX pin 
int soft_uart_send_byte (uint8_t data)
{ 
   // Byte still being transmitted. 1 start bit, 8 data bits, 1 stop bit
   if ((tx_bit_counter > 0) && (tx_bit_counter < 11)) return -1; 
   
   tx_byte = data;
   tx_bit_counter = 1; tx_parity_counter = 0;
   GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET); // start bit
   uint16_t counter = TIM2->CNT;
   counter += bitime;
   TIM_SetCompare3(TIM2, counter);
   TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE); // Enable TX interrupt
   return 0;
}

// Use PA3 as TIM2_CH4 - softuart RX pin 
// Receives one byte.
// Returns:
//   1  - no new byte available 
//   0  - on success
//  -1  - new byte available with patity Error
int soft_uart_receive_byte(uint8_t *data)
{ 
   if (recived_byte_flags & RX_DONE_BIT)
     { *data = recived_byte; 
       if (recived_byte_flags & PARITY_OK_BIT)
        { recived_byte_flags = 0; return 0;}
       recived_byte_flags = 0;
       return  -1; 
     }
   else return 1;
}


/*****  Working with RX and TX ring buffers ************************************/
int soft_uart_init_buffers (uint8_t * tx_bf, uint16_t tx_sz, uint8_t * rx_bf, uint16_t rx_sz)
{
   if (tx_sz < 2) return -1;
   if (rx_sz < 2) return -2;
   RingBuffer_Init(&tx_b, tx_bf, tx_sz);
   RingBuffer_Init(&rx_b, rx_bf, rx_sz);
   return 0;
}

// Returns number of bytes added to TX buffer 
int soft_uart_send_buffer (const uint8_t * data, uint16_t len)
{ 
   uint16_t bytes = RingBuffer_PutBytes(&tx_b, data, len);
   // No current transmission - start it
   if (soft_uart_is_tx_complete())
   { 
      uint8_t tmp_byte;
      if (RingBuffer_Get(&tx_b, &tmp_byte))
         soft_uart_send_byte(tmp_byte);
   }
   return bytes;
}

// Returns number of bytes read from RX buffer 
int soft_uart_receive_buffer (uint8_t * data, uint16_t len)
{
   return RingBuffer_GetBytes(&rx_b, data, len);
}


/*****   Check TX/RX completion   **********************************************/
bool soft_uart_is_tx_complete ()
{
   return (tx_bit_counter > 0 && tx_bit_counter < 12) ? false : true; 
}

bool soft_uart_is_uart_rx_complite ()
{
   return (recived_byte_flags & RX_DONE_BIT) ? false : true; 
}


/******   TIM2 interrupt handler **********************************************/

static inline void byte_received ()
 {  recived_byte = rx_byte;   
    recived_byte_flags |= RX_DONE_BIT;
    // Fill the RX ring buffer with valid bytes only
    if ((rx_b.b)&&(recived_byte_flags & PARITY_OK_BIT)) 
                        RingBuffer_Put(&rx_b, recived_byte); 
    TIM_ICInit(TIM2, &tim_input);
 }

void TIM2_IRQHandler (void)
{ 
   /**  Transmit byte. Use PA2 as  TIM2_CH3 - softuart TX pin ***/
   if (TIM_GetFlagStatus(TIM2, TIM_FLAG_CC3))
   {
      capture = TIM_GetCapture3(TIM2);
      TIM_SetCompare3(TIM2, capture + bitime);
      
      if (tx_bit_counter < 9) // sending data bits
      {
         GPIO_WriteBit(GPIOA, GPIO_Pin_2, (tx_byte & 1) ? Bit_SET : Bit_RESET);
         if(tx_byte & 1) tx_parity_counter++;
         tx_byte >>= 1;
      }
      else if (tx_bit_counter == 9) // sending parity bit
       { switch (pabit)
          { case odd:  
                if(tx_parity_counter & 1) GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
                else                      GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
                break;
            case even: 
                if(tx_parity_counter & 1) GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
                else                      GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
                break;
            case mark:
                 GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
                 break;
            case space:
                 GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
                 break;
            case none:
            default: // sending stop bit
                 tx_bit_counter=10; 
                 GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
                 break;
          }
          
       }
      else if (tx_bit_counter == 10) // sending stop bit
      {
         GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
      }
      else // all bits transmitted
      {  tx_parity_counter = 0;
         if (tx_b.b && RingBuffer_Get(&tx_b, &tx_byte))
         {
            tx_bit_counter = 0; 
            GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET); // start bit
         }
         else
         {
            TIM_ITConfig(TIM2, TIM_IT_CC3, DISABLE);
         }
      }
      tx_bit_counter++;
      TIM_ClearFlag(TIM2, TIM_FLAG_CC3);
   }
   
   /**  Receive byte. Use PA3 as  TIM2_CH4 - softuart RX pin **/
   if (TIM_GetFlagStatus(TIM2, TIM_FLAG_CC4))
   {
      capture = TIM_GetCapture4(TIM2);  
      // Receiving start bit 
      if (rx_bit_counter == 0)
      {  
         rx_parity_counter = 0;
         TIM_OC4Init(TIM2, &tim_output); // Switch to output compare
         TIM_SetCompare4(TIM2, capture + bitime * 3 / 2); // Set time for first data bit
         rx_bit_counter++;
         
      }
      else if (rx_bit_counter < 9) // Receiving data bits
      {
         TIM_SetCompare4(TIM2, capture + bitime);
         rx_byte >>= 1;
         if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)) 
           { rx_byte |= 0x80; rx_parity_counter++;} 
         rx_bit_counter++;
      }
     else if (rx_bit_counter == 9) // Receiving parity bit or stop bit
      {  uint8_t pbit_rx =  (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3));
         if (pbit_rx) rx_parity_counter++;
         switch (pabit)
          { case odd:  
                if(rx_parity_counter & 1) recived_byte_flags |= PARITY_OK_BIT;
                else                      recived_byte_flags &= ~PARITY_OK_BIT;
                TIM_SetCompare4(TIM2, capture + bitime);
                break;
            case even: 
                if(rx_parity_counter & 1) recived_byte_flags &= ~PARITY_OK_BIT;
                else                      recived_byte_flags |= PARITY_OK_BIT;
                TIM_SetCompare4(TIM2, capture + bitime);
                break;
            case mark:
                 if (pbit_rx) recived_byte_flags |= PARITY_OK_BIT;
                 else         recived_byte_flags &= ~PARITY_OK_BIT;
                 TIM_SetCompare4(TIM2, capture + bitime);
                 break;
            case space:
                 if (pbit_rx) recived_byte_flags &= ~PARITY_OK_BIT;
                 else         recived_byte_flags |= PARITY_OK_BIT;
                 TIM_SetCompare4(TIM2, capture + bitime);
                 break;
            case none:  // Half stop bit - Switch back to input capture, wait for start bit
            default: 
                 recived_byte_flags |= PARITY_OK_BIT;
                 rx_bit_counter = -1;
                 byte_received();
                 break;
          } 
         rx_bit_counter++;
      }
      else // Receiving stop bit. Half stop bit - Switch back to input capture, wait for start bit
      {  rx_bit_counter = 0;
         byte_received();
      }
      TIM_ClearFlag(TIM2, TIM_FLAG_CC4);
   }
}

/******************************************************************************/

