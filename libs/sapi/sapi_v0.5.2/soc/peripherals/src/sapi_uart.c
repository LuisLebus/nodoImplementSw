/* Copyright 2014, Pablo Ridolfi (UTN-FRBA).
 * Copyright 2014, Juan Cecconi.
 * Copyright 2015-2017, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Date: 2016-02-26 */

/*==================[inclusions]=============================================*/

#include "sapi_uart.h"

#include "string.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

typedef struct {
	LPC_USART_T* 	addr;
	pinConfig_t		tx;
	pinConfig_t		rx;
	IRQn_Type       irqAddr;
} uartModule_t;

typedef struct {
	callBackFuncPtr_t 	txIsrCallback;
	void* 				txIsrCallbackParams;
	callBackFuncPtr_t 	rxIsrCallback;
	void* 				rxIsrCallbackParams;
} uartCallback_t;

/*==================[internal data declaration]==============================*/
static const uartModule_t uartModule[] = {
		// { uartAddr, { txPort, txpin, txfunc }, { rxPort, rxpin, rxfunc }, uartIrqAddr },

		// UART0 (GPIO0_2 = U0_TXD, GPIO0_3 = U0_RXD)
		{ LPC_UART0, { FUNC1, 0, 2 }, { FUNC1, 0, 3 }, UART0_IRQn },	// 0
		// UART1 (GPIO0_15 = U1_TXD, GPIO0_16 = U1_RXD)
		{ LPC_UART1, { FUNC1, 0,15 }, { FUNC1, 0,16 }, UART1_IRQn }, 	// 1
		// UART2 (GPIO2_8 = U2_TXD, GPIO2_9 = U2_RXD)
		{ LPC_UART2, { FUNC2, 2, 8 }, { FUNC2, 2, 9 }, UART2_IRQn }, 	// 2
		// UART3/DEBUG (GPIO0_0 = U3_TXD, GPIO0_1 = U3_RXD)
		{ LPC_UART3, { FUNC2, 0, 0 }, { FUNC2, 0, 1 }, UART3_IRQn } 	// 3
};

static uartCallback_t uartCallback[] = {
		// { txCallback, txCallbackParam, rxCallback, rxCallbackParam },
		{ 0, NULL, 0, NULL },	// 0
		{ 0, NULL, 0, NULL },	// 1
		{ 0, NULL, 0, NULL },	// 2
		{ 0, NULL, 0, NULL }	// 3
};

/*==================[internal functions declaration]=========================*/
#ifdef SAPI_USE_INTERRUPTS
static void uartProcessIRQ( uartMap_t uart );

/*==================[internal functions definition]==========================*/

static void uartProcessIRQ( uartMap_t uart )
{
	uint8_t status = Chip_UART_ReadLineStatus( uartModule[uart].addr );

	// Rx Interrupt
	if(status & UART_LSR_RDR) { // uartRxReady
		// Execute callback
		if(uartCallback[uart].rxIsrCallback != 0)
			(*uartCallback[uart].rxIsrCallback)(0);
	}

	// Tx Interrupt
	if( ( status & UART_LSR_THRE ) && // uartTxReady
			( Chip_UART_GetIntsEnabled( uartModule[uart].addr ) & UART_IER_THREINT ) ) {

		// Execute callback
		if(uartCallback[uart].txIsrCallback != 0)
			(*uartCallback[uart].txIsrCallback)(0);
	}
}
#endif /* SAPI_USE_INTERRUPTS */

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
#ifdef SAPI_USE_INTERRUPTS

// UART Global Interrupt Enable/Disable
void uartInterrupt( uartMap_t uart, bool_t enable )
{
   if( enable ) {
      // Interrupt Priority for UART channel
      NVIC_SetPriority( uartModule[uart].irqAddr, 5 ); // FreeRTOS Requiere prioridad >= 5 (numero mas alto, mas baja prioridad)
      // Enable Interrupt for UART channel
      NVIC_EnableIRQ( uartModule[uart].irqAddr );
   } else {
      // Disable Interrupt for UART channel
      NVIC_DisableIRQ( uartModule[uart].irqAddr );
   }
}

// UART Interrupt event Enable and set a callback
void uartCallbackSet( uartMap_t uart, uartEvents_t event, callBackFuncPtr_t callbackFunc, void* callbackParam )
{   
   uint32_t intMask;

   switch(event)
   {

      case UART_RECEIVE:
         // Enable UART Receiver Buffer Register Interrupt
         intMask = UART_IER_RBRINT | UART_IER_RLSINT;
         
         if( callbackFunc != 0 )
         {
        	 // Set callback
        	 uartCallback[uart].rxIsrCallback = callbackFunc;
        	 uartCallback[uart].rxIsrCallbackParams = callbackParam;
         } else
         {
            return;
         }
         break;
      case UART_TRANSMITER_FREE:
         // Enable THRE irq (TX)
         intMask = UART_IER_THREINT;

         if(callbackFunc != 0)
         {
        	 // Set callback
        	 uartCallback[uart].txIsrCallback = callbackFunc;
        	 uartCallback[uart].txIsrCallbackParams = callbackParam;
         }
         else
         {
            return;
         }
         break;
      default:
         return;
   }

   // Enable UART Interrupt
   Chip_UART_IntEnable(uartModule[uart].addr, intMask);
}
                 
// UART Interrupt event Disable
void uartCallbackClr( uartMap_t uart, uartEvents_t event )
{
   uint32_t intMask;

   switch(event)
   {
      case UART_RECEIVE:
         // Enable UART Receiver Buffer Register Interrupt
         intMask = UART_IER_RBRINT | UART_IER_RLSINT;
         break;
      case UART_TRANSMITER_FREE:
         // Enable THRE irq (TX)
         intMask = UART_IER_THREINT;
         break;
      default:
         return;
   }

   // Disable UART Interrupt
   Chip_UART_IntDisable(uartModule[uart].addr, intMask);
}
 
// UART Set Pending Interrupt. Useful to force first character in tx transmission
void uartSetPendingInterrupt(uartMap_t uart)
{
   NVIC_SetPendingIRQ(uartModule[uart].irqAddr);
}

// UART Clear Pending Interrupt.
void uartClearPendingInterrupt(uartMap_t uart)
{
   NVIC_ClearPendingIRQ(uartModule[uart].irqAddr);
} 
#endif /* SAPI_USE_INTERRUPTS */


// Return TRUE if have unread data in RX FIFO
bool_t uartRxReady( uartMap_t uart )
{
   return Chip_UART_ReadLineStatus( uartModule[uart].addr ) & UART_LSR_RDR;
}
// Return TRUE if have space in TX FIFO
bool_t uartTxReady( uartMap_t uart )
{
   return Chip_UART_ReadLineStatus( uartModule[uart].addr ) & UART_LSR_THRE;
}
// Read from RX FIFO
uint8_t uartRxRead( uartMap_t uart )
{
   return Chip_UART_ReadByte( uartModule[uart].addr );
}
// Write in TX FIFO
void uartTxWrite( uartMap_t uart, const uint8_t value )
{
   Chip_UART_SendByte( uartModule[uart].addr, value );
}

//-------------------------------------------------------------

// UART Initialization
void uartInit( uartMap_t uart, uint32_t baudRate )
{
   // Initialize UART
   Chip_UART_Init( uartModule[uart].addr );
   // Set Baud rate
   Chip_UART_SetBaud( uartModule[uart].addr, baudRate );
   
   //Chip_UART_ConfigData(LPC_UART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
   
   // Restart FIFOS using FCR (FIFO Control Register).
   // Set Enable, Reset content, set trigger level
   Chip_UART_SetupFIFOS( uartModule[uart].addr,
                         UART_FCR_FIFO_EN |
                         UART_FCR_TX_RS   |
                         UART_FCR_RX_RS   |
                         UART_FCR_TRG_LEV0 );
	/*Chip_UART_SetupFIFOS(lpcUarts[uart].uartAddr, 
                          (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));*/
   
   // Dummy read
   Chip_UART_ReadByte( uartModule[uart].addr );
   
   // Enable UART Transmission
   Chip_UART_TXEnable( uartModule[uart].addr );
   
   // Configure UARTn_TXD uartPin
   Chip_IOCON_PinMux(
		   LPC_IOCON,
		   uartModule[uart].tx.port,
		   uartModule[uart].tx.pin,
		   IOCON_MODE_PULLDOWN,
		   uartModule[uart].tx.func );

   // Configure UARTn_RXD uartPin
   Chip_IOCON_PinMux(
		   LPC_IOCON,
		   uartModule[uart].rx.port,
		   uartModule[uart].rx.pin,
		   IOCON_MODE_INACT,
		   uartModule[uart].rx.func );
}

// UART Initialization 2
void uartInit2( uartMap_t uart, uint32_t baudRate, uint8_t dataBits, uint8_t parity, uint8_t stopBits )
{
   // Initialize UART
   Chip_UART_Init( uartModule[uart].addr );
   // Set dataBits, stopBits and parity
   
   uint32_t config = 0;

   config = (dataBits-5) | ((stopBits-1) << 2);
   if( parity != UART_PARITY_NONE )
   {
      config |= UART_LCR_PARITY_EN;

      if( parity == UART_PARITY_EVEN )
      {
         config |= UART_LCR_PARITY_EVEN;
      }

      if( parity == UART_PARITY_ODD )
      {
         config |= UART_LCR_PARITY_ODD;
      }
   }
   else
   {
      config |= UART_LCR_PARITY_DIS;
   }   
   // example: config = UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_EN | UART_LCR_PARITY_EVEN;
   Chip_UART_ConfigData( uartModule[uart].addr, config );

   // Set Baud rate
   Chip_UART_SetBaud( uartModule[uart].addr, baudRate );
   // Restart FIFOS using FCR (FIFO Control Register).
   // Set Enable, Reset content, set trigger level
   Chip_UART_SetupFIFOS( uartModule[uart].addr,
                         UART_FCR_FIFO_EN |
                         UART_FCR_TX_RS   |
                         UART_FCR_RX_RS   |
                         UART_FCR_TRG_LEV0 );
   // Dummy read
   Chip_UART_ReadByte( uartModule[uart].addr );
   // Enable UART Transmission
   Chip_UART_TXEnable( uartModule[uart].addr );

   // Configure UARTn_TXD uartPin
   Chip_IOCON_PinMux(
		   LPC_IOCON,
		   uartModule[uart].tx.port,
		   uartModule[uart].tx.pin,
		   IOCON_MODE_PULLDOWN,
		   uartModule[uart].tx.func );

   // Configure UARTn_RXD uartPin
   Chip_IOCON_PinMux(
		   LPC_IOCON,
		   uartModule[uart].rx.port,
		   uartModule[uart].rx.pin,
		   IOCON_MODE_INACT,
		   uartModule[uart].rx.func );
}

// Read 1 byte from RX FIFO, check first if exist aviable data
bool_t uartReadByte( uartMap_t uart, uint8_t* receivedByte )
{
   bool_t retVal = TRUE;

   if ( uartRxReady(uart) )
   {
      *receivedByte = uartRxRead(uart);
   }
   else
   {
      retVal = FALSE;
   }

   return retVal;
}

// Blocking Write 1 byte to TX FIFO
void uartWriteByte( uartMap_t uart, const uint8_t value )
{
   // Wait for space in FIFO (blocking)
   while( uartTxReady( uart ) == FALSE );
   // Send byte
   uartTxWrite( uart, value );
}

// Blocking Send a string
void uartWriteString( uartMap_t uart, const char* str )
{
   while( *str != 0 )
   {
      uartWriteByte( uart, (uint8_t)*str );
      str++;
   }
}

// Blocking, Send a Byte Array
void uartWriteByteArray( uartMap_t uart, const uint8_t* byteArray, uint32_t byteArrayLen )
{
   uint32_t i = 0;

   for( i=0; i<byteArrayLen; i++ )
   {
      uartWriteByte( uart, byteArray[i] );
   }
}

/*==================[ISR external functions definition]======================*/

#ifdef SAPI_USE_INTERRUPTS

__attribute__ ((section(".after_vectors")))

void UART0_IRQHandler(void)
{
	uartProcessIRQ( UART0 );
}

void UART1_IRQHandler(void)
{
   uartProcessIRQ( UART1 );
}

void UART2_IRQHandler(void)
{
	uartProcessIRQ( UART2 );
}

void UART3_IRQHandler(void)
{
	uartProcessIRQ( UART3 );
}
#endif /* SAPI_USE_INTERRUPTS */

/*==================[end of file]============================================*/
