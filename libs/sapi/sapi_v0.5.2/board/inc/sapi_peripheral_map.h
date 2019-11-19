/* Copyright 2015, Eric Pernia.
 * Copyright 2016, Ian Olivieri.
 * Copyright 2016, Eric Pernia.
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
 */

/* Date: 2015-09-23 */

#ifndef _SAPI_PERIPHERALMAP_H_
#define _SAPI_PERIPHERALMAP_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"

/*==================[c++]====================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[typedef]================================================*/

typedef struct {
	uint8_t func;
	uint8_t port;
	uint8_t pin;
} pinConfig_t;

/* ------- Begin EDU-CIAA-NXP Peripheral Map ------ */

/* Defined for sapi_gpio.h */
typedef enum {
	VCC = -2, GND = -1,

	//J2-5
	GPIO0_9, 	GPIO0_8,	GPIO0_7,	GPIO0_6,	GPIO0_0,	GPIO0_1,	GPIO0_18,	GPIO0_17,
	GPIO0_15,	GPIO0_16,	GPIO0_23,	GPIO0_24,	GPIO0_25,	GPIO0_26,	GPIO1_30,	GPIO1_31,
	GPIO0_2,	GPIO0_3,	GPIO0_21,	LEDR,	GPIO0_27,	GPIO0_28,	GPIO2_13,
	//J2-27

	//J2-38
	GPIO0_4,	GPIO0_5,	GPIO0_10,	GPIO0_11,	GPIO2_0,	GPIO2_1,	GPIO2_2,	GPIO2_3,
	GPIO2_4,	GPIO2_5,	GPIO2_6,	GPIO2_7,	GPIO2_8,	GPIO2_10,	GPIO2_11,	GPIO2_12,
	//J2-53

	GPIO4_28,	GPIO1_29,	GPIO1_26,	GPIO1_23,	GPIO1_20,	LEDB,	GPIO1_28,	GPIO1_25,	GPIO1_22,
	GPIO1_19,	GPIO4_29,	LEDG,	GPIO1_27,	GPIO1_24,	GPIO1_21,	GPIO1_18,
	//PAD19
} gpioMap_t;

/* Defined for sapi_uart.h */
typedef enum {
	UART0  = 0,
	UART1  = 1,
	UART2  = 2,
	UART3  = 3,	//Se utiliza para el DEBUG, se inicializa en Board_Init() y tambien en sapi_stdio.c (ver!)
} uartMap_t;

/* Defined for sapi_can.h */
typedef enum {
	CAN0  = 0,
	CAN1  = 1,
	CAN_MAX = 2,
} canMap_t;

/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_PERIPHERALMAP_H_ */
