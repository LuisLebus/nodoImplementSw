/* Copyright 2015-2016, Eric Pernia.
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
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

/* Date: 2015-09-23 */

/*==================[inclusions]=============================================*/
#include "sapi_gpio.h"

/*==================[macros and definitions]=================================*/

/*==================[typedef]================================================*/
typedef struct {
	pinConfig_t gpio;
} gpioModule_t;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
const gpioModule_t gpioModule[] = {
		//{ PinFUNC, GpioPortN, GpioPinN }

		//J2-5
		{ { FUNC0, 0, 9  }},
		{ { FUNC0, 0, 8  }},
		{ { FUNC0, 0, 7  }},
		{ { FUNC0, 0, 6  }},
		{ { FUNC0, 0, 0  }},
		{ { FUNC0, 0, 1  }},
		{ { FUNC0, 0,18  }},
		{ { FUNC0, 0,17  }},
		{ { FUNC0, 0,15  }},
		{ { FUNC0, 0,16  }},
		{ { FUNC0, 0,23  }},
		{ { FUNC0, 0,24  }},
		{ { FUNC0, 0,25  }},
		{ { FUNC0, 0,26  }},
		{ { FUNC0, 1,30  }},
		{ { FUNC0, 1,31  }},
		{ { FUNC0, 0, 2  }},
		{ { FUNC0, 0, 3  }},
		{ { FUNC0, 0,21  }},
		{ { FUNC0, 0,22  }},
		{ { FUNC0, 0,27  }},
		{ { FUNC0, 0,28  }},
		{ { FUNC0, 2,13  }},
		//J2-27

		//J2-38
		{ { FUNC0, 0, 4  }},
		{{ FUNC0, 0, 5  }},
		{{ FUNC0, 0,10  }},
		{{ FUNC0, 0,11  }},
		{{ FUNC0, 2, 0  }},
		{{ FUNC0, 2, 1  }},
		{{ FUNC0, 2, 2  }},
		{{ FUNC0, 2, 3  }},
		{{ FUNC0, 2, 4  }},
		{{ FUNC0, 2, 5  }},
		{{ FUNC0, 2, 6  }},
		{{ FUNC0, 2, 7  }},
		{{ FUNC0, 2, 8  }},
		{{ FUNC0, 2,10  }},
		{{ FUNC0, 2,11  }},
		{{ FUNC0, 2,12  }},
		//J2-53

		//PAD3
		{{  FUNC0, 4,28  }},
		{{  FUNC0, 1,29  }},
		{{  FUNC0, 1,26  }},
		{ { FUNC0, 1,23  }},
		{ { FUNC0, 1,20  }},
		{ { FUNC0, 3,26  }},
		{ { FUNC0, 1,28  }},
		{ { FUNC0, 1,15  }},
		{ { FUNC0, 1,22  }},
		{ { FUNC0, 1,19  }},
		{ { FUNC0, 4,29  }},
		{ { FUNC0, 3,25  }},
		{ { FUNC0, 1,27  }},
		{ { FUNC0, 1,24  }},
		{ { FUNC0, 1,21  }},
		{ { FUNC0, 1,18 } },
		//PAD19
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

bool_t gpioInit(void)
{
	bool_t ret_val = 1;

	Chip_GPIO_Init(LPC_GPIO);

	return ret_val;
}

bool_t gpioDeinit(void)
{
	bool_t ret_val = 1;

	Chip_GPIO_DeInit(LPC_GPIO);

	return ret_val;
}


bool_t gpioConfigure( gpioMap_t pin, gpioMode_t mode )
{
   if( pin == VCC ){
	  return FALSE;
   }
   if( pin == GND ){
	  return FALSE;
   }

   bool_t ret_val = 1;

   switch(mode)
   {
	   case GPIO_INPUT:
		   Chip_IOCON_PinMuxSet(
				   LPC_IOCON,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   IOCON_MODE_INACT | gpioModule[pin].gpio.func );

		   Chip_GPIO_SetDir(
				   LPC_GPIO,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   GPIO_INPUT );
		  break;
	   case GPIO_INPUT_PULLUP:
		   Chip_IOCON_PinMuxSet(
				   LPC_IOCON,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   IOCON_MODE_PULLUP | gpioModule[pin].gpio.func );

		   Chip_GPIO_SetDir(
				   LPC_GPIO,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   GPIO_INPUT );
		  break;
	   case GPIO_INPUT_PULLDOWN:
		   Chip_IOCON_PinMuxSet(
				   LPC_IOCON,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   IOCON_MODE_PULLDOWN | gpioModule[pin].gpio.func );

		   Chip_GPIO_SetDir(
				   LPC_GPIO,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   GPIO_INPUT );
		  break;
	   case GPIO_INPUT_PULLUP_PULLDOWN:
		   Chip_IOCON_PinMuxSet(
				   LPC_IOCON,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   IOCON_MODE_REPEATER | gpioModule[pin].gpio.func );

		   Chip_GPIO_SetDir(
				   LPC_GPIO,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   GPIO_INPUT );
		  break;
	   case GPIO_OUTPUT:
		   Chip_IOCON_PinMuxSet(
				   LPC_IOCON,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   IOCON_MODE_INACT | gpioModule[pin].gpio.func );

		   Chip_GPIO_WriteDirBit(
				   LPC_GPIO,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   GPIO_OUTPUT);

		   Chip_GPIO_SetPinState(
				   LPC_GPIO,
				   gpioModule[pin].gpio.port,
				   gpioModule[pin].gpio.pin,
				   0 );
		  break;
	   default:
		   ret_val = 0;
		  break;
   }

   return ret_val;
}


bool_t gpioWrite( gpioMap_t pin, bool_t value )
{
   if( pin == VCC ){
	  return FALSE;
   }
   if( pin == GND ){
	  return FALSE;
   }

   bool_t ret_val = 1;

   Chip_GPIO_SetPinState(
		   LPC_GPIO,
		   gpioModule[pin].gpio.port,
		   gpioModule[pin].gpio.pin,
		   value
   );

   return ret_val;
}


bool_t gpioToggle( gpioMap_t pin )
{
   return gpioWrite( pin, !gpioRead(pin) );
}


bool_t gpioRead( gpioMap_t pin )
{
   if( pin == VCC ){
      return TRUE;
   }
   if( pin == GND ){
	  return FALSE;
   }

   bool_t ret_val = OFF;

   ret_val = (bool_t) Chip_GPIO_ReadPortBit(
		   LPC_GPIO,
		   gpioModule[pin].gpio.port,
		   gpioModule[pin].gpio.pin
   );

   return ret_val;
}

/*==================[end of file]============================================*/
