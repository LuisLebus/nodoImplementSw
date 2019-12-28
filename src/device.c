/*=====[Module Name]===========================================================
 * Copyright YYYY Author Compelte Name <author@mail.com>
 * All rights reserved.
 * License: license text or at least name and link 
         (example: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 0.0.0
 * Creation Date: YYYY/MM/DD
 */
 
/*=====[Inclusion of own header]=============================================*/
#include "FreeRTOS.h"
#include "task.h"

#include "device.h"
#include "sapi.h"
#include "impSw.h"
#include "j1939.h"

/*=====[Inclusions of private function dependencies]=========================*/

/*=====[Definition macros of private constants]==============================*/

/*=====[Private function-like macros]========================================*/

/*=====[Definitions of private data types]===================================*/

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

/*=====[Prototypes (declarations) of private functions]======================*/
static void devJ1939RxTask(void *pvParameters);
static void impSwTask(void *pvParameters);

/*=====[Implementations of public functions]=================================*/
void deviceInit(void)
{
	j1939Init(CAN2, true);
	impSwInit(GPIO0_11);

	xTaskCreate(devJ1939RxTask, (signed char *) "devJ1939RxTask", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);
	xTaskCreate(impSwTask, (signed char *) "impSwTask", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();
}
/*=====[Implementations of interrupt functions]==============================*/

/*=====[Implementations of private functions]================================*/
static void devJ1939RxTask(void *pvParameters)
{
	j1939Message_t j1939Message;

	while (1)
	{
		if( j1939Get(&j1939Message, portMAX_DELAY) )
		{
			//Este dispositivo descarta todos los mensajes recibidos
		}
	}
}

static void impSwTask(void *pvParameters)
{
	j1939Message_t j1939Message;

	while (1)
	{
		j1939Message.PDUFormat = DEV_J1939_MSG_IMP_SW;
		j1939Message.PDUSpecific = J1939_GLOBAL_ADDRESS;
		j1939Message.priority = J1939_INFO_PRIORITY;
		j1939Message.dlc = J1939_DATA_LENGTH;
		j1939Message.data[0] = impSwGet();

		//Cada 0,5 [s] se publica el estado del implement switch
		j1939Put(&j1939Message, 0);

		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}
