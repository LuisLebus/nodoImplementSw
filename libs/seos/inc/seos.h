/***************************************************************************//**
 * @file 
 * @author 
 * @date 
 * @details 
 ******************************************************************************/
#ifndef _SEOS_H_
#define _SEOS_H_
/*==================[inclusions]=============================================*/
#include "sapi_datatypes.h"

/*==================[macros]=================================================*/
#define SEOS_MAX_TASKS        	2

/*==================[typedef]================================================*/
//typedef void (*ptrTask_t)(void *);

typedef enum
{
    TASK_NORMAL = 0,
	TASK_FAST
}taskType_t;

typedef enum
{
    TASK_STOPPED = 0,
	TASK_READY,
	TASK_RUNNING,
	TASK_SUSPENDED
}taskStatus_t;

typedef struct
{
	callBackFuncPtr_t ptrTask;
	tick_t delay;
	tick_t period;
	taskStatus_t status;
	taskType_t type;
}task_t;

typedef enum
{
	SEOS_OK = 0,
	SEOS_ERROR
}seosError_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
void seosScheduleInit(void);
void seosScheduleStart(tick_t tickRateMs);
void seosDispatchTask(void);

seosError_t seosAddTask(callBackFuncPtr_t ptrTask, tick_t delay, tick_t period, taskType_t type);

seosError_t seosSetPeriodTaskIndex(uint8_t taskIndex, tick_t period);
seosError_t seosSetPeriodTaskName(callBackFuncPtr_t ptrTask, tick_t period);

seosError_t seosDeleteTaskIndex(uint8_t taskIndex);
seosError_t seosDeleteTaskName(callBackFuncPtr_t ptrTask);

seosError_t seosSuspendTaskIndex(uint8_t taskIndex );
seosError_t seosSuspendTaskName(callBackFuncPtr_t ptrTask);

uint8_t seosActivateTaskIndex(uint8_t taskIndex);
uint8_t seosActivateTaskName(callBackFuncPtr_t ptrTask);

uint8_t seosIsRunningTaskIndex(uint8_t taskIndex);
uint8_t seosIsRunningTaskName(callBackFuncPtr_t ptrTask);

uint8_t seosIsPresentTaskIndex(uint8_t taskIndex);
uint8_t seosIsPresentTaskName(callBackFuncPtr_t ptrTask);

void seosWaitTimeAndDispatch(tick_t ms);

/*==================[end of file]============================================*/
#endif /* _SEOS_H_ */
