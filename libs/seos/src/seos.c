/***************************************************************************//**
 * @file 
 * @author 
 * @date 
 * @details 
 ******************************************************************************/
/*==================[inclusions]=============================================*/
#include "seos.h"
#include "sapi.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
static void seosScheduleTasks(void* ptr);

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
static task_t seosTasksArray[SEOS_MAX_TASKS];

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static void seosScheduleTasks(void* ptr)
{
	uint8_t taskIndex;

	for(taskIndex = 0; taskIndex < SEOS_MAX_TASKS; taskIndex++)
	{
		if(seosTasksArray[taskIndex].ptrTask && (seosTasksArray[taskIndex].status == TASK_STOPPED) && (seosTasksArray[taskIndex].type == TASK_NORMAL) )
		{		
            if(seosTasksArray[taskIndex].delay == 0)
			{
				seosTasksArray[taskIndex].status = TASK_READY;
				if(seosTasksArray[taskIndex].period)
				{
					seosTasksArray[taskIndex].delay = seosTasksArray[taskIndex].period;
				}
			}
			else
			{
				seosTasksArray[taskIndex].delay--;
			}
		}
	}
}

/*==================[external functions definition]==========================*/
void seosScheduleInit(void)
{
	uint8_t taskIndex;
	for (taskIndex = 0; taskIndex < SEOS_MAX_TASKS; taskIndex++)
	{
		seosDeleteTaskIndex(taskIndex);
	}
}


void seosScheduleStart(tick_t tickRateMs)
{
	if( tickInit( tickRateMs ) )
		tickCallbackSet( seosScheduleTasks, NULL );
}


void seosDispatchTask(void)
{
	uint8_t taskIndex;

	for (taskIndex = 0; taskIndex < SEOS_MAX_TASKS; taskIndex++)
	{               
        if (seosTasksArray[taskIndex].ptrTask && (seosTasksArray[taskIndex].status == TASK_READY) && (seosTasksArray[taskIndex].type == TASK_NORMAL) )
		{
            seosTasksArray[taskIndex].status = TASK_RUNNING;
            (*seosTasksArray[taskIndex].ptrTask)((uint8_t*)&taskIndex);            
            if(seosTasksArray[taskIndex].status == TASK_RUNNING)
                seosTasksArray[taskIndex].status = TASK_STOPPED;
            if (seosTasksArray[taskIndex].period == 0)
            {
                seosDeleteTaskIndex(taskIndex);
            }
        }
        else if( seosTasksArray[taskIndex].ptrTask && (seosTasksArray[taskIndex].type == TASK_FAST) )
        {
			(*seosTasksArray[taskIndex].ptrTask)((uint8_t*)&taskIndex);
		}
	}
}


seosError_t seosAddTask(callBackFuncPtr_t ptrTask, tick_t delay, tick_t period, taskType_t type)
{
	seosError_t retVal = SEOS_ERROR;
    uint8_t taskIndex = 0;
	
    for(taskIndex = 0; taskIndex < SEOS_MAX_TASKS; taskIndex++)
	{               
        if(seosTasksArray[taskIndex].ptrTask == ptrTask)
            return retVal;
	}
        
    taskIndex = 0;
	while( (seosTasksArray[taskIndex].ptrTask != NULL) && (taskIndex < SEOS_MAX_TASKS) )
	{
		taskIndex++;
	}
	
	if(taskIndex != SEOS_MAX_TASKS)
	{
		seosTasksArray[taskIndex].ptrTask = ptrTask;
        seosTasksArray[taskIndex].delay = delay;
        seosTasksArray[taskIndex].period = period;
        seosTasksArray[taskIndex].type = type;
        seosTasksArray[taskIndex].status = TASK_STOPPED;
        
        retVal = SEOS_OK;
	}
    	
	return retVal;
}


seosError_t seosSetPeriodTaskIndex(uint8_t taskIndex, tick_t period)
{
	seosError_t retVal = SEOS_ERROR;
	
    if(seosTasksArray[taskIndex].ptrTask != NULL)
	{
        seosTasksArray[taskIndex].period = period;
        seosTasksArray[taskIndex].delay = period;
        retVal = SEOS_OK;
    }
    	
	return retVal;
}


seosError_t seosSetPeriodTaskName(callBackFuncPtr_t ptrTask, tick_t period)
{
	seosError_t retVal = SEOS_ERROR;
    uint8_t taskIndex = 0;
	
    while( (seosTasksArray[taskIndex].ptrTask != ptrTask) && (taskIndex < SEOS_MAX_TASKS) )
	{
		taskIndex++;
	}

	if(taskIndex != SEOS_MAX_TASKS)
	{		
		retVal = seosSetPeriodTaskIndex(taskIndex, period);
	}
    	
	return retVal;
}


seosError_t seosDeleteTaskIndex(uint8_t taskIndex)
{
	seosError_t retVal = SEOS_ERROR;

	if (seosTasksArray[taskIndex].ptrTask != NULL)
	{		
		seosTasksArray[taskIndex].ptrTask = NULL;
        retVal = SEOS_OK;
	}
	
	return retVal;
}


seosError_t seosDeleteTaskName(callBackFuncPtr_t ptrTask)
{
	seosError_t retVal = SEOS_ERROR;
    uint8_t taskIndex = 0;
	
    while ((seosTasksArray[taskIndex].ptrTask != ptrTask) && (taskIndex < SEOS_MAX_TASKS))
	{
		taskIndex++;
	}

	if (taskIndex != SEOS_MAX_TASKS)
	{		
		retVal = seosDeleteTaskIndex(taskIndex);
	}
        
    return retVal;
}


seosError_t seosSuspendTaskIndex(uint8_t taskIndex)
{    
	seosError_t retVal = SEOS_ERROR;

	if (seosTasksArray[taskIndex].ptrTask != NULL)
	{		
		seosTasksArray[taskIndex].status = TASK_SUSPENDED;
		retVal = SEOS_OK;
	}
	
    return retVal;
}


seosError_t seosSuspendTaskName(callBackFuncPtr_t ptrTask)
{    
	seosError_t retVal = SEOS_ERROR;
    uint8_t taskIndex = 0;
	
    while ((seosTasksArray[taskIndex].ptrTask != ptrTask) && (taskIndex < SEOS_MAX_TASKS))
	{
		taskIndex++;
	}

	if (taskIndex != SEOS_MAX_TASKS)
	{	        
		retVal = seosSuspendTaskIndex(taskIndex);
	}
        
    return retVal;
}


seosError_t seosActivateTaskIndex(uint8_t taskIndex)
{
	seosError_t retVal = SEOS_ERROR;

	if (seosTasksArray[taskIndex].ptrTask != NULL)
	{
		seosTasksArray[taskIndex].status = TASK_STOPPED;
		retVal = SEOS_OK;
	}

    return retVal;
}


seosError_t seosActivateTaskName(callBackFuncPtr_t ptrTask)
{
	seosError_t retVal = SEOS_ERROR;
    uint8_t taskIndex = 0;

    while( (seosTasksArray[taskIndex].ptrTask != ptrTask) && (taskIndex < SEOS_MAX_TASKS) )
	{
		taskIndex++;
	}

    if(taskIndex != SEOS_MAX_TASKS)
    {
    	retVal = seosActivateTaskIndex(taskIndex);
    }

    return retVal;
}


seosError_t seosIsRunningTaskName(callBackFuncPtr_t ptrTask)
{
    uint8_t taskIndex;
    seosError_t retVal = SEOS_ERROR;
    
    for(taskIndex = 0; taskIndex < SEOS_MAX_TASKS; taskIndex++)
	{
		if(seosTasksArray[taskIndex].ptrTask == ptrTask)
            if(seosTasksArray[taskIndex].status == TASK_RUNNING)
            	retVal = SEOS_OK;
	}
    
    return retVal;
}


seosError_t seosIsRunningTaskIndex(uint8_t taskIndex)
{
	seosError_t retVal = SEOS_ERROR;

    if (seosTasksArray[taskIndex].ptrTask != NULL)
		if(seosTasksArray[taskIndex].status == TASK_RUNNING)
			retVal = SEOS_OK;

    return retVal;
}


seosError_t seosIsPresentTaskName(callBackFuncPtr_t ptrTask)
{
    uint8_t taskIndex;
    seosError_t retVal = SEOS_ERROR;
    
    for(taskIndex = 0; taskIndex < SEOS_MAX_TASKS; taskIndex++)
	{
		if(seosTasksArray[taskIndex].ptrTask == ptrTask)
			retVal = SEOS_OK;
	}
    
    return retVal;
}


seosError_t seosIsPresentTaskIndex(uint8_t taskIndex)
{
	seosError_t retVal = SEOS_ERROR;
    
    if (seosTasksArray[taskIndex].ptrTask != NULL)
    	retVal = SEOS_OK;
    
    return retVal;
}


void seosWaitTimeAndDispatch(tick_t ms)
{
    delay_t delay;

    delayConfig(&delay, ms);

    while( delayRead(&delay) )
        seosDispatchTask();
}

/*==================[end of file]============================================*/
