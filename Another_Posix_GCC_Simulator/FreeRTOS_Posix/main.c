/*
	FreeRTOS.org V6.0.4 - Copyright (C) 2003-2009 Richard Barry.

	This file is part of the FreeRTOS.org distribution.

	FreeRTOS.org is free software; you can redistribute it and/or modify it
	under the terms of the GNU General Public License (version 2) as published
	by the Free Software Foundation and modified by the FreeRTOS exception.

	FreeRTOS.org is distributed in the hope that it will be useful,	but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
	FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
	more details.

	You should have received a copy of the GNU General Public License along
	with FreeRTOS.org; if not, write to the Free Software Foundation, Inc., 59
	Temple Place, Suite 330, Boston, MA  02111-1307  USA.

	A special exception to the GPL is included to allow you to distribute a
	combined work that includes FreeRTOS.org without being obliged to provide
	the source code for any proprietary components.  See the licensing section
	of http://www.FreeRTOS.org for full details.


	***************************************************************************
	*                                                                         *
	* Get the FreeRTOS eBook!  See http://www.FreeRTOS.org/Documentation      *
	*                                                                         *
	* This is a concise, step by step, 'hands on' guide that describes both   *
	* general multitasking concepts and FreeRTOS specifics. It presents and   *
	* explains numerous examples that are written using the FreeRTOS API.     *
	* Full source code for all the examples is provided in an accompanying    *
	* .zip file.                                                              *
	*                                                                         *
	***************************************************************************

	1 tab == 4 spaces!

	Please ensure to read the configuration and relevant port sections of the
	online documentation.

	http://www.FreeRTOS.org - Documentation, latest information, license and
	contact details.

	http://www.SafeRTOS.com - A version that is certified for use in safety
	critical systems.

	http://www.OpenRTOS.com - Commercial support, development, porting,
	licensing and training services.
*/

/**
 * Creates all the demo application tasks and co-routines, then starts the
 * scheduler.
 *
 * Main. c also creates a task called "Print".  This only executes every
 * five seconds but has the highest priority so is guaranteed to get
 * processor time.  Its main function is to check that all the other tasks
 * are still operational.  Nearly all the tasks in the demo application
 * maintain a unique count that is incremented each time the task successfully
 * completes its function.  Should any error occur within the task the count is
 * permanently halted.  The print task checks the count of each task to ensure
 * it has changed since the last time the print task executed.  If any count is
 * found not to have changed the print task displays an appropriate message.
 * If all the tasks are still incrementing their unique counts the print task
 * displays an "OK" message.
 *
 * The LED flash tasks do not maintain a count as they already provide visual
 * feedback of their status.
 *
 * The print task blocks on the queue into which messages that require
 * displaying are posted.  It will therefore only block for the full 5 seconds
 * if no messages are posted onto the queue.
 *
 * Main. c also provides a demonstration of how the trace visualisation utility
 * can be used, and how the scheduler can be stopped.
 *
 * \page MainC main.c
 * \ingroup DemoFiles
 * <HR>
 */

/* System headers. */
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <errno.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "partest.h"

/* Demo file headers. */
#include "BlockQ.h"
#include "PollQ.h"
#include "death.h"
#include "crflash.h"
#include "flop.h"
#include "print.h"
#include "fileIO.h"
#include "semtest.h"
#include "integer.h"
#include "dynamic.h"
#include "mevents.h"
#include "crhook.h"
#include "blocktim.h"
#include "GenQTest.h"
#include "QPeek.h"
#include "countsem.h"
#include "recmutex.h"

/* Communication with unix socket */
#include "AIO/AIOUnixSocket.h"
#include "Device/Device.h"
#include "Device/net.h"
#include "CmdLine/cmdline.h"
#include "EchoClient/EchoClient.h"


/* Priority definitions for the tasks in the demo application. */
#define mainLED_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainCREATOR_TASK_PRIORITY	( tskIDLE_PRIORITY + 3 )
#define mainPRINT_TASK_PRIORITY		( tskIDLE_PRIORITY + 4 )
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_BLOCK_PRIORITY	( tskIDLE_PRIORITY + 3 )
#define mainCOM_TEST_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainSEMAPHORE_TASK_PRIORITY	( tskIDLE_PRIORITY + 1 )
#define mainGENERIC_QUEUE_PRIORITY	( tskIDLE_PRIORITY )

#define mainDEBUG_LOG_BUFFER_SIZE	( ( unsigned short ) 20480 )

/* The number of flash co-routines to create. */
#define mainNUM_FLASH_CO_ROUTINES	( 8 )

/* The create delete Tasks routines are very expensive so they are
 * disabled unless required for testing. */
#define mainUSE_SUICIDAL_TASKS_DEMO		0

#define mainUSE_MATH_TASKS_DEMO			1
#define mainUSE_POLLEDQUEUE_TASKS_DEMO	1
#define mainUSE_BLOCKTIME_TASKS_DEMO	1
#define mainUSE_SEMAPHORETEST_TASKS_DEMO	1

/* The multi-event task demo resumes low priority tasks explicitly.
 * But the resumed task may not accomplish expected actions,
 * because system tick may schedule a higher priority task again.
 * To make this task run without error, try to lower tick rate,
 * and run this demo with error check task only
 */
#define mainUSE_MULTIEVENT_TASK_DEMO	1

#define mainUSE_QUEUEPEEK_TASK_DEMO		1
#define mainUSE_BLOCKINGQUEUE_TASK_DEMO	1

/* Remove some of the CPU intensive tasks. */
#define mainCPU_INTENSIVE_TASKS		0

/* Error check tasks */
#define mainUSE_ERRORCHECK_TASK		0

/* Task function for the "Print" task as described at the top of the file. */
static void vErrorChecks( void *pvParameters );

/* Function that checks the unique count of all the other tasks as described at
the top of the file. */
static void prvCheckOtherTasksAreStillRunning( void );

/* Constant definition used to turn on/off the pre-emptive scheduler. */
static const short sUsingPreemption = configUSE_PREEMPTION;

/* Used to demonstrate the "task switched in" callback function. */
static portBASE_TYPE prvExampleTaskHook( void * pvParameter );


/* Just used to count the number of times the example task callback function is
called, and the number of times a queue send passes. */
static unsigned long uxCheckTaskHookCallCount = 0;
static unsigned long uxQueueSendPassedCount = 0;
/*-----------------------------------------------------------*/

#define NETDEV_TASK 1
#define OTHER_TASK 1

int main( int argc, char *argv[] )
{
/******************************************
 * For Communication with Unix Socket
 ******************************************/
#if NETDEV_TASK != 0
xAIOUnixSocket xAIOUnixSocket;
xDeviceAggr_t xDevAggr;
struct CmdLine xCmdLine;

	/* Parse command line */
	CmdLineParse( &xCmdLine, argc, argv );
	/* Initialize AIO Unix Socket layer */
	if ( iAIOUnixOpen( &xAIOUnixSocket, xCmdLine.pcUnixSrv, xCmdLine.pcUnixCli ) < 0 )
	{
		printf("Unable to create unix socket layer\n");
		exit( EXIT_FAILURE );
	}
	/* Initialize device layer */
	if ( iDevAggrInit( &xDevAggr, DM_UNIX, &xAIOUnixSocket ) < 0 )
	{
		printf("Unable to initialize device layer\n");
		exit( EXIT_FAILURE );
	}
	/* Initialize virtual net devices */
	if ( iNetInit( &xDevAggr ) < 0 )
	{
		printf("Unable to initialize virtual net devices\n");
		exit( EXIT_FAILURE );
	}
#endif


#if OTHER_TASK != 0
	/* Initialise hardware and utilities. */
	vParTestInitialise();
	vPrintInitialise();

#if mainUSE_MATH_TASKS_DEMO != 0
	/* CREATE ALL THE DEMO APPLICATION TASKS. */
	vStartMathTasks( tskIDLE_PRIORITY );
#endif

#if mainUSE_POLLEDQUEUE_TASKS_DEMO != 0
	vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
#endif

#if mainUSE_BLOCKTIME_TASKS_DEMO != 0
	vCreateBlockTimeTasks();
#endif

#if mainUSE_SEMAPHORETEST_TASKS_DEMO != 0
	vStartSemaphoreTasks( mainSEMAPHORE_TASK_PRIORITY );
#endif

#if mainUSE_MULTIEVENT_TASK_DEMO != 0
	vStartMultiEventTasks();
#endif

#if mainUSE_QUEUEPEEK_TASK_DEMO != 0
	vStartQueuePeekTasks();
#endif

#if mainUSE_BLOCKINGQUEUE_TASK_DEMO != 0
	vStartBlockingQueueTasks( mainQUEUE_BLOCK_PRIORITY );
#endif

#if mainCPU_INTENSIVE_TASKS == 1
	vStartRecursiveMutexTasks();
	vStartDynamicPriorityTasks();
	vStartGenericQueueTasks( mainGENERIC_QUEUE_PRIORITY );
	vStartCountingSemaphoreTasks();
#endif

	/* Create the co-routines that communicate with the tick hook. */
	vStartHookCoRoutines();

#if mainUSE_ERRORCHECK_TASK == 1
	/* Create the "Print" task as described at the top of the file. */
	xTaskCreate( vErrorChecks, "Print", configMINIMAL_STACK_SIZE, NULL, mainPRINT_TASK_PRIORITY, NULL );
#endif

	/* This task has to be created last as it keeps account of the number of tasks
	it expects to see running. */
#if mainUSE_SUICIDAL_TASKS_DEMO == 1
	vCreateSuicidalTasks( mainCREATOR_TASK_PRIORITY );
#endif

#endif

#if NETDEV_TASK != 0
	/* Create Unix socket echo client */
	xTaskCreate( vEchoClient, "EchoCli", configMINIMAL_STACK_SIZE, (void*)0, tskIDLE_PRIORITY + 1, NULL );
#endif
	/* FIXME: This is for debug only */
	void vTestTask( void* dummy );
	// xTaskCreate( vTestTask, "Test", configMINIMAL_STACK_SIZE, (void*)0, tskIDLE_PRIORITY + 1, NULL );
	/* Set the scheduler running.  This function will not return unless a task calls vTaskEndScheduler(). */
	vTaskStartScheduler();

	return 1;
}
/*-----------------------------------------------------------*/

static portBASE_TYPE prvExampleTaskHook( void * pvParameter )
{
	if( pvParameter != ( void * ) 0xabcd )
	{
		/* The parameter did not contain the value we expected, so cause an
		error to be detected by setting the call count back to zero. */
		uxCheckTaskHookCallCount = 0;
	}
	else
	{
		/* Simply increment a number so we know the callback has been executed. */
		uxCheckTaskHookCallCount++;
	}

	return 0;
}
/*-----------------------------------------------------------*/

void vMainQueueSendPassed( void )
{
	/* This is just an example implementation of the "queue send" trace hook. */
	uxQueueSendPassedCount++;
}
/*-----------------------------------------------------------*/

static void vErrorChecks( void *pvParameters )
{
portTickType xExpectedWakeTime;
const portTickType xPrintRate = ( portTickType ) 5000 / portTICK_RATE_MS;
const long lMaxAllowableTimeDifference = ( long ) 0;
portTickType xWakeTime;
long lTimeDifference;
const char *pcReceivedMessage;
const char * const pcTaskBlockedTooLongMsg = "Print task blocked too long!\r\n";

	( void ) pvParameters;

	/* Register our callback function. */
#if ( configUSE_APPLICATION_TASK_TAG == 1 )
	vTaskSetApplicationTaskTag( NULL, prvExampleTaskHook );
#endif

	/* Loop continuously, blocking, then checking all the other tasks are still
	running, before blocking once again.  This task blocks on the queue of
	messages that require displaying so will wake either by its time out expiring,
	or a message becoming available. */
	for( ;; )
	{
#if ( configUSE_APPLICATION_TASK_TAG == 1 )
		/* Call the task hook function. We are not interested in the return as the error check catches it. */
		(void) xTaskCallApplicationTaskHook( NULL, (void *)0xabcd );
#endif

		/* Calculate the time we will unblock if no messages are received
		on the queue.  This is used to check that we have not blocked for too long. */
		xExpectedWakeTime = xTaskGetTickCount();
		xExpectedWakeTime += xPrintRate;

		/* Block waiting for either a time out or a message to be posted that
		required displaying. */
		/* FIXME: Absolute time should be used for time out,
		 * for time interrupt may raise between these two statements.
		 */
		pcReceivedMessage = pcPrintGetNextMessage( xPrintRate );

		/* Was a message received? */
		if( pcReceivedMessage == NULL )
		{
			/* A message was not received so we timed out, did we unblock at the
			expected time? */
			xWakeTime = xTaskGetTickCount();

			/* Calculate the difference between the time we unblocked and the
			time we should have unblocked. */
			if( xWakeTime > xExpectedWakeTime )
			{
				printf("xWakeTime = %d, xExpectedWakeTime = %d, Difference = %d\n",
						xWakeTime, xExpectedWakeTime,
						xWakeTime - xExpectedWakeTime );
				lTimeDifference = ( long ) ( xWakeTime - xExpectedWakeTime );
			}
			else
			{
				lTimeDifference = ( long ) ( xExpectedWakeTime - xWakeTime );
			}

			if( lTimeDifference > lMaxAllowableTimeDifference )
			{
				/* We blocked too long - create a message that will get
				printed out the next time around.  If we are not using
				preemption then we won't expect the timing to be so
				accurate. */
				if( sUsingPreemption == pdTRUE )
				{
					vPrintDisplayMessage( &pcTaskBlockedTooLongMsg );
				}
			}

			/* Check the other tasks are still running, just in case. */
			prvCheckOtherTasksAreStillRunning();
		}
		else
		{
			/* We unblocked due to a message becoming available.  Send the message
			for printing. */
			vDisplayMessage( pcReceivedMessage );
		}
	}
}
/*-----------------------------------------------------------*/

static void prvCheckOtherTasksAreStillRunning( void )
{
static short sErrorHasOccurred = pdFALSE;
static unsigned long uxLastHookCallCount = 0, uxLastQueueSendCount = 0;

#if mainUSE_BLOCKINGQUEUE_TASK_DEMO != 0
	if( xAreBlockingQueuesStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Blocking queues count unchanged!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
#endif

#if mainUSE_MATH_TASKS_DEMO != 0
	if( xAreMathsTaskStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Maths task count unchanged!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
#endif

#if mainUSE_POLLEDQUEUE_TASKS_DEMO != 0
	if( xArePollingQueuesStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Polling queue count unchanged!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
#endif

#if mainUSE_SEMAPHORETEST_TASKS_DEMO != 0
	if( xAreSemaphoreTasksStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Semaphore take count unchanged!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
#endif

#if mainUSE_MULTIEVENT_TASK_DEMO != 0
	if( xAreMultiEventTasksStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Error in multi events tasks!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
#endif

	if( xAreHookCoRoutinesStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Error in tick hook to co-routine communications!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}

#if mainUSE_BLOCKTIME_TASKS_DEMO != 0
	if( xAreBlockTimeTestTasksStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Error in block time test tasks!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
#endif

#if mainUSE_QUEUEPEEK_TASK_DEMO != 0
	if( xAreQueuePeekTasksStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Error in queue peek test task!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
#endif

#if mainCPU_INTENSIVE_TASKS == 1
	if( xAreRecursiveMutexTasksStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Error in recursive mutex tasks!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}

	if( xAreDynamicPriorityTasksStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Dynamic priority count unchanged!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}

	if( xAreGenericQueueTasksStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Error in generic queue test task!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}

	if( xAreCountingSemaphoreTasksStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Error in counting semaphore demo task!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
#endif /* mainCPU_INTENSIVE_TASKS */

#if mainUSE_SUICIDAL_TASKS_DEMO == 1
	if( xIsCreateTaskStillRunning() != pdTRUE )
	{
		vDisplayMessage( "Incorrect number of tasks running!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
#endif /* mainUSE_SUICIDAL_TASKS_DEMO */

	/* The hook function associated with this task is called each time the task
	is switched in.  We therefore expect the number of times the callback
	function has been executed to have increrment since the last time this
	function executed. */
#if ( configUSE_APPLICATION_TASK_TAG == 1 )
	if( uxCheckTaskHookCallCount <= uxLastHookCallCount )
	{
		vDisplayMessage( "Error in task hook call count!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
	else
	{
		uxLastHookCallCount = uxCheckTaskHookCallCount;
	}
#endif /* configUSE_APPLICATION_TASK_TAG */

	/* We would expect some queue sending to occur between calls of this
	function. */
	if( uxQueueSendPassedCount <= uxLastQueueSendCount )
	{
		vDisplayMessage( "Error in queue send hook call count!\r\n" );
		sErrorHasOccurred = pdTRUE;
	}
	else
	{
		uxLastQueueSendCount = uxQueueSendPassedCount;
	}

	if( sErrorHasOccurred == pdFALSE )
	{
		vDisplayMessage( "OK\n" );
	}
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* The co-routines are executed in the idle task using the idle task hook. */
	vCoRoutineSchedule();	/* Comment this out if not using Co-routines. */

	/* Because printf() is not reentrant, so write is used here */
	// write(STDOUT_FILENO, "This is IDLE task\n", 18);

#ifdef __GCC_POSIX__
	struct timespec xTimeToSleep, xTimeSlept;
		/* Makes the process more agreeable when using the Posix simulator. */
		xTimeToSleep.tv_sec = 1;
		xTimeToSleep.tv_nsec = 0;
		nanosleep( &xTimeToSleep, &xTimeSlept );
#endif
}
/*-----------------------------------------------------------*/
/* FIXME: This is for debug only */
void vTestTask( void *dummy )
{
	int i = 0;

	while(1)
	{
		portENTER_CRITICAL();
		i++;
		portEXIT_CRITICAL();
	}
}
/*-----------------------------------------------------------*/
