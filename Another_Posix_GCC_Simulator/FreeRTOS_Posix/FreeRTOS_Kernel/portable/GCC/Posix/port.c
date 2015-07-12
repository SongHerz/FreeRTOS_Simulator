/*
	Copyright (C) 2009 William Davy - william.davy@wittenstein.co.uk
	2011 Jan. Modified for more POSIX compliant by Hongzhi Song
	Contributed to FreeRTOS.org V6.0.4.

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

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the Posix port.
 *----------------------------------------------------------*/

#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include <sys/times.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <poll.h>
#include <fcntl.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Utilities */
#include "util.h"

/* For deferred work queue */
#include "fifo.h"

/* Inter-Thread Communication */
#include "itc.h"

/* For restricted binary semaphore */
#include "rbsem.h"

/* For trace */
#include "trace.h"

/*-----------------------------------------------------------*/

#ifndef configUSE_EXTERNAL_TICK_SOURCE
#define configUSE_EXTERNAL_TICK_SOURCE 0
#endif
/*-----------------------------------------------------------*/

#define MAX_NUMBER_OF_TASKS 		( _POSIX_THREAD_THREADS_MAX )
/*-----------------------------------------------------------*/

/* Parameters to pass to the newly created pthread. */
typedef struct XPARAMS
{
	pdTASK_CODE pxCode;
	void *pvParams;
} xParams;

/* Task Status */
enum {
	TASK_NONE,		/* The task does not exist */
	TASK_CREATING,	/* The task is being created */
	TASK_STOPPED,	/* The task has stopped */
	TASK_RUNNING,	/* The task is running */
	/* NOTE: THIS shold be the last enum,
	 * since each task has its unique TASK_STOPPING value.
	 */
	TASK_STOPPING		/* The task is being stopped */
};

/* Task boot status */
enum {
	/* Task thread has not been created yet */
	TASK_BOOT_STAGE0,
	/* Task thread has been created, but not suspended yet */
	TASK_BOOT_STAGE1,
	/* Task thread has been created, and has been suspended for the first time.
	 * But actual FreeRTOS task code has not executed yet.
	 */
	TASK_BOOT_STAGE2,
	/* Task thread has been resumed for the first time.
	 * And actual FreeRTOS task code is about to run.
	 */
	TASK_BOOT_DONE
};

/* Each task maintains its own interrupt status in the critical nesting variable. */
typedef struct
{
	portLONG lIndex;
	pthread_t hThread;
	xTaskHandle hTask;
	sig_atomic_t volatile xStatus;
	sig_atomic_t volatile xBootStage;
	unsigned volatile portBASE_TYPE uxCriticalNesting;
	/* For resume wait */
	RBSem_t xResumeSem;
	/* For task switch synchronization */
	sem_t xTaskResumeSyncSem;
	sem_t xTaskSuspendSyncSem;
	/* For Inter-Thread Communication */
	ITC_t xITC;

	/* Task switches */
	portLONG volatile lSwitches;
} xThreadState;
/*-----------------------------------------------------------*/

/* Management ITC status */
enum {
	MANAGEMENT_ITC_STATUS_FREE,
	MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING,
	MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_RELEASING,
	MANAGEMENT_ITC_STATUS_TASK_THREAD_HOLDING,
	MANAGEMENT_ITC_STATUS_TASK_THREAD_RELEASING,
	/* When a task request is processed,
	 * management ITC status is set to this by the management loop
	 */
	MANAGEMENT_ITC_STATUS_TASK_THREAD_POST_RELEASING
};

typedef struct {
	pthread_mutex_t xMutex;
	xThreadState xThread;
	volatile portLONG lITCMsgs;
	volatile int xITCStatus;
	volatile int xITCStatusSettingTaskIndex;
	volatile int xITCRequestSignalHandlerReleaseDepth;
} xManageState_t;

static xManageState_t xManageState;

#define	MANAGE_OPERATION_LOCK()										\
	do {															\
		if ( pthread_mutex_lock( &xManageState.xMutex ) != 0 )		\
			BlockOnError( "Cannot lock management mutex" );			\
	} while(0)

#define	MANAGE_OPERATION_UNLOCK()									\
	do {															\
		if ( pthread_mutex_unlock( &xManageState.xMutex ) != 0 )	\
			BlockOnError( "Cannot unlock management mutex" );		\
	} while(0)



static pthread_key_t xThreadStateKey;
static xThreadState *pxThreads;
static pthread_once_t xSetupOnce = PTHREAD_ONCE_INIT;
/*-----------------------------------------------------------*/

typedef struct {
#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )
	pthread_mutex_t xMutex;
#endif
	/* Signal handlers send their bottom halves here first,
	 * And the bottom halves will be send to management ITC or deferred work queue
	 * in the management loop.
	 */
	ITC_t xLatch;
	sig_atomic_t volatile lLatchedInterrupts;
	sig_atomic_t volatile xEnabled;

#define DEFERRED_WORK_QUEUE_SIZE	8
	struct xFIFO xDeferredWorkQueue;

	/* Interrupt signals array */
#define	MAX_INTERRUPT_SIGNALS_COUNT	8
	int xSignals[ MAX_INTERRUPT_SIGNALS_COUNT ];
	/* How many interrupt signals registered */
	int lSignalsRegistered;
} xInterruptState_t;

xInterruptState_t xInterruptState;

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )

#define	INT_OPERATION_LOCK()													\
	do {																		\
		if ( pthread_mutex_lock( &xInterruptState.xMutex ) != 0 )				\
			BlockOnError( "Cannot lock interrupt operation" );					\
		vTraceIntOpLock( __FILE__, __LINE__ );									\
	} while(0)

#define	INT_OPERATION_UNLOCK(  xNeedWakeUp  )									\
	do {																		\
		vTraceIntOpUnlock( __FILE__, __LINE__ );								\
		/* If a task uses only portENTER_CRITICAL() and portEXIT_CRITICAL() API	\
		 * in an infinite loop, when all interrupt BHs have been pushed to		\
		 * deferred work queue, and the interrupt is disabled at that time, no	\
		 * further interrupt or request will be sent to management ITC, and that\
		 * task will run forever. So, we have to wake up the management loop in	\
		 * this situation.														\
		 */																		\
		if ( ( xNeedWakeUp ) 													\
			&& !xFIFOEmpty( &xInterruptState.xDeferredWorkQueue )				\
			&& prvManageGetMsgs() == 0 )										\
		{																		\
			ITCMsg_t xMsg;														\
			ITC_CONSTRUCT_REQUEST(  &xMsg, lIndexOfCurrentTask,					\
									MANAGEMENT_ITC_WAKEUP, NULL );				\
			if ( prvSendToManageITC( &xMsg ) < 0 )								\
				BlockOnError( "Cannot send wake up request" );					\
		}																		\
		if ( pthread_mutex_unlock( &xInterruptState.xMutex ) != 0 )				\
			BlockOnError( "Cannot unlock interrupt operation" );				\
	} while(0)

#else

#define INT_OPERATION_LOCK()
#define INT_OPERATION_UNLOCK( xNeedWakeUp )

#endif	/* if configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 */

/* Tick interrupt status */
enum {
	TICK_NOT_ISSUED,
	TICK_ISSUED,
	TICK_DONE
};
static volatile sig_atomic_t xTickStatus = TICK_NOT_ISSUED;

static volatile sig_atomic_t xSchedulerEnd = pdTRUE;
#if ( configUSE_PREEMPTION == 1 )
static volatile sig_atomic_t xPendYield = pdFALSE;
#endif
static volatile sig_atomic_t lIndexOfLastAddedTask = 0;
static volatile sig_atomic_t lIndexOfLastSuspendedTask = -1;
static volatile sig_atomic_t lIndexOfCurrentTask = -1;
#if ( configUSE_EXTERNAL_TICK_SOURCE == 1)
static volatile unsigned long ulTotalTicks = 0;
#endif

#ifndef	NDEBUG
static volatile int xManageLoopTime = 0;
#define	PRV_MANAGE_LOOP_TIME_INCREASE()					\
	do	{												\
		xManageLoopTime ++;								\
	} while(0)

#define	PRV_MANAGE_ADD_ITCMSG_ADD_TIME( PMSG )			\
	do {												\
		ITC_DEBUG_ADD_ADD_TIME( PMSG, xManageLoopTime );\
	} while(0)

#define	PRV_MANAGE_ADD_ITC_MSG_EXE_TIME( PMSG )			\
	do {												\
		ITC_DEBUG_ADD_EXE_TIME( PMSG, xManageLoopTime );\
	} while(0)

#else

#define	PRV_MANAGE_LOOP_TIME_INCREASE()
#define	PRV_MANAGE_ADD_ITCMSG_ADD_TIME( PMSG )
#define	PRV_MANAGE_ADD_ITC_MSG_EXE_TIME( PMSG )

#endif	/* NDEBUG */
/*-----------------------------------------------------------*/

/* Management related functions */
static int prvManageITCRecv( ITCMsg_t *pxMsg );
static int prvSendToManageITC( const ITCMsg_t *pxMsg );
static portLONG prvManageGetMsgs( void );

/*-----------------------------------------------------------*/

/*
 * Setup the timer to generate the tick interrupts.
 */
static void prvSetupTimerInterrupt( void );
static void prvStartFirstTask( void );
static void *prvWaitForStart( void * pvParams );
static void prvCleanUpThread( void *xThreadId );
static void prvSuspendSignalHandler(int sig);
static void prvSetupIssues( void );				/* Simulator set up issues */
static void prvDestroyIssues( void );			/* Simulator destroy issues */
static void prvSuspendThread( portLONG lTaskIndex, pthread_t xThreadId );
static void prvResumeThread( portLONG lTaskIndex, pthread_t xThreadId );
static portLONG prvGetThreadIndex( xTaskHandle hTask );
static portLONG prvGetThreadIndexFromThreadHandle( const pthread_t xThreadHandle );
/*
static pthread_t prvGetThreadHandle( xTaskHandle hTask );
*/
static portLONG prvGetFreeThreadState( void );
/*-----------------------------------------------------------*/

/* Initialization stuffs */
static volatile portBASE_TYPE prvInitialized = 0;
static void prvInitOnce( void );		/* Initialization stuff */
static portBASE_TYPE prvInited( void );	/* Check if initialized */

/*
 * Exception handlers.
 */
void vPortYield( void );
void prvSystemTickHandler( int sig );

/*-----------------------------------------------------------*/

/* Structure used for pxPortInitialiseStack() only */
struct xInitialiseStackParam {
	portSTACK_TYPE *pxTopOfStack;
	pdTASK_CODE pxCode;
	void *pvParameters;
};
/*-----------------------------------------------------------*/

/*
 * DO functions
 */
static portBASE_TYPE prvDoRegisterInterruptSignal( int signum );
static portSTACK_TYPE *prvDoPortInitialiseStackWrapper( const struct xInitialiseStackParam *pxParam );
static portSTACK_TYPE *prvDoPortInitialiseStack( const struct xInitialiseStackParam *pxParam );
static void prvDoYieldFromISR( void );
static void prvDoYield( void );
static void prvDoDisableInterrupts( void );
static void prvDoEnableInterrupts( void );
static portBASE_TYPE prvDoSetInterruptMask( void );
static void prvDoClearInterruptMask( portBASE_TYPE xMask );
static void prvDoEnterCritical( void );
static void prvDoExitCritical( void );
static void prvDoForciblyEndThreadWrapper( void *pxTaskToDelete );
static void prvDoForciblyEndThread( void *pxTaskToDelete, portBASE_TYPE *pxNeedAck );
static void prvDoAddTaskHandle( void *pxTaskHandle );
static void prvDoEndScheduler( void );
static void prvDoFindTicksPerSecond( void );
static unsigned long prvDoGetTimerValue( void );
static void prvDoAddInterruptBottomHalf( void (*bh)(void*), void *param );
static void prvAddBottomHalf( void (*bh)(void*), void *param, ITCSrc_t src );
#if 0
static void prvDispatchInterruptBottomHalf( const ITCMsg_t *pxMsg );
#endif
/* DO function for system tick */
static void prvDoSystemTick( void* dummy );


/* Management ITC signal handler releaser */
static void prvDoManagementITCStatusMayReleaseSignalHandlerAndMayYield( void );
/* Management ITC signal handler releaser and may yield */
static void prvManagementITCStatusMayReleaseSignalHandlerAndMayYieldRequest( void );
/* Management loop */
static void prvManagementLoop( void );


/* 
 * Task request function defination macro and some helper macros.
 */
#define _TASK_REQUEST_RETURN_void( pxMsg )						\
	do {														\
		return;													\
	} while(0)

#define _TASK_REQUEST_RETURN_portSTACK_TYPE_POINTER( pxMsg )	\
	do {														\
		return (portSTACK_TYPE *)ITC_GET_RET( pxMsg );			\
	} while(0)

#define _TASK_REQUEST_RETURN_portBASE_TYPE( pxMsg )				\
	do {														\
		return (portBASE_TYPE)ITC_GET_RET( pxMsg );				\
	} while(0)

#define _TASK_REQUEST_RETURN_unsigned_long( pxMsg )				\
	do {														\
		return (unsigned long)ITC_GET_RET( pxMsg );				\
	} while(0)

#define _TASK_REQUEST_DO_FUNC_WITH_PARAM( DO_FUNC_NAME, DO_FUNC_PARAM )		\
			DO_FUNC_NAME( DO_FUNC_PARAM )

#define _TASK_REQUEST_DO_FUNC_WITHOUT_PARAM( DO_FUNC_NAME, DUMMY_PARAM )	\
			DO_FUNC_NAME()

#define _TASK_REQUEST_RETURN_DO_FUNC_void( DO_FUNC )			\
		do {													\
			DO_FUNC;											\
			return;												\
		} while(0)

#define _TASK_REQUEST_RETURN_DO_FUNC_portSTACK_TYPE_POINTER( DO_FUNC )	\
	do {																\
		return DO_FUNC;													\
	} while(0)

#define _TASK_REQUEST_RETURN_DO_FUNC_portBASE_TYPE( DO_FUNC )	\
	do {														\
		return DO_FUNC;											\
	} while(0)

#define _TASK_REQUEST_RETURN_DO_FUNC_unsigned_long( DO_FUNC )	\
	do {														\
		return DO_FUNC;											\
	} while(0)

#define	TASK_REQUEST_FUNCTION( RET_TYPE, RET_NAME, FUNC_NAME, FUNC_PARAM,					\
	   	DO_FUNC_NAME, DO_FUNC_WITH_PARAM, DO_FUNC_PARAM,									\
		ITCMSG_OP, ITCMSG_PARAM )															\
RET_TYPE FUNC_NAME( FUNC_PARAM )															\
{																							\
	xThreadState *pxThreadState;															\
	ITCMsg_t xMsg;																			\
																							\
	/* Setup once only */																	\
	prvInitOnce();																			\
																							\
	/* When xSchedulerEnd is TRUE, the scheduler has not started yet.						\
	 * When xSchedulerEnd is not TURE, task request may still raise from management thread.	\
	 * In these situation, we call prvDo* function directly.								\
	 */																						\
	if ( xSchedulerEnd == pdTRUE															\
		||	( ( pxThreadState = ( xThreadState* )pthread_getspecific( xThreadStateKey ) )	\
			, pxThreadState->lIndex ) == MANAGEMENT_THREAD_INDEX )							\
	{																						\
		_TASK_REQUEST_RETURN_DO_FUNC_##RET_NAME(											\
				_TASK_REQUEST_DO_FUNC_##DO_FUNC_WITH_PARAM( DO_FUNC_NAME, DO_FUNC_PARAM )	\
				);																			\
	}																						\
																							\
	/* Check if the pointer is valid */														\
	if ( pxThreadState == NULL )															\
		BlockOnError( "Cannot get xThreadState pointer" );									\
																							\
	/* Prepare the message */																\
	ITC_CONSTRUCT_REQUEST( &xMsg, pxThreadState->lIndex, ( ITCMSG_OP ), ( ITCMSG_PARAM ) );	\
	ITC_DEBUG_ADD_SOURCE( &xMsg, ITC_SRC_TASK_THREAD );										\
																							\
	while( 1 )																				\
	{																						\
		int xOldManageITCStatus; 															\
		/* Now, check management ITC status */												\
		xOldManageITCStatus = __sync_val_compare_and_swap(									\
				&xManageState.xITCStatus,													\
				MANAGEMENT_ITC_STATUS_FREE,													\
				MANAGEMENT_ITC_STATUS_TASK_THREAD_HOLDING );								\
																							\
		if( xOldManageITCStatus == MANAGEMENT_ITC_STATUS_FREE )								\
		{																					\
			/* Previously, management ITC is in free status.								\
			 * Now, its status is TASK_THREAD_HOLDING.										\
			 * Message can be sent safely.													\
			 */																				\
			xManageState.xITCStatusSettingTaskIndex = pxThreadState->lIndex;				\
																							\
			/* Remember add time */															\
			PRV_MANAGE_ADD_ITCMSG_ADD_TIME( &xMsg );										\
																							\
			/* Send the message */															\
			if ( prvSendToManageITC( &xMsg ) < 0 )											\
				BlockOnError( "Cannot send message to management thread" );					\
																							\
			/* Now, change to TASK_THREAD_RELEASING */										\
			xManageState.xITCStatus = MANAGEMENT_ITC_STATUS_TASK_THREAD_RELEASING;			\
																							\
			break;																			\
		}																					\
		else																				\
		{																					\
			/* Previously, some one owns it.												\
			 * And the owner must be a signal handler.										\
			 * Yield to relinquish CPU for signal handler bottom half.						\
			 */																				\
			switch( xOldManageITCStatus )													\
			{																				\
				case MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING:							\
				case MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_RELEASING:						\
				case MANAGEMENT_ITC_STATUS_TASK_THREAD_RELEASING:							\
				case MANAGEMENT_ITC_STATUS_TASK_THREAD_POST_RELEASING:						\
					/* This is the right path */											\
					sched_yield();															\
					break;																	\
																							\
				case MANAGEMENT_ITC_STATUS_TASK_THREAD_HOLDING:								\
					/* This shoud NOT happen */												\
					printf( "Task %d has already owned the management ITC in %s\n",			\
							xManageState.xITCStatusSettingTaskIndex, "SENDING_MSG_PHASE" );	\
					vTraceHistory();														\
					printf( "Current ITC Msg:\n" );											\
					vITCMsgPrint( &xMsg );													\
					printf( "Current Task Index: %d\n", lIndexOfCurrentTask );				\
					abort();																\
					break;																	\
																							\
				default:																	\
					BlockOnError( "Unknown management ITC owner" );							\
					break;																	\
			}																				\
		}																					\
	}																						\
																							\
	/* Get the acknowledgement */															\
	if ( ITCRecv( &pxThreadState->xITC, &xMsg ) < 0 )										\
		BlockOnError( "Cannot get message from other thread" );								\
	/* Check if the message is from management thread										\
	 * and it is an acknowledgement */														\
	if ( ITC_GET_IDX( &xMsg ) != MANAGEMENT_THREAD_INDEX									\
		|| ITC_GET_OP( &xMsg ) != MANAGEMENT_ACK )											\
		BlockOnError(																		\
			"Message is not from management thread or it is not a acknowledgement" );		\
																							\
	/* Return, if necessary */																\
	_TASK_REQUEST_RETURN_##RET_NAME( &xMsg );												\
}

/*-----------------------------------------------------------*/
/* Management related functions */
static int prvManageITCRecv( ITCMsg_t *pxMsg )
{
	int ret;

	MANAGE_OPERATION_LOCK();
	ret = ITCRecv( &xManageState.xThread.xITC, pxMsg );
	if ( ret > 0 )
	{
		xManageState.lITCMsgs --;
		assert( xManageState.lITCMsgs >= 0 );
	}
	MANAGE_OPERATION_UNLOCK();

	return ret;
}

static int prvSendToManageITC( const ITCMsg_t *pxMsg )
{
	int ret;

	MANAGE_OPERATION_LOCK();
	ret = ITCSendTo( &xManageState.xThread.xITC, pxMsg );
	if ( ret > 0 )
		xManageState.lITCMsgs ++;
	MANAGE_OPERATION_UNLOCK();

	return ret;
}

static portLONG prvManageGetMsgs( void )
{
	int ret;

	MANAGE_OPERATION_LOCK();
	ret = xManageState.lITCMsgs;
	MANAGE_OPERATION_UNLOCK();

	return ret;
}

/*-----------------------------------------------------------*/

portBASE_TYPE xPortRegisterInterruptSignal( int signum )
{
	prvInitOnce();

	return prvDoRegisterInterruptSignal( signum );
}

portBASE_TYPE prvDoRegisterInterruptSignal( int signum )
{
	if( xInterruptState.lSignalsRegistered >= MAX_INTERRUPT_SIGNALS_COUNT )
	{
		BlockOnError( "Maximum interrupt signals has already been registered" );
		return pdFALSE;
	}
	else
	{
		int i;

		/* Check if the signal has been registered */
		for( i = 0; i < xInterruptState.lSignalsRegistered; i++ )
		{
			if( xInterruptState.xSignals[ i ] == signum )
			{
				BlockOnError( "Interrupt signal has already been registered" );
				return pdFALSE;
			}
		}
		/* OK, register the interrupt signal */
		xInterruptState.xSignals[ xInterruptState.lSignalsRegistered ++ ] = signum;
		return pdTRUE;
	}
}
/*-----------------------------------------------------------*/

void prvInitOnce( void )
{
	/* If pthread_once() nested by pthread_once(), program stalls */
	if ( prvInitialized )
		return;

	prvInitialized = 1;

	if ( pthread_once( &xSetupOnce, prvSetupIssues ) != 0 )
		BlockOnError( "Cannot proces setup issues" );
}

portBASE_TYPE prvInited( void )
{
	return prvInitialized;
}
/*-----------------------------------------------------------*/

TASK_REQUEST_FUNCTION( static portSTACK_TYPE *, portSTACK_TYPE_POINTER,
		prvDoPortInitialiseStackWrapper, const struct xInitialiseStackParam *pxParam,
		prvDoPortInitialiseStack, WITH_PARAM, pxParam,
		PORT_INITIALISE_STACK, ( void* )pxParam )

/*
 * See header file for description.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
struct xInitialiseStackParam xParam = {
		.pxTopOfStack = pxTopOfStack,
		.pxCode = pxCode,
		.pvParameters = pvParameters
};

	return prvDoPortInitialiseStackWrapper( &xParam );
}

portSTACK_TYPE *prvDoPortInitialiseStack( const struct xInitialiseStackParam *pxParam )
{
portSTACK_TYPE *pxTopOfStack;
pdTASK_CODE pxCode;
void *pvParameters;
/* Should actually keep this struct on the stack. */
xParams *pxThisThreadParams;
pthread_attr_t xThreadAttributes;

	/* Setup once only */
	prvInitOnce();

	/* Get parameters */
	pxTopOfStack = pxParam->pxTopOfStack;
	pxCode = pxParam->pxCode;
	pvParameters = pxParam->pvParameters;

	/* No need to join the threads. */
	pthread_attr_init( &xThreadAttributes );
	/*
	pthread_attr_setdetachstate( &xThreadAttributes, PTHREAD_CREATE_DETACHED );
	*/

	pxThisThreadParams = malloc( sizeof( xParams ) );

	/* Add the task parameters. */
	pxThisThreadParams->pxCode = pxCode;
	pxThisThreadParams->pvParams = pvParameters;

	lIndexOfLastAddedTask = prvGetFreeThreadState();

	/* Create the new pThread. */
	/* Assign task status */
	pxThreads[ lIndexOfLastAddedTask ].xStatus = TASK_CREATING;
	/* Create resume pipe, thread ITC and new thread */
	if ( 0 != pthread_create(
			&( pxThreads[ lIndexOfLastAddedTask ].hThread ),
			&xThreadAttributes, prvWaitForStart,
			(void *)pxThisThreadParams )
		)
	{
		/* Thread create failed, signal the failure */
		pxTopOfStack = 0;
	}

	/* Destroy attribute */
	pthread_attr_destroy( &xThreadAttributes );

	/* Wait until the task suspends. */
	while ( pxThreads[ lIndexOfLastAddedTask ].xStatus != TASK_STOPPED );

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

void prvStartFirstTask( void )
{
	portLONG lTaskToResumeIndex;

	/* Get task thread index of the first task. */
	lTaskToResumeIndex = prvGetThreadIndex( xTaskGetCurrentTaskHandle() );

	/* Initialise the critical nesting count ready for the first task. */
	pxThreads[ lTaskToResumeIndex ].uxCriticalNesting = 0;

	prvResumeThread( lTaskToResumeIndex, pxThreads[ lTaskToResumeIndex ].hThread );
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portBASE_TYPE xPortStartScheduler( void )
{
sigset_t xSignalToBlock;
sigset_t xSignalsBlocked;
portLONG lIndex;

	/* Setup once only */
	prvInitOnce();

	/* Establish the signals to block before they are needed. */
	sigfillset( &xSignalToBlock );
	/* unmask SIGFPE, SIGILL, SIGSEGV, SIGBUS, SIGINT */
	/* The kernel won't mask SIGKILL and SIGSTOP */
	sigdelset( &xSignalToBlock, SIGFPE );
	sigdelset( &xSignalToBlock, SIGILL );
	sigdelset( &xSignalToBlock, SIGSEGV );
	sigdelset( &xSignalToBlock, SIGBUS );
	sigdelset( &xSignalToBlock, SIGINT );
	/* Unmask some additional signal */
	sigdelset( &xSignalToBlock, SIGABRT );
	sigdelset( &xSignalToBlock, SIG_TICK );
	sigdelset( &xSignalToBlock, SIG_AIO_RX );

	/* Block until the end */
	if ( pthread_sigmask( SIG_SETMASK, &xSignalToBlock, &xSignalsBlocked ) != 0 )
		BlockOnError( "Cannot set management thread signal mask" );

	for ( lIndex = 0; lIndex < MAX_NUMBER_OF_TASKS; lIndex++ )
	{
		pxThreads[ lIndex ].uxCriticalNesting = 0;
	}

	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	prvSetupTimerInterrupt();

	/* Indicate that scheduler has been started */
	xSchedulerEnd = pdFALSE;

	/* Start the first task. Will not return unless all threads are killed. */
	prvStartFirstTask();

	/* Management loop goes here */
	prvManagementLoop();

	printf( "Cleaning Up, Exiting.\n" );
	/* Destroy issues */
	prvDestroyIssues();

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

TASK_REQUEST_FUNCTION( void, void, vPortEndScheduler, void,
		prvDoEndScheduler, WITHOUT_PARAM, DUMMY,
		PORT_END_SCHEDULER, NULL )

void prvDoEndScheduler( void )
{
portBASE_TYPE xNumberOfThreads;
portBASE_TYPE xResult;
	for ( xNumberOfThreads = 0; xNumberOfThreads < MAX_NUMBER_OF_TASKS; xNumberOfThreads++ )
	{
		if ( ( pthread_t )NULL != pxThreads[ xNumberOfThreads ].hThread )
		{
			/* Kill all of the threads, they are in the detached state. */
			xResult = pthread_cancel( pxThreads[ xNumberOfThreads ].hThread );
		}
	}

	/* Signal the scheduler to exit its loop. */
	xSchedulerEnd = pdTRUE;
	/*
	(void)pthread_kill( hMainThread, SIG_RESUME );
	*/
	/* TODO: Send a message tell the management loop to exit */
}
/*-----------------------------------------------------------*/

TASK_REQUEST_FUNCTION( void, void, vPortYieldFromISR, void,
		prvDoYieldFromISR, WITHOUT_PARAM, DUMMY,
		PORT_YIELD_FROM_ISR, NULL)

void prvDoYieldFromISR( void )
{
	/* Calling Yield from a Interrupt/Signal handler often doesn't work.
	 * Therefore, simply indicate that a yield is required soon.
	 */
#if ( configUSE_PREEMPTION == 1 )
	xPendYield = pdTRUE;
#endif
}
/*-----------------------------------------------------------*/

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )

void vPortEnterCritical( void )
{
	prvInitOnce();

	INT_OPERATION_LOCK();

	prvDoEnterCritical();

	INT_OPERATION_UNLOCK( 0 );
}

#else

TASK_REQUEST_FUNCTION( void, void, vPortEnterCritical, void,
		prvDoEnterCritical, WITHOUT_PARAM, DUMMY,
		PORT_ENTER_CRITICAL, NULL )

#endif

void prvDoEnterCritical( void )
{
	prvDoDisableInterrupts();

	if( lIndexOfCurrentTask >= 0 )
	{
		/* lIndexOfCurrentTask >=0 means this is called in a valid
		 * task environment.
		 */
		pxThreads[ lIndexOfCurrentTask ].uxCriticalNesting++;
	}
}
/*-----------------------------------------------------------*/

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )

void vPortExitCritical( void )
{
	prvInitOnce();

	INT_OPERATION_LOCK();

	prvDoExitCritical();

	INT_OPERATION_UNLOCK( 1 );
}

#else

TASK_REQUEST_FUNCTION( void, void, vPortExitCritical, void,
		prvDoExitCritical, WITHOUT_PARAM, DUMMY,
		PORT_EXIT_CRITICAL, NULL )

#endif

void prvDoExitCritical( void )
{
	if( lIndexOfCurrentTask >= 0 )
	{
		/* Check for unmatched exits. */
		if ( pxThreads[ lIndexOfCurrentTask ].uxCriticalNesting > 0 )
		{
			pxThreads[ lIndexOfCurrentTask ].uxCriticalNesting--;
		}

		/* If we have reached 0 then re-enable the interrupts. */
		if( pxThreads[ lIndexOfCurrentTask ].uxCriticalNesting == 0 )
		{
			/* This function will check xPendYield, and switch tasks,
			 * if preemption kernel is enabled.
			 */
			prvDoEnableInterrupts();
		}
	}
	else
	{
		prvDoEnableInterrupts();
	}
}
/*-----------------------------------------------------------*/

TASK_REQUEST_FUNCTION( void, void, vPortYield, void,
		prvDoYield, WITHOUT_PARAM, DUMMY,
		PORT_YIELD, NULL )

void prvDoYield( void )
{
long lTaskToSuspendIndex;
long lTaskToResumeIndex;
pthread_t xTaskToSuspend;
pthread_t xTaskToResume;

	lTaskToSuspendIndex = prvGetThreadIndex( xTaskGetCurrentTaskHandle() );
	xTaskToSuspend = pxThreads[ lTaskToSuspendIndex ].hThread;

	vTaskSwitchContext();

	lTaskToResumeIndex = prvGetThreadIndex( xTaskGetCurrentTaskHandle() );
	xTaskToResume = pxThreads[ lTaskToResumeIndex ].hThread;
	if ( xTaskToSuspend != xTaskToResume )
	{
		extern volatile unsigned portBASE_TYPE uxSchedulerSuspended;
		assert( uxSchedulerSuspended == 0 );

		vTraceAddTaskSwitch( lTaskToSuspendIndex, lTaskToResumeIndex );
		/* Switch tasks. */
		prvSuspendThread( lTaskToSuspendIndex, xTaskToSuspend );
		prvResumeThread( lTaskToResumeIndex, xTaskToResume );
	}
}
/*-----------------------------------------------------------*/

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )

void vPortDisableInterrupts( void )
{
	prvInitOnce();

	INT_OPERATION_LOCK();

	prvDoDisableInterrupts();

	INT_OPERATION_UNLOCK( 0 );
}

#else

TASK_REQUEST_FUNCTION( void, void, vPortDisableInterrupts, void,
		prvDoDisableInterrupts, WITHOUT_PARAM, DUMMY,
		PORT_DISABLE_INTERRUPTS, NULL )

#endif

void prvDoDisableInterrupts( void )
{
	xInterruptState.xEnabled = pdFALSE;
}

/*-----------------------------------------------------------*/

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )

void vPortEnableInterrupts( void )
{
	prvInitOnce();

	INT_OPERATION_LOCK();

	prvDoEnableInterrupts();

	INT_OPERATION_UNLOCK( 1 );
}

#else

TASK_REQUEST_FUNCTION( void, void, vPortEnableInterrupts, void,
		prvDoEnableInterrupts, WITHOUT_PARAM, DUMMY,
		PORT_ENABLE_INTERRUPTS, NULL )

#endif

void prvDoEnableInterrupts( void )
{
	xInterruptState.xEnabled = pdTRUE;
}

/*-----------------------------------------------------------*/

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )

/* xPortSetInterruptMask() and vPortClearInterruptMask()
 * are used in pairs, and runs in BH context.
 * In this simulator, a BH won't be interrupted by other interrupts,
 * and interrupt operation will be locked, when an interrupt raised.
 * So, no lock is required here.
 */
portBASE_TYPE xPortSetInterruptMask( void )
{
	prvInitOnce();

	return prvDoSetInterruptMask();
}

#else

TASK_REQUEST_FUNCTION( portBASE_TYPE, portBASE_TYPE, xPortSetInterruptMask, void,
		prvDoSetInterruptMask, WITHOUT_PARAM, DUMMY,
		PORT_SET_INTERRUPT_MASK, NULL )

#endif

portBASE_TYPE prvDoSetInterruptMask( void )
{
portBASE_TYPE xReturn = xInterruptState.xEnabled;
	xInterruptState.xEnabled = pdFALSE;
	return xReturn;
}

/*-----------------------------------------------------------*/

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )

/* xPortSetInterruptMask() and vPortClearInterruptMask()
 * are used in pairs, and runs in BH context.
 * In this simulator, a BH won't be interrupted by other interrupts,
 * and interrupt operation will be locked, when an interrupt raised.
 * So, no lock is required here.
 */
void vPortClearInterruptMask( portBASE_TYPE xMask )
{
	prvInitOnce();

	prvDoClearInterruptMask( xMask );
}

#else

TASK_REQUEST_FUNCTION( void, void, vPortClearInterruptMask, portBASE_TYPE xMask,
		prvDoClearInterruptMask, WITH_PARAM, xMask,
		PORT_CLEAR_INTERRUPT_MASK, (void*)xMask )

#endif

void prvDoClearInterruptMask( portBASE_TYPE xMask )
{
	xInterruptState.xEnabled = xMask;
}

/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
void prvSetupTimerInterrupt( void )
{
#if ( configUSE_EXTERNAL_TICK_SOURCE == 0 )
struct itimerval itimer, oitimer;
portTickType xMicroSeconds = portTICK_RATE_MICROSECONDS;

	/* Initialise the structure with the current timer information. */
	if ( 0 == getitimer( TIMER_TYPE, &itimer ) )
	{
		/* Set the interval between timer events. */
		itimer.it_interval.tv_sec = 0;
		itimer.it_interval.tv_usec = xMicroSeconds;

		/* Set the current count-down. */
		itimer.it_value.tv_sec = 0;
		itimer.it_value.tv_usec = xMicroSeconds;

		/* Set-up the timer interrupt. */
		if ( 0 != setitimer( TIMER_TYPE, &itimer, &oitimer ) )
		{
			printf( "Set Timer problem.\n" );
		}
	}
	else
	{
		printf( "Get Timer problem.\n" );
	}
#else
  /* Nothing to do here.
   * SIGALRM is delivered to this process by some other process.
   */
#endif
}
/*-----------------------------------------------------------*/

void prvDoSystemTick( void *dummy )
{
	if ( pdTRUE == xInterruptState.xEnabled )
	{
#if ( configUSE_EXTERNAL_TICK_SOURCE == 1 )
		/* System tick increment */
		ulTotalTicks++;
#endif
		/* Tick Increment. */
		vTaskIncrementTick();

#if ( configUSE_PREEMPTION == 1 )
		/* DO NOT SWITCH TASK here.
		 * Just set this to TRUE, and tasks will be switched
		 * at a proper time.
		 */
		xPendYield = pdTRUE;
#endif
	}

	/* Mark tick done */
	assert( xTickStatus == TICK_ISSUED );
	xTickStatus = TICK_DONE;

	return;
}
/*-----------------------------------------------------------*/

void prvSystemTickHandler( int sig )
{
	/* Save original errno */
	int errno_saved = errno;

	assert( xTickStatus == TICK_NOT_ISSUED
			|| xTickStatus == TICK_ISSUED
			|| xTickStatus == TICK_DONE
		  );
	if( xTickStatus == TICK_NOT_ISSUED )
	{
		xTickStatus = TICK_ISSUED;
		/* Add to bottom half */
		prvDoAddInterruptBottomHalf( prvDoSystemTick, NULL );
	}

	/* Restore original errno */
	errno = errno_saved;
}
/*-----------------------------------------------------------*/

TASK_REQUEST_FUNCTION( void, void, vPortForciblyEndThread, void *pxTaskToDelete,
		prvDoForciblyEndThreadWrapper, WITH_PARAM, pxTaskToDelete,
		PORT_FORCIBLY_END_THREAD, pxTaskToDelete )

void prvDoForciblyEndThreadWrapper( void *pxTaskToDelete )
{
	prvDoForciblyEndThread( pxTaskToDelete, NULL );
}

void prvDoForciblyEndThread( void *pxTaskToDelete, portBASE_TYPE *pxNeedACK )
{
xTaskHandle hTaskToDelete = ( xTaskHandle )pxTaskToDelete;
portLONG lTaskToDelete;
portLONG lTaskToResume;
pthread_t xTaskToDelete;
pthread_t xTaskToResume;
portBASE_TYPE xNeedACK = pdTRUE;

	lTaskToDelete = prvGetThreadIndex( hTaskToDelete );
	lTaskToResume = prvGetThreadIndex( xTaskGetCurrentTaskHandle() );
	xTaskToDelete = pxThreads[ lTaskToDelete ].hThread;
	xTaskToResume = pxThreads[ lTaskToResume ].hThread;

	if ( xTaskToResume == xTaskToDelete )
	{
		/* This is a suicidal thread, need to select a different task to run. */
		vTaskSwitchContext();
		lTaskToResume = prvGetThreadIndex( xTaskGetCurrentTaskHandle() );
		xTaskToResume = pxThreads[ lTaskToResume ].hThread;
		/* Ensure after the switch, tasks are diffent */
		assert( xTaskToResume != xTaskToDelete );
		/* Now, delete the suicidal task */
		if ( pthread_cancel( xTaskToDelete ) != 0 )
			BlockOnError( "Cannot cancel a suicidal task thread" );
		/* Wait until the thread canceled */
		if ( pthread_join( xTaskToDelete, NULL ) != 0 )
			BlockOnError( "Pthread join failed" );

		/* And, resume another task */
		prvResumeThread( lTaskToResume, xTaskToResume );

		/* Since suicidal task thread no longer exists, ACK is not required,
		 * for there is no receiver
		 */
		xNeedACK = pdFALSE;
	}
	else
	{
		/* The task to be deleted is suspending.
		 * Just delete it.
		 */
		if ( pthread_cancel( xTaskToDelete ) != 0 )
			BlockOnError( "Cannot cancel a task thread" );
		/* Wait until the thread canceled */
		if ( pthread_join( xTaskToDelete, NULL ) != 0 )
			BlockOnError( "Pthread join failed" );
	}

	if ( pxNeedACK != NULL )
		*pxNeedACK = xNeedACK;

	return;
}
/*-----------------------------------------------------------*/

void *prvWaitForStart( void * pvParams )
{
xParams * pxParams = ( xParams * )pvParams;
pdTASK_CODE pvCode = pxParams->pxCode;
void * pParams = pxParams->pvParams;
	free( pvParams );

	pthread_cleanup_push( prvCleanUpThread, (void *)pthread_self() );

	sigset_t xSignalToBlock;
	pthread_t xSelfHandle = pthread_self();
	portLONG lSelfIndex = prvGetThreadIndexFromThreadHandle( xSelfHandle );

	/* Set signal mask for this task thread */
	sigfillset( &xSignalToBlock );
	/* unmask SIGFPE, SIGILL, SIGSEGV, SIGBUS, SIGINT */
	/* The kernel won't mask SIGKILL and SIGSTOP */
	sigdelset( &xSignalToBlock, SIGFPE );
	sigdelset( &xSignalToBlock, SIGILL );
	sigdelset( &xSignalToBlock, SIGSEGV );
	sigdelset( &xSignalToBlock, SIGBUS );
	sigdelset( &xSignalToBlock, SIGINT );
	/* Unmask some additional signal */
	sigdelset( &xSignalToBlock, SIGABRT );
	sigdelset( &xSignalToBlock, SIG_SUSPEND );

	if ( pthread_sigmask( SIG_SETMASK, &xSignalToBlock, NULL ) != 0 )
		BlockOnError( "Cannot set task thread signal mask" );

	/* Bind xThreadState of this thread to key */
	if ( pthread_setspecific( xThreadStateKey, &pxThreads[ lSelfIndex ]) != 0 )
		BlockOnError( "Cannot bind xThreadState to key" );

	/* Create thread ITC */
	if ( ITCInit( &pxThreads[ lSelfIndex ].xITC ) < 0 )
		BlockOnError( "Cannot initialize ITC\n" );

	/* Create resume semaphore */
	if ( RBSemInit( &pxThreads[ lSelfIndex ].xResumeSem ) < 0 )
		BlockOnError( "Cannot initialize resume pipe\n" );

	/* Initialize task switch synchronization semaphores */
	if ( sem_init( &pxThreads[ lSelfIndex ].xTaskResumeSyncSem, 0, 0 ) < 0 )
		BlockOnError( "Cannot initialize task resuming synchronization semaphore" );

	if ( sem_init( &pxThreads[ lSelfIndex ].xTaskSuspendSyncSem, 0, 0 ) < 0 )
		BlockOnError( "Cannot initialize task suspending synchronization semaphore" );

	/* Set thread cancelability type to asynchronous */
	if ( pthread_setcanceltype( PTHREAD_CANCEL_ASYNCHRONOUS, NULL ) != 0 )
		BlockOnError( "Cannot set task thread cancelability type to asynchronous" );

	pxThreads[ lSelfIndex ].xBootStage = TASK_BOOT_STAGE1;
	prvSuspendThread( lSelfIndex, pthread_self() );
	pxThreads[ lSelfIndex ].xBootStage = TASK_BOOT_DONE;

	pvCode( pParams );

	pthread_cleanup_pop( 1 );
	return (void *)NULL;
}
/*-----------------------------------------------------------*/

void prvCleanUpThread( void *xThreadId )
{
/* Just release resources acquired at thread startup.
 * Scheduler related work has been done by prvDoForciblyEndThread(),
 * when we get here.
 */
portLONG lIndex;

	lIndex = prvGetThreadIndexFromThreadHandle( (pthread_t)xThreadId );

	pxThreads[ lIndex ].hThread = (pthread_t)NULL;
	pxThreads[ lIndex ].hTask = (xTaskHandle)NULL;
	pxThreads[ lIndex ].xStatus = TASK_NONE;
	pxThreads[ lIndex ].xBootStage = TASK_BOOT_STAGE0;
	pxThreads[ lIndex ].uxCriticalNesting = 0;
	/* Destroy resume pipe */
	RBSemDestroy( &pxThreads[ lIndex ].xResumeSem );
	/* Destroy task switch synchronization semaphores */
	(void)sem_destroy( &pxThreads[ lIndex ].xTaskResumeSyncSem );
	(void)sem_destroy( &pxThreads[ lIndex ].xTaskSuspendSyncSem );
	/* Destroy thread ITC */
	ITCDestroy( &pxThreads[ lIndex ].xITC );
	/* Clear switch count */
	pxThreads[ lIndex ].lSwitches = 0;
}
/*-----------------------------------------------------------*/

void prvSuspendSignalHandler(int sig)
{
portLONG lTaskIndex;

	/* Get task index */
	lTaskIndex = lIndexOfLastSuspendedTask;

	/* Set task status to TASK_STOPPED */
	pxThreads[ lTaskIndex ].xStatus = TASK_STOPPED;

	/* Inform suspending process has been completed */
	if ( sem_post( &pxThreads[ lTaskIndex ].xTaskSuspendSyncSem ) < 0 )
		BlockOnError( "Cannot post task suspending synchronization semaphore" );

	/* Wait on the resume signal. */
	if ( RBSemWait( &pxThreads[ lTaskIndex ].xResumeSem ) < 0 )
		BlockOnError( "Resuming task thread error" );

	/* Now, the suspended task resumed again */
	assert( lTaskIndex == lIndexOfCurrentTask );

	/* Need to set the interrupts based on the task's critical nesting. */
	if ( pxThreads[ lTaskIndex ].uxCriticalNesting == 0 )
	{
		prvDoEnableInterrupts();
	}
	else
	{
		prvDoDisableInterrupts();
	}
	/* xStatus == TASK_RUNNING also means task switch procedure completes */
	/* Task status should be changed atomically or it will cause dead lock */
	pxThreads[ lTaskIndex ].xStatus = TASK_RUNNING;

	/* Inform resuming process has been completed */
	if ( sem_post( &pxThreads[ lTaskIndex ].xTaskResumeSyncSem ) < 0 )
		BlockOnError( "Cannot post task resuming synchronization semaphore" );
}
/*-----------------------------------------------------------*/


/* This function should be called by management thread only */
void prvSuspendThread( portLONG lTaskIndex, pthread_t xThreadId )
{
	/* Task status should be TASK_CREATING or TASK_RUNNING */
	assert( pxThreads[ lTaskIndex ].xStatus == TASK_CREATING
			|| pxThreads[ lTaskIndex ].xStatus == TASK_RUNNING );

	/* Check task boot stage */
	switch( pxThreads[ lTaskIndex ].xBootStage )
	{
		case TASK_BOOT_STAGE0:
			BlockOnError( "Impossible stage" );
			break;

		case TASK_BOOT_STAGE1:
			/* Now, this function is invoked directly from prvWaitForStart()
			 * for the first time for the task. And, we're ready to suspend
			 * the task for the first time, and move to stage2.
			 */
			pxThreads[ lTaskIndex ].xBootStage = TASK_BOOT_STAGE2;
			break;

		case TASK_BOOT_STAGE2:
			/* Rarely, we reach here. For the boot stage will transit to
			 * TASK_BOOT_DONE, when the task has been resumed for the first
			 * time. If we do reach here, wait the boot stage done.
			 */
			while( pxThreads[ lTaskIndex ].xBootStage != TASK_BOOT_DONE )
			{
				sched_yield();
			}
			break;

		case  TASK_BOOT_DONE:
			break;

		default:
			BlockOnError( "Unknown boot stage" );
	}

	/* Assign a unique TASK_STOPPING value */
	pxThreads[ lTaskIndex ].xStatus = TASK_STOPPING + lTaskIndex;
	/* Set last suspended task var */
	lIndexOfLastSuspendedTask = lTaskIndex;
	/* Send suspend signal */
	if( pthread_kill( xThreadId, SIG_SUSPEND ) != 0 )
		BlockOnError( "Cannot send SIG_SUSPEND signal" );
	/* Check the unique TASK_STOPPING value.
	 * If it changed, it means this suspend procedure has been accomplished.
	 */
#if 0
	while ( pxThreads[ lTaskIndex ].xStatus == TASK_STOPPING + lTaskIndex )
	{
		sched_yield();
	}
#endif
	/* Wait for task switch sync semaphore */
	if ( sem_wait( &pxThreads[ lTaskIndex ].xTaskSuspendSyncSem ) < 0 )
		BlockOnError( "Wait task suspending synchronization semaphore failed" );

	/* When xStatus == TASK_RUNNING, the task is resumed for the first time.
	 * Since the task is suspended by itself at the beginning, task is in
	 * TASK_RUNNING state.
	 */
	assert( ( pxThreads[ lTaskIndex ].xStatus == TASK_RUNNING
			&& pxThreads[ lTaskIndex ].lSwitches == 1L
			)
			|| pxThreads[ lTaskIndex ].xStatus == TASK_STOPPED
		  );
 
	return;
}
/*-----------------------------------------------------------*/

void prvResumeThread( portLONG lTaskIndex, pthread_t xThreadId )
{
#if 0
	/* FIXME: This is for debug only */
	{
		static portLONG oldTaskIndex = -1;
		if (lTaskIndex != oldTaskIndex )
		{
			printf("%d\n", lTaskIndex);
			oldTaskIndex = lTaskIndex;
		}
	}
#endif

	TRACE_TASK_SWITCH();

	/* Ensure, the resumed task is actually stopped */
	assert( pxThreads[ lTaskIndex ].xStatus == TASK_STOPPED );

	/* Update current task index */
	lIndexOfCurrentTask = lTaskIndex;

	/* Update task switch count */
	pxThreads[ lTaskIndex ].lSwitches ++;

	/* Resume the task thread */
	if ( RBSemPost( &pxThreads[ lTaskIndex ].xResumeSem ) < 0 )
		BlockOnError( "Unable to resume a thread" );

	/* When a task thread is resumed, it may change interrupt status,
	 * so wait for the thread until it acturally running,
	 * to make interrupt status change a part of the current request.
	 */
#if 0
	while( pxThreads[ lTaskIndex ].xStatus != TASK_RUNNING )
		sched_yield();
#endif
	if ( sem_wait( &pxThreads[ lTaskIndex ].xTaskResumeSyncSem ) < 0 )
		BlockOnError( "Wait task switch synchronization semaphore failed" );
	assert( pxThreads[ lTaskIndex ].xStatus == TASK_RUNNING );
}
/*-----------------------------------------------------------*/

void prvSetupIssues( void )
{
/* The following code would allow for configuring the scheduling of this task as a Real-time task.
 * The process would then need to be run with higher privileges for it to take affect.
int iPolicy;
int iResult;
int iSchedulerPriority;
	iResult = pthread_getschedparam( pthread_self(), &iPolicy, &iSchedulerPriority );
	iResult = pthread_attr_setschedpolicy( &xThreadAttributes, SCHED_FIFO );
	iPolicy = SCHED_FIFO;
	iResult = pthread_setschedparam( pthread_self(), iPolicy, &iSchedulerPriority );		*/

struct sigaction sigsuspendself, sigtick;
portLONG lIndex;
int flags;

	pxThreads = ( xThreadState *)malloc( sizeof( xThreadState ) * MAX_NUMBER_OF_TASKS );
	for ( lIndex = 0; lIndex < MAX_NUMBER_OF_TASKS; lIndex++ )
	{
		pxThreads[ lIndex ].lIndex = lIndex;
		pxThreads[ lIndex ].hThread = ( pthread_t )NULL;
		pxThreads[ lIndex ].hTask = ( xTaskHandle )NULL;
		pxThreads[ lIndex ].xStatus = TASK_NONE;
		pxThreads[ lIndex ].xBootStage = TASK_BOOT_STAGE0;
		pxThreads[ lIndex ].uxCriticalNesting = 0;
		RBSemCleanUp( &pxThreads[ lIndex ].xResumeSem );
		pxThreads[ lIndex ].lSwitches = 0;
	}

	sigsuspendself.sa_flags = 0;
	sigsuspendself.sa_handler = prvSuspendSignalHandler;
	sigfillset( &sigsuspendself.sa_mask );
	/* unmask SIGFPE, SIGILL, SIGSEGV, SIGBUS, SIGINT */
	/* The kernel won't mask SIGKILL and SIGSTOP */
	sigdelset( &sigsuspendself.sa_mask, SIGFPE );
	sigdelset( &sigsuspendself.sa_mask, SIGILL );
	sigdelset( &sigsuspendself.sa_mask, SIGSEGV );
	sigdelset( &sigsuspendself.sa_mask, SIGBUS );
	sigdelset( &sigsuspendself.sa_mask, SIGINT );
	sigdelset( &sigsuspendself.sa_mask, SIGABRT );

	sigtick.sa_flags = 0;
	sigtick.sa_handler = prvSystemTickHandler;
	sigfillset( &sigtick.sa_mask );
	/* unmask SIGFPE, SIGILL, SIGSEGV, SIGBUS, SIGINT */
	/* The kernel won't mask SIGKILL and SIGSTOP */
	sigdelset( &sigtick.sa_mask, SIGFPE );
	sigdelset( &sigtick.sa_mask, SIGILL );
	sigdelset( &sigtick.sa_mask, SIGSEGV );
	sigdelset( &sigtick.sa_mask, SIGBUS );
	sigdelset( &sigtick.sa_mask, SIGINT );
	sigdelset( &sigtick.sa_mask, SIGABRT );

	/* Signal mask is duplicated on fork() */
	if ( 0 != sigaction( SIG_SUSPEND, &sigsuspendself, NULL ) )
	{
		printf( "Problem installing SIG_SUSPEND_SELF\n" );
	}
	if ( 0 != sigaction( SIG_TICK, &sigtick, NULL ) )
	{
		printf( "Problem installing SIG_TICK\n" );
	}

	/* Initialize trace */
	vTraceInit( 24 );

	/* Initialize management mutex */
	if ( pthread_mutex_init( &xManageState.xMutex, NULL ) != 0 )
		BlockOnError( "Cannot initialize management mutex" );

	/* Initialize xThread */
	memset( &xManageState.xThread, 0, sizeof( xManageState.xThread ) );
	xManageState.xThread.lIndex = MANAGEMENT_THREAD_INDEX;

	/* Initialize management ITC */
	if ( ITCInit( &xManageState.xThread.xITC ) < 0 )
		BlockOnError( "Cannot initialize management ITC\n" );
	/* Set management ITC read nonblock */
	if ( ( flags = fcntl( ITCGetReadFd( &xManageState.xThread.xITC ), F_GETFL )) < 0 )
		BlockOnError( "Cannot get management ITC read fd flags" );
	if ( fcntl( ITCGetReadFd( &xManageState.xThread.xITC ), F_SETFL, flags | O_NONBLOCK ) < 0 )
		BlockOnError( "Cannot set management ITC read fd nonblock" );

	/* Initialize management ITC related status */
	xManageState.lITCMsgs = 0L;
	xManageState.xITCStatus = MANAGEMENT_ITC_STATUS_FREE;
	xManageState.xITCStatusSettingTaskIndex = -2;
	xManageState.xITCRequestSignalHandlerReleaseDepth = 0;


	/* Initialize thread-specific data key */
	if( pthread_key_create( &xThreadStateKey, NULL ) != 0 )
		BlockOnError( "Cannot create xThreadStateKey" );

	/* Bind xThreadState of management thread to key */
	if ( pthread_setspecific( xThreadStateKey, &xManageState.xThread ) != 0 )
		BlockOnError( "Cannot bind xThreadState to key" );

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1)
	/* Initialize interrupt mutex */
	if ( pthread_mutex_init( &xInterruptState.xMutex, NULL ) != 0 )
		BlockOnError( "Cannot initialize interrupt mutex" );
#endif
	/* Initialize interrupt ITC */
	if ( ITCInit( &xInterruptState.xLatch ) < 0 )
		BlockOnError( "Cannot initialize interrupt ITC\n" );

	/* Set interrupt ITC read nonblock */
	if ( ( flags = fcntl( ITCGetReadFd( &xInterruptState.xLatch ), F_GETFL )) < 0 )
		BlockOnError( "Cannot get interrupt ITC read fd flags" );
	if ( fcntl( ITCGetReadFd( &xInterruptState.xLatch ), F_SETFL, flags | O_NONBLOCK ) < 0 )
		BlockOnError( "Cannot set interrupt ITC read fd nonblock" );

	/* Set interrupts count to zero */
	xInterruptState.lLatchedInterrupts = 0;
	/* Enable interrupts */
	xInterruptState.xEnabled = pdTRUE;

	/* Initialize deferred work queue */
	if ( xFIFOInit( &xInterruptState.xDeferredWorkQueue,
				sizeof( ITCMsg_t ),
				DEFERRED_WORK_QUEUE_SIZE ) < 0 )
		BlockOnError( "Cannot initialize deferred work queue" );

	/* Clear interrupt signals */
	memset( &xInterruptState.xSignals, 0, sizeof( xInterruptState.xSignals ) );
	/* Initialize signals registered */
	xInterruptState.lSignalsRegistered = 0;

	/* Register SIG_TICK interrupt signal */
	if ( prvDoRegisterInterruptSignal( SIG_TICK ) == pdFALSE )
		BlockOnError( "Cannot register interrupt signal SIG_TICK" );

	printf( "Running as PID: %d\n", getpid() );
}
/*-----------------------------------------------------------*/

void prvDestroyIssues( void )
{
	free( pxThreads );
#ifndef	NDEBUG
	/* No destructor for trace */
#endif
	(void)pthread_key_delete( xThreadStateKey );
	(void)ITCDestroy( &xManageState.xThread.xITC );
#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1)
	(void)pthread_mutex_destroy( &xInterruptState.xMutex );
#endif
	(void)vFIFODestroy( &xInterruptState.xDeferredWorkQueue );
	(void)ITCDestroy( &xInterruptState.xLatch );
}
/*-----------------------------------------------------------*/

portLONG prvGetThreadIndex( xTaskHandle hTask )
{
portLONG lIndex;
	for ( lIndex = 0; lIndex < MAX_NUMBER_OF_TASKS; lIndex++ )
	{
		if ( pxThreads[ lIndex ].hTask == hTask )
			break;
	}

	return lIndex;
}
/*-----------------------------------------------------------*/

portLONG prvGetThreadIndexFromThreadHandle( const pthread_t xThreadHandle )
{
	portLONG lIndex;

	for ( lIndex = 0; lIndex < MAX_NUMBER_OF_TASKS; lIndex++ )
	{
		if ( pxThreads[ lIndex ].hThread == xThreadHandle )
			break;
	}

	return lIndex;
}
/*-----------------------------------------------------------*/

portLONG prvGetFreeThreadState( void )
{
portLONG lIndex;
	for ( lIndex = 0; lIndex < MAX_NUMBER_OF_TASKS; lIndex++ )
	{
		if ( pxThreads[ lIndex ].hThread == ( pthread_t )NULL )
		{
			break;
		}
	}

	if ( MAX_NUMBER_OF_TASKS == lIndex )
	{
		printf( "No more free threads, please increase the maximum.\n" );
		lIndex = 0;
		prvDoEndScheduler();
	}

	return lIndex;
}
/*-----------------------------------------------------------*/

TASK_REQUEST_FUNCTION( void, void, vPortAddTaskHandle, void *pxTaskHandle,
		prvDoAddTaskHandle, WITH_PARAM, pxTaskHandle,
		PORT_ADD_TASK_HANDLE, pxTaskHandle )

void prvDoAddTaskHandle( void *pxTaskHandle )
{
portLONG lIndex;

	pxThreads[ lIndexOfLastAddedTask ].hTask = ( xTaskHandle )pxTaskHandle;
	for ( lIndex = 0; lIndex < MAX_NUMBER_OF_TASKS; lIndex++ )
	{
		if ( pxThreads[ lIndex ].hThread == pxThreads[ lIndexOfLastAddedTask ].hThread )
		{
			if ( pxThreads[ lIndex ].hTask != pxThreads[ lIndexOfLastAddedTask ].hTask )
			{
				pxThreads[ lIndex ].hThread = ( pthread_t )NULL;
				pxThreads[ lIndex ].hTask = NULL;
				pxThreads[ lIndex ].uxCriticalNesting = 0;
			}
		}
	}
}
/*-----------------------------------------------------------*/

TASK_REQUEST_FUNCTION( void, void, vPortFindTicksPerSecond, void,
		prvDoFindTicksPerSecond, WITHOUT_PARAM, DUMMY,
		PORT_FIND_TICKS_PER_SECOND, NULL )

void prvDoFindTicksPerSecond( void )
{
unsigned long ulTicksPerSecond;

	/* Setup once only */
	prvInitOnce();

#if ( configUSE_EXTERNAL_TICK_SOURCE == 0 )
	/* Needs to be reasonably high for accuracy. */
	ulTicksPerSecond = sysconf(_SC_CLK_TCK);
#else
  /* This is just a virtual tick */
  ulTicksPerSecond = configTICK_RATE_HZ;
#endif
	printf( "Timer Resolution for Run TimeStats is %ld ticks per second.\n", ulTicksPerSecond );
}
/*-----------------------------------------------------------*/

TASK_REQUEST_FUNCTION( unsigned long, unsigned_long, ulPortGetTimerValue, void,
		prvDoGetTimerValue, WITHOUT_PARAM, DUMMY,
		PORT_GET_TIMER_VALUE, NULL )

unsigned long prvDoGetTimerValue( void )
{
#if ( configUSE_EXTERNAL_TICK_SOURCE == 0 )
struct tms xTimes;
	unsigned long ulTotalTime = times( &xTimes );
	/* Return the application code times.
	 * The timer only increases when the application code is actually running
	 * which means that the total execution times should add up to 100%.
	 */
	return ( unsigned long ) xTimes.tms_utime;

	/* Should check ulTotalTime for being clock_t max minus 1. */
	(void)ulTotalTime;
#else
  unsigned long ulTotalTime = ulTotalTicks;
  return ulTotalTime;
#endif
}
/*-----------------------------------------------------------*/

/* In normal case,
 * Async-signal-safe: Yes
 */
void vPortAddInterruptBottomHalf( void (*bh)(void*), void *param )
{
	/* Check if simulator has been initialized.
	 * We believe when this function is invoked,
	 * the simulator does have been initialized.
	 * But for sure, we still check here.
	 */
	if ( ! prvInited() )
		BlockOnError( "Simulator has not been initialized yet" );

	prvDoAddInterruptBottomHalf( bh, param );
}


/*
 * Async-signal-safe: Yes
 */
void prvDoAddInterruptBottomHalf( void (*bh)(void*), void *param )
{
	ITCMsg_t xMsg;

	ITC_CONSTRUCT_BOTTOM_HALF_REQUEST( &xMsg, -1, bh, param );
	/* Send the request to interrupt ITC */
	if( ITCSendTo( &xInterruptState.xLatch, &xMsg ) < 0 )
		BlockOnError( "Cannot send bottom half message to interrupt ITC" );

	/* Increase interrupts count */
	xInterruptState.lLatchedInterrupts++;
}


#if 0

/* In normal case,
 * Async-signal-safe: Yes
 */
void prvDispatchInterruptBottomHalf( const ITCMsg_t *pxMsg )
{
	/* Check if the message operation is correct */
	if ( pxMsg->op != PORT_ADD_BOTTOM_HALF )
		BlockOnError( "Invalid message operation type" );

	TRACE_INTERRUPTS();


	if( xInterruptState.xEnabled == pdFALSE )
	{
		/* Interrupt is disabled now.
		 * Push the BH request to deferred work queue.
		 */
		ITCMsg_t xTmp;
		xTmp = *pxMsg;
		ITC_DEBUG_ADD_SOURCE( &xTmp, ITC_SRC_DEFERRED_WORK_QUEUE1 );

		if( xFIFOPush( &xInterruptState.xDeferredWorkQueue, &xTmp ) < 0 )
			BlockOnError( "Cannot push to deferred work queue" );
	}
	else
	{
		/* Now, interrupt is enabled.
		 * Add the bh to management ITC or deferred work queue.
		 */
		int xOldManageITCStatus;

		xOldManageITCStatus = __sync_val_compare_and_swap(
				&xManageState.xITCStatus,
				MANAGEMENT_ITC_STATUS_FREE,
				MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING );

		if( xOldManageITCStatus == MANAGEMENT_ITC_STATUS_FREE )
		{
			xManageState.xITCStatusSettingTaskIndex = -1;
			/* Previously, management ITC is in FREE status.
			 * Now, its status is SIGNAL_HANDLER_HOLDING.
			 * Add operation to bottom half.
			 */
			prvAddBottomHalf(
					ITC_GET_BH( pxMsg ),
					ITC_GET_BH_PARAM( pxMsg ),
					ITC_SRC_SIGNAL_HANDLER1
					);
			/* Send status releasing request */
			prvManagementITCStatusMayReleaseSignalHandlerAndMayYieldRequest();
		}
		else
		{
			/* Management ITC has been already in non-FREE status.
			 * Check who is holding it.
			 */
			switch( xOldManageITCStatus )
			{
				case MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_RELEASING:
					xManageState.xITCStatus = MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING;
				case MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING:
					/* Yeah, another signal handler has been holding it.
					 * Operation can be added safely.
					 */
					prvAddBottomHalf(
							ITC_GET_BH( pxMsg ),
							ITC_GET_BH_PARAM( pxMsg ),
							ITC_SRC_SIGNAL_HANDLER2
							);
					/* Send status releasing request */
					prvManagementITCStatusMayReleaseSignalHandlerAndMayYieldRequest();
					break;

				case MANAGEMENT_ITC_STATUS_TASK_THREAD_HOLDING:
				case MANAGEMENT_ITC_STATUS_TASK_THREAD_RELEASING:
				case MANAGEMENT_ITC_STATUS_TASK_THREAD_POST_RELEASING:
					/* Its status was set by a task thread.
					 * Add the operation to deferred work queue.
					 */
					{
						ITCMsg_t xTmp;
						xTmp = *pxMsg;
						ITC_DEBUG_ADD_SOURCE( &xTmp, ITC_SRC_DEFERRED_WORK_QUEUE2 );

						if( xFIFOPush( &xInterruptState.xDeferredWorkQueue, &xTmp ) < 0 )
							BlockOnError( "Cannot push bottom half to deferred work queue" );
					}
					break;

				default:
					BlockOnError( "Unknown owner" );
					break;
			}
		}
	}
}

#endif

/* Async-signal-safe: Yes */
void prvAddBottomHalf( void (*bh)(void*), void *param, ITCSrc_t src )
{
	ITCMsg_t xMsg;

	ITC_CONSTRUCT_BOTTOM_HALF_REQUEST( &xMsg, -1, bh, param );
	ITC_DEBUG_ADD_SOURCE( &xMsg, src );

	/* Remember add time */
	PRV_MANAGE_ADD_ITCMSG_ADD_TIME( &xMsg );

	if ( prvSendToManageITC( &xMsg ) < 0 )
		BlockOnError( "Cannot write to management ITC" );
}
/*-----------------------------------------------------------*/

void prvManagementAck( const ITCMsg_t *pxMsg, const void *pRet )
{
	ITCMsg_t xAck;
	portLONG lTaskIndex;

	/* Get source task thread index */
	lTaskIndex = ITC_GET_IDX( pxMsg );

	/* Prepare return value */
	ITC_CONSTRUCT_ACK( &xAck, MANAGEMENT_THREAD_INDEX, MANAGEMENT_ACK, pRet );

	if ( ITCSendTo( &pxThreads[ lTaskIndex ].xITC, &xAck ) < 0 )
		BlockOnError( "Cannot write to task thread\n" );

	return;
}
/*-----------------------------------------------------------*/

void prvDoManagementITCStatusMayReleaseSignalHandlerAndMayYield( void )
{
	assert( xManageState.xITCStatus == MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING );
	assert( xInterruptState.xEnabled == pdTRUE );

	if ( -- xManageState.xITCRequestSignalHandlerReleaseDepth == 0 )
	{
#if 0
		/* System tick may have been sent to interrupt ITC,
		 * but not sent to management ITC yet.
		 */
		/* Clear system tick status */
		assert( xTickStatus == TICK_NOT_ISSUED
				|| xTickStatus == TICK_DONE
			  );
#endif

		if ( xTickStatus == TICK_DONE )
			xTickStatus = TICK_NOT_ISSUED;

#if ( configUSE_PREEMPTION == 1 )
		/* Check xPendYield */
		if( xPendYield == pdTRUE )
		{
			prvDoYield();
			xPendYield = pdFALSE;
		}
#endif

		/* At last, set management ITC status to SIGNAL_HANDLER_RELEASING */
		xManageState.xITCStatus = MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_RELEASING;
		xManageState.xITCStatusSettingTaskIndex = -2;

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )
		INT_OPERATION_UNLOCK( 0 );
#endif
	}	/* End if (-- xManageState.xITCRequestSignalHandlerReleaseDepth == 0 ) */

	return;
}

void prvManagementITCStatusMayReleaseSignalHandlerAndMayYieldRequest( void )
{
	ITCMsg_t xMsg;

	ITC_CONSTRUCT_REQUEST( &xMsg, -1, MANAGEMENT_ITC_MAY_RELEASE_SIGNAL_HANDLER_AND_MAY_YIELD, NULL );
	ITC_DEBUG_ADD_SOURCE( &xMsg, ITC_SRC_OTHER );

	/* Record a release depth.
	 * In case, multiple signal bhs and release requests are issued,
	 * but after the first clean request is fulfilled, all the following
	 * signal bhs lost their management ITC owner-ship.
	 * And a task may send an ITC message to management ITC.
	 * If the task is suspended after all bhs are processed,
	 * the message would be handled in some other task's environment,
	 * and it will cause severe problem.
	 */
	xManageState.xITCRequestSignalHandlerReleaseDepth ++;

	/* Remember add time */
	PRV_MANAGE_ADD_ITCMSG_ADD_TIME( &xMsg );

	if ( prvSendToManageITC( &xMsg ) < 0 )
		BlockOnError( "Cannot send signal handler releasing request" );

	return;
}
/*-----------------------------------------------------------*/

void prvManagementLoop( void )
{
	ITCMsg_t xMsg;
	void *pRet;
	sigset_t xSigs;
	int i;
	struct pollfd fds[ 2 ];

	/* Initialize interrupt signals set */
	sigemptyset( &xSigs );
	for( i = 0; i < xInterruptState.lSignalsRegistered; i++ )
		if( sigaddset( &xSigs, xInterruptState.xSignals[ i ] ) < 0 )
			BlockOnError( "Cannot add signal to sigset" );

	/* initialize pollfd array */
#define	INTERRUPT_ITC_POLLFD	0
#define	MANAGEMENT_ITC_POLLFD	1
	fds[ INTERRUPT_ITC_POLLFD ].fd = ITCGetReadFd( &xInterruptState.xLatch );
	fds[ INTERRUPT_ITC_POLLFD ].events = POLLIN;
	fds[ MANAGEMENT_ITC_POLLFD ].fd = ITCGetReadFd( &xManageState.xThread.xITC );
	fds[ MANAGEMENT_ITC_POLLFD ].events = POLLIN;

	while( 1 )
	{
		/* Unmask SIG_AIO_RX and SIG_TICK */
		if ( pthread_sigmask( SIG_UNBLOCK, &xSigs, NULL ) < 0 )
			BlockOnError( "Cannot block signals" );

		/* Poll ITC read fds */
		while( 1 )
		{
			int ret = poll( fds, sizeof(fds)/sizeof(fds[0]), -1 );
			if( ret > 0 )
				break;
			else if( ret < 0 )
			{
				if ( errno == EINTR )
					continue;
				else
					BlockOnError( "Poll ITC read fds failed" );
			}
			else	/* ret == 0 */
				BlockOnError( "Impossible return value" );
		}


		/* Mask SIG_AIO_RX and SIG_TICK */
		if ( pthread_sigmask( SIG_BLOCK, &xSigs, NULL ) < 0 )
			BlockOnError( "Cannot block signals" );



		/* Process management ITC */
#if ( configUSE_FAST_INTERRUPT_RESPONSE == 1 )
#define MANAGEMENT_ITC_PROCESS_BEGIN	do
#define MANAGEMENT_ITC_PROCESS_END		while( 0 );
#else
#define MANAGEMENT_ITC_PROCESS_BEGIN	while( 1 )
#define MANAGEMENT_ITC_PROCESS_END
#endif

		MANAGEMENT_ITC_PROCESS_BEGIN
		{
			/* Management ITC POLLIN flag is not checked,
			 * for that ITC may be filled by BH dispatch process
			 */
			/* If management ITC is not empty */
			if ( prvManageITCRecv( &xMsg ) < 0 )
			{
				if ( errno == EAGAIN || errno == EWOULDBLOCK )
					break;
				else
					BlockOnError( "Cannot read from management ITC" );
			}

			/* Increase management loop time */
			PRV_MANAGE_LOOP_TIME_INCREASE();

			/* Add execution time to ITC message */
			PRV_MANAGE_ADD_ITC_MSG_EXE_TIME( &xMsg );
			/* Add to trace */
			vTraceAdd( &xMsg );

			/* When we go here, the management ITC status must be one of them: */
			assert( xManageState.xITCStatus == MANAGEMENT_ITC_STATUS_FREE
					|| xManageState.xITCStatus == MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING
					|| xManageState.xITCStatus == MANAGEMENT_ITC_STATUS_TASK_THREAD_HOLDING
					|| xManageState.xITCStatus == MANAGEMENT_ITC_STATUS_TASK_THREAD_RELEASING
					/* When task A request portYIELD, after that request is processed, the management ITC is empty.
					 * And task B is resumed and checks and pushes a WAKEUP request
					 * immediately to management ITC, before this loop exits.
					 * The management ITC status will be TASK_THREAD_POST_RELEASING.
					 */
					|| xManageState.xITCStatus == MANAGEMENT_ITC_STATUS_TASK_THREAD_POST_RELEASING
				  );

			/* Check if the current management status is
			 * MANAGEMENT_ITC_STATUS_TASK_THREAD_HOLDING.
			 * If it is, it means:
			 * 1. the current message was sent by a task thread,
			 * 	  and this manage thread is scheduled before that thread
			 * 	  releasing the owner-ship.
			 * 2. the current message is not sent by a task thread,
			 *    but a task thread has sent its message out.
			 *
			 * In case 1, we should sched_yield(), or the request from task thread
			 * may be inserted after a bottom request.
			 * In case 2, same as case 1, we should sched_yield().
			 *
			 * So, wait for the task thread message sending completion.
			 */
			while( xManageState.xITCStatus == MANAGEMENT_ITC_STATUS_TASK_THREAD_HOLDING )
				sched_yield();

			/* Request task index should be the current running task index */
			assert( ITC_GET_IDX( &xMsg ) == -1 || ITC_GET_IDX( &xMsg ) == lIndexOfCurrentTask );

			/* Check operation */
			switch( ITC_GET_OP( &xMsg ) )
			{
				case PORT_INITIALISE_STACK:
					pRet = (void*)prvDoPortInitialiseStack(
							( struct xInitialiseStackParam* )ITC_GET_PARAM( &xMsg ) );
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_YIELD_FROM_ISR:
					prvDoYieldFromISR();
					pRet = NULL;
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_YIELD:
					prvDoYield();
					pRet = NULL;
					prvManagementAck( &xMsg, pRet );
					break;

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 0 )
				case PORT_DISABLE_INTERRUPTS:
					prvDoDisableInterrupts();
					pRet = NULL;
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_ENABLE_INTERRUPTS:
					prvDoEnableInterrupts();
					pRet = NULL;
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_SET_INTERRUPT_MASK:
					pRet = (void*)prvDoSetInterruptMask();
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_CLEAR_INTERRUPT_MASK:
					prvDoClearInterruptMask( ( portBASE_TYPE )ITC_GET_PARAM( &xMsg ) );
					pRet = NULL;
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_ENTER_CRITICAL:
					prvDoEnterCritical();
					pRet = NULL;
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_EXIT_CRITICAL:
					prvDoExitCritical();
					pRet = NULL;
					prvManagementAck( &xMsg, pRet );
					break;
#endif	/* if configUSE_ASYNC_INTERRUPT_OPERATIONS == 0 */

				case PORT_FORCIBLY_END_THREAD:
					{
						portBASE_TYPE xNeedACK;

						prvDoForciblyEndThread( ITC_GET_PARAM( &xMsg ), &xNeedACK );
						pRet = NULL;
						if( xNeedACK == pdTRUE )
							prvManagementAck( &xMsg, pRet );
					}
					break;

				case PORT_ADD_TASK_HANDLE:
					prvDoAddTaskHandle( ITC_GET_PARAM( &xMsg ) );
					pRet = NULL;
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_END_SCHEDULER:
					prvDoEndScheduler();
					pRet = NULL;
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_FIND_TICKS_PER_SECOND:
					prvDoFindTicksPerSecond();
					pRet = NULL;
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_GET_TIMER_VALUE:
					pRet = (void*)prvDoGetTimerValue();
					prvManagementAck( &xMsg, pRet );
					break;

				case PORT_ADD_BOTTOM_HALF:
					ITC_RUN_BH( &xMsg );
					/* DO NOT ACK */
					break;

				case MANAGEMENT_ITC_MAY_RELEASE_SIGNAL_HANDLER_AND_MAY_YIELD:
					prvDoManagementITCStatusMayReleaseSignalHandlerAndMayYield();
					/* DO NOT ACK.
					 * For this request is sent from this management loop.
					 */
					break;

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )
				case MANAGEMENT_ITC_WAKEUP:
					/* This request is for waking up management loop only.
					 * No further process is required.
					 */
					break;
#endif

				default:
					BlockOnError( "Unknown operation" );
					break;
			}	/* switch( ITC_GET_OP( &xMsg ) ) */

			/* If the request is a task request, but not wake up request, set
			 * management ITC status to
			 * MANAGEMENT_ITC_STATUS_TASK_THREAD_POST_RELEASING
			 */
#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )
			if ( xMsg.idx >= 0 && xMsg.op != MANAGEMENT_ITC_WAKEUP )
				xManageState.xITCStatus = MANAGEMENT_ITC_STATUS_TASK_THREAD_POST_RELEASING;
#elif ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 0 )
			if ( xMsg.idx >= 0 )
				xManageState.xITCStatus = MANAGEMENT_ITC_STATUS_TASK_THREAD_POST_RELEASING;
#else
#error "Invalid configUSE_ASYNC_INTERRUPT_OPERATIONS, it shoud be '0' or '1'"
#endif

		}
		MANAGEMENT_ITC_PROCESS_END
		/* End,  Process management ITC */


#if ( configUSE_FAST_INTERRUPT_RESPONSE == 1 )

#define INT_OPERATION_MAY_LOCK()										\
	do {																\
		if ( xManageState.xITCRequestSignalHandlerReleaseDepth == 0 )	\
		{																\
			INT_OPERATION_LOCK();										\
		}																\
	} while( 0 )

#define INT_OPERATION_MAY_UNLOCK()										\
	do {																\
		if ( xManageState.xITCRequestSignalHandlerReleaseDepth == 0 )	\
		{																\
			INT_OPERATION_UNLOCK( 0 );									\
		}																\
	} while( 0 )

#elif ( configUSE_FAST_INTERRUPT_RESPONSE == 0 )

#define INT_OPERATION_MAY_LOCK()											\
	do {																	\
		assert( xManageState.xITCRequestSignalHandlerReleaseDepth == 0 );	\
		INT_OPERATION_LOCK();												\
	} while( 0 )

#define INT_OPERATION_MAY_UNLOCK()											\
	do {																	\
		assert( xManageState.xITCRequestSignalHandlerReleaseDepth == 0 );	\
		INT_OPERATION_UNLOCK( 0 );											\
	} while( 0 )

#else
#error "Invalid configUSE_FAST_INTERRUPT_RESPONSE, it should be '0' or '1'"
#endif	/* if ( configUSE_FAST_INTERRUPT_RESPONSE == 1 */


		/* Acquire interrupt operation lock,
		 * before processing interrupt latch and deferred work queue */
		INT_OPERATION_MAY_LOCK();

		/* Move BHs from interrupt latch to deferred work queue
		 * unconditionally.
		 */
		while(1)
		{
			/* It is not necessary to check interrupt ITC POLLIN flag. */
			ITCMsg_t xMsg;
			
			/* Read one BH request */
			if ( ITCRecv( &xInterruptState.xLatch, &xMsg ) < 0 )
			{
				if ( errno == EAGAIN || errno == EWOULDBLOCK )
					/* No request in the interrupt ITC */
					break;
				else
					BlockOnError( "Cannot fetch BH request from interrupt ITC" );
			}

			/* Move the BH request to deferred work queue */
			if ( xFIFOPush( &xInterruptState.xDeferredWorkQueue, &xMsg ) < 0 )
				BlockOnError( "Cannot move BH request to deferred work queue" );
			xInterruptState.lLatchedInterrupts --;

			TRACE_INTERRUPTS();
		}


		/* Process deferred work queue */
		if( xInterruptState.xEnabled == pdTRUE
			&& !xFIFOEmpty( &xInterruptState.xDeferredWorkQueue ))
		{
			int xOldManageITCStatus;

			/* Deferred works are acturally signal handler works.
			 * So acquire signal handler management ITC owner first.
			 */
			xOldManageITCStatus = __sync_val_compare_and_swap(
					&xManageState.xITCStatus,
					MANAGEMENT_ITC_STATUS_FREE,
					MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING );
			/* Now, xManageState.xITCStatus == MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING,
			 * or MANAGEMENT_ITC_STATUS_TASK_THREAD_RELEASING
			 */

			switch( xOldManageITCStatus )
			{
				case MANAGEMENT_ITC_STATUS_TASK_THREAD_POST_RELEASING:
					/* POST_RELEASING means the task request has been processed.
					 * And, other tasks cannot send their requests yet.
					 */
				case MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_RELEASING:
					/* Set management ITC status to SIGNAL_HANDLER_HOLDING */
					xManageState.xITCStatus = MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING;
				case MANAGEMENT_ITC_STATUS_FREE:
					/* Now, the management ITC status has been set to SIGNAL_HANDLER_HOLDING
					 * by gcc atomic compare and swap
					 */
				case MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING:
					{
						ITCMsg_t *pxMsg;

						while( ( pxMsg = (ITCMsg_t*)xFIFOFront( &xInterruptState.xDeferredWorkQueue )) != NULL )
						{

							/* Remember add time */
							assert( xInterruptState.xEnabled == pdTRUE );
							PRV_MANAGE_ADD_ITCMSG_ADD_TIME( pxMsg );

							if( prvSendToManageITC( pxMsg ) < 0 )
								BlockOnError( "Cannot send deferred work to management ITC" );
							vFIFOPop( &xInterruptState.xDeferredWorkQueue );
						}
						prvManagementITCStatusMayReleaseSignalHandlerAndMayYieldRequest();
					}
					/* Interrupt operation lock will be unlocked in
					 * prvDoManagementITCStatusMayReleaseSignalHandlerAndMayYield()
					 */
					break;

				case MANAGEMENT_ITC_STATUS_TASK_THREAD_RELEASING:
					/* A critical section request may be raised before checking deferred
					 * queue, and not yet processed. And that task may set management
					 * ITC status to MANAGEMENT_ITC_STATUS_TASK_THREAD_RELEASING.
					 * So, we have to let next loop process that request.
					 */
				case MANAGEMENT_ITC_STATUS_TASK_THREAD_HOLDING:
					/* Let it go, we will block on and continue from the poll() */
					assert( xManageState.xITCRequestSignalHandlerReleaseDepth == 0 );
#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )
					/* Release interrupt operation lock */
					INT_OPERATION_UNLOCK( 0 );
#endif
					break;

				default:
					BlockOnError( "Unknown management ITC status" );
					break;
			}	/* End switch( xOldManageITCStatus ) */

		}	/* if( xInterruptState.xEnabled == pdTRUE && !xFIFOEmpty( &xInterrupt.xDeferredWorkQueue ) ) */
		else
		{
			/* xInterruptState.xEnabled != pdTRUE || xFIFOEmpty( &xInterrupt.xDeferredWorkQueue ) */
			INT_OPERATION_MAY_UNLOCK();
		}
		/* End process deferred work queue */


#if 0
		/* Process interrupt ITC */
		for( i = 0;; i++ )
		{
			/* It is not necessary to check interrupt ITC POLLIN flag. */
			ITCMsg_t xMsg;

			/* Read one BH request */
			if ( ITCRecv( &xInterruptState.xLatch, &xMsg ) < 0 )
			{
				if ( errno == EAGAIN || errno == EWOULDBLOCK )
					/* No request in the interrupt ITC */
					break;
				else
					BlockOnError( "Cannot fetch BH request from interrupt ITC" );
			}

#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )
			/* Only do this, when
			 * 1. this is the first loop
			 * 2. no BH resides in management ITC
			 */
			if ( i == 0 && xManageState.xITCRequestSignalHandlerReleaseDepth == 0 )
			{
				INT_OPERATION_LOCK();
				/* Check if interrupt is enabled */
				if ( xInterruptState.xEnabled == pdTRUE )
				{
					/* Interrupt operation will be unlocked when the last interrupt
					 * is processed by the management loop
					 */
				}
				else
				{
					/* Unlock interrupt operation. Or, there is no chance for task threads
					 * to re-enable interrupt.
					 */
					INT_OPERATION_UNLOCK();
				}
			}
#endif
			/* Decrease interrut count */
			xInterruptState.lLatchedInterrupts --;
			/* Dispatch the BH request */
			prvDispatchInterruptBottomHalf( &xMsg );
		}	/* End while(1), Process interrupt ITC */
#endif


		/* Clear management status */
		switch( xManageState.xITCStatus )
		{
			case MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_HOLDING:
				assert( xInterruptState.xEnabled == pdTRUE );
				break;

			case MANAGEMENT_ITC_STATUS_SIGNAL_HANDLER_RELEASING:
			case MANAGEMENT_ITC_STATUS_TASK_THREAD_POST_RELEASING:
				xManageState.xITCStatus = MANAGEMENT_ITC_STATUS_FREE;
				break;

			case MANAGEMENT_ITC_STATUS_FREE:
				break;

			case MANAGEMENT_ITC_STATUS_TASK_THREAD_HOLDING:
			case MANAGEMENT_ITC_STATUS_TASK_THREAD_RELEASING:
				/* Let it go, we will block on and continue from the poll() */
				break;

			default:
				BlockOnError( "Unknown management ITC owner" );
				break;
		}


	}	/* End while(1) */

	return;
}
/*-----------------------------------------------------------*/
