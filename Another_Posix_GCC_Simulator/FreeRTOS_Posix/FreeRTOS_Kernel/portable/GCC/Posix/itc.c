/*
 * itc.c
 *
 *  Created on: 2010-12-28
 *      Author: Hongzhi Song
 */

#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <limits.h>
#include "FreeRTOSConfig.h"
#include "util.h"
#include "itc.h"

static char *ITCOpNames[ ITC_OPERATION_COUNT ] = {
	"PORT_INITIALISE_STACK",
	"PORT_YIELD_FROM_ISR",
	"PORT_YIELD",
	"PORT_DISABLE_INTERRUPTS",
	"PORT_ENABLE_INTERRUPTS",
	"PORT_SET_INTERRUPT_MASK",
	"PORT_CLEAR_INTERRUPT_MASK",
	"PORT_ENTER_CRITICAL",
	"PORT_EXIT_CRITICAL",
	"PORT_FORCIBLY_END_THREAD",
	"PORT_ADD_TASK_HANDLE",
	"PORT_END_SCHEDULER",
	"PORT_FIND_TICKS_PER_SECOND",
	"PORT_GET_TIMER_VALUE",
	"PORT_ADD_BOTTOM_HALF",

	"MANAGEMENT_ITC_MAY_RELEASE_SIGNAL_HANDLER_AND_MAY_YIELD",
#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )
	"MANAGEMENT_ITC_WAKEUP",
#endif
	"MANAGEMENT_QUIT",
	"MANAGEMENT_ACK",
	"MANAGEMENT_TASK_SWITCH_NOTICE"
};


#ifndef	NDEBUG
static char *ITCSrcNames[] = {
	"TASK_THREAD",
	"SIGNAL_HANDLER1",
	"SIGNAL_HANDLER2",
	"SIGNAL_HANDLER3",
	"DEFERRED_WORK_QUEUE1",
	"DEFERRED_WORK_QUEUE2",
	"OTHER"
};
#endif	/* NDEBUG */


int ITCInit( ITC_t *itc )
{
	int pipefd[2];

	/* Create pipe */
	if ( pipe( pipefd ) < 0 )
	{
		return -1;
	}

	/* Check pipe write atomicity */
	/* For PIPE_BUF macro, see <limits.h> */
	if ( PIPE_BUF < sizeof( ITCMsg_t ) )
	{
		(void)close( pipefd[0] );
		(void)close( pipefd[1] );

		return -1;
	}

	/* Set own pipefd */
	itc->own.pipefd[0] = pipefd[0];
	itc->own.pipefd[1] = pipefd[1];

	/* Clear destination */
	itc->dst.pipefd[0] = -1;
	itc->dst.pipefd[1] = -1;

	return 0;
}


void ITCDestroy( const ITC_t *itc )
{
	(void)close( itc->own.pipefd[0] );
	(void)close( itc->own.pipefd[1] );
}


int ITCBind( ITC_t *itc, const ITC_t *dst )
{
	itc->dst.pipefd[0] = dst->own.pipefd[0];
	itc->dst.pipefd[1] = dst->own.pipefd[1];
	return 0;
}


int ITCRecv( const ITC_t *itc, ITCMsg_t *msg )
{
	return read_all( itc->own.rfd, msg, sizeof( ITCMsg_t ) );
}


int ITCSendTo( const ITC_t *dst, const ITCMsg_t *msg )
{
	return write_all( dst->own.wfd, msg, sizeof( ITCMsg_t ) );
}


int ITCSend( const ITC_t *src, const ITCMsg_t *msg )
{
	int wfd = src->dst.wfd;

	/* Check if destination is available */
	if ( wfd == -1 )
	{
		errno = ENOENT;
		return -1;
	}
	return write_all( wfd, msg, sizeof( ITCMsg_t ) );
}


void vITCMsgPrint( const ITCMsg_t *msg )
{

	printf( "Idx = %2ld, ", msg->idx );
#ifndef	NDEBUG
	printf( "atm = %d, etm = %d, ", msg->__debug.add_time, msg->__debug.exe_time );
	{
		char *itcsrc_name;
		int itcsrc = msg->__debug.src;

		if ( itcsrc >= ITC_SRC_TASK_THREAD && itcsrc <= ITC_SRC_OTHER )
			itcsrc_name = ITCSrcNames[ itcsrc ];
		else
			itcsrc_name = "Unknown";

		printf( "src = %19s, ", itcsrc_name );
	}
#endif

	printf(	"op = %26s, ", ITCOpNames[ msg->op ] );

	switch( msg->op )
	{
		case PORT_ADD_BOTTOM_HALF:
			printf( "bh = %p, param = %p\n", msg->bh.bh, msg->bh.param );
			break;

#ifndef	NDEBUG
		case MANAGEMENT_TASK_SWITCH_NOTICE:
			printf(	"suspend = %ld, resume = %ld\n",
					msg->__debug.ts.suspend_idx,
					msg->__debug.ts.resume_idx );
			break;
#endif

		default:
			printf( "param = %p\n", msg->param );
			break;
	}

	return;
}

const char* xITCGetOpName( int idx )
{
	if ( idx >= PORT_INITIALISE_STACK && idx <= MANAGEMENT_TASK_SWITCH_NOTICE )
		return ITCOpNames[ idx ];
	else
		return NULL;
}
