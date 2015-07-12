/*
 * trace.c
 *
 *  Created on: 2010-12-31
 *      Author: Hongzhi Song
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "util.h"
#include "fifo.h"
#include "itc.h"
#include "trace.h"

#ifndef	NDEBUG

long long llTaskSwitches = 0;
long long llInterrupts = 0;
static int xOpStatistics[ ITC_OPERATION_COUNT ] = { 0 };

static struct xFIFO xFIFO;
static struct xFIFO xIntOpFIFO;

enum {
	INT_OP_LOCK,
	INT_OP_UNLOCK
};

/* Operation location structure */
struct xOpLoc {
	const char *pcOpName;
	const char *pcFile;
	int lLine;
};

void vTraceInit( int depth )
{
	if( xFIFOInit( &xFIFO, sizeof( ITCMsg_t ), depth ) < 0 )
		BlockOnError( "Cannot initialize trace" );

#define	INT_OP_DEPTH	16

	if( xFIFOInit( &xIntOpFIFO, sizeof( struct xOpLoc ), INT_OP_DEPTH ) < 0 )
		BlockOnError( "Cannot initialize interrupt trace" );
}


void vTraceAdd( const ITCMsg_t *msg )
{
	/* Check that element's operation type */
	if ( msg->op >= PORT_INITIALISE_STACK
		&& msg->op <= MANAGEMENT_TASK_SWITCH_NOTICE )
	{
		xOpStatistics[ msg->op ]++;
	}
	else
		BlockOnError( "Unexpected operation" );

	/* Push one element to fifo,
	 * if fail, means the fifo is full,
	 * pop one and then push.
	 */
	if( xFIFOPush( &xFIFO, msg ) < 0 )
	{
		vFIFOPop( &xFIFO );
		if( xFIFOPush( &xFIFO, msg ) < 0 )
			BlockOnError( "Cannot push to trace" );
	}
}

void vTraceAddTaskSwitch( long lSuspendIndex, long lResumeIndex )
{
	ITCMsg_t xMsg;

	ITC_CONSTRUCT_NOTICE( &xMsg, -2, MANAGEMENT_TASK_SWITCH_NOTICE, NULL );
	ITC_DEBUG_ADD_SOURCE( &xMsg, ITC_SRC_OTHER );
	ITC_DEBUG_ADD_TASK_SWITCH( &xMsg, lSuspendIndex, lResumeIndex );
	ITC_DEBUG_ADD_ADD_TIME( &xMsg, 0 );
	ITC_DEBUG_ADD_EXE_TIME( &xMsg, 0 );

	vTraceAdd( &xMsg );

	return;
}

void vTraceHistory( void )
{
	ITCMsg_t *pxMsg;
	int i = 1;
	int lTotalOpCount = 0;

	printf("Interrupts: %lld\n", llInterrupts);
	printf("Task switches: %lld\n", llTaskSwitches);
	printf("ITC message history (oldest~latest):\n");
	while( (pxMsg = (ITCMsg_t*)xFIFOFront( &xFIFO )) != NULL )
	{
		printf("%2d. ", i++ );
		vITCMsgPrint( pxMsg );
		vFIFOPop( &xFIFO );
	}
	/* Print operations count and ratio */
	printf("\n%28s, %10s, %6s\n", "OP", "Count", "Ratio" );
	for( i = PORT_INITIALISE_STACK; i <= MANAGEMENT_TASK_SWITCH_NOTICE; i++ )
		lTotalOpCount += xOpStatistics[ i ];
	for( i = PORT_INITIALISE_STACK; i <= MANAGEMENT_TASK_SWITCH_NOTICE; i++ )
		printf("%28s, %10d, %02.2lf%%\n",
				xITCGetOpName( i ),
				xOpStatistics[ i ],
				xOpStatistics[ i ]*100 / (double)lTotalOpCount
			  );

	return;
}


#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )

void vTraceIntOpGeneric( const char *op_name, const char *file, int line )
{
	struct xOpLoc xOpLoc;

	xOpLoc.pcOpName = op_name;
	xOpLoc.pcFile = file;
	xOpLoc.lLine = line;

	if( xFIFOPush( &xIntOpFIFO, &xOpLoc ) < 0 )
	{
		vFIFOPop( &xIntOpFIFO );
		if ( xFIFOPush( &xIntOpFIFO, &xOpLoc ) < 0 )
			BlockOnError( "Cannot push interrupt lock/unlock operation to trace" );
	}
}
 
void vTraceIntOpHistory( void )
{
	struct xOpLoc *pxLoc;

	printf( "Interrupt lock/unlock history:\n" );
	printf( "%15s, %50s, %10s\n", "Lock/Unlock", "File", "Line" );
	while( (pxLoc = (struct xOpLoc *)xFIFOFront( &xIntOpFIFO )) != NULL )
	{
		printf( "%15s, %50s, %10d\n", pxLoc->pcOpName, pxLoc->pcFile, pxLoc->lLine );
		vFIFOPop( &xIntOpFIFO );
	}

	return;
}
#endif	/* End if configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 */

#endif	/* NDEBUG */
