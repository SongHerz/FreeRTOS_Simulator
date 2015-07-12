/*
 * trace.h
 *
 *  Created on: 2010-12-31
 *      Author: Hongzhi Song
 */

#ifndef TRACE_H_
#define TRACE_H_

#include "FreeRTOSConfig.h"
#include "itc.h"

#ifndef	NDEBUG

extern long long llTaskSwitches;
extern long long llInterrupts;

#define	TRACE_TASK_SWITCH()		\
	do {						\
		llTaskSwitches++;		\
	} while(0)

#define	TRACE_INTERRUPTS()		\
	do {						\
		llInterrupts++;			\
	} while(0)

void vTraceInit( int depth );

void vTraceAdd( const ITCMsg_t *msg );

void vTraceAddTaskSwitch( long lSuspendIndex, long lResumeIndex );

void vTraceHistory( void );

#if	( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )

void vTraceIntOpGeneric( const char *op_name, const char *file, int line );
#define	vTraceIntOpLock( file, line )	vTraceIntOpGeneric( "INT_OP_LOCK", (file), (line) )
#define	vTraceIntOpUnlock( file, line )	vTraceIntOpGeneric( "INT_OP_UNLOCK", (file), (line) )
void vTraceIntOpHistory( void );

#endif

#else

#define	TRACE_TASK_SWITCH_INCREASE()
#define	TRACE_INTERRUPTS()

#define	vTraceInit( depth )

#define	vTraceExeAdd( msg )

#define	vTraceAddTaskSwitch( lSuspendIndex, lResumeIndex )

#define	vTraceHistory()

#define	vTraceIntOpLock( file, line )
#define	vTraceIntOpUnlock( file, line )
#define	vTraceIntOpHistory()

#endif	/* NDEBUG */

#endif	/* TRACE_H_ */
