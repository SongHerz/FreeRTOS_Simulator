/*
 * itc.h
 *
 *  Created on: 2010-12-28
 *      Author: Hongzhi Song
 */

/* Inter-Thread Communication */
#ifndef ITC_H_
#define ITC_H_

#include "FreeRTOSConfig.h"

/* Operation enums:
 * PORT_* are used as task requests.
 * MANAGEMENT_* are used internally.
 */
typedef enum {
	/* Requests */
	PORT_INITIALISE_STACK = 0,
	PORT_YIELD_FROM_ISR,
	PORT_YIELD,
	PORT_DISABLE_INTERRUPTS,	/* May be async operation */
	PORT_ENABLE_INTERRUPTS,		/* May be async operation */
	PORT_SET_INTERRUPT_MASK,	/* May be async operation */
	PORT_CLEAR_INTERRUPT_MASK,	/* May be async operation */
	PORT_ENTER_CRITICAL,		/* May be async operation */
	PORT_EXIT_CRITICAL,			/* May be async operation */
	PORT_FORCIBLY_END_THREAD,
	PORT_ADD_TASK_HANDLE,
	PORT_END_SCHEDULER,
	PORT_FIND_TICKS_PER_SECOND,
	PORT_GET_TIMER_VALUE,
	PORT_ADD_BOTTOM_HALF,	/* For serializaing interrupts( signal stuff ) */

	/* Management requests */
	MANAGEMENT_ITC_MAY_RELEASE_SIGNAL_HANDLER_AND_MAY_YIELD,	/* For management ITC status cleaning */
#if ( configUSE_ASYNC_INTERRUPT_OPERATIONS == 1 )
	MANAGEMENT_ITC_WAKEUP,	/* For waking up management loop */
#endif
	MANAGEMENT_QUIT,			/* Management loop quit */

	/* Acknowledgement */
	MANAGEMENT_ACK,

	/* For DEBUG only */
	MANAGEMENT_TASK_SWITCH_NOTICE,	/* For debug task switch message */

	/* Number of operations */
	ITC_OPERATION_COUNT
} ITCOp_t;


#ifndef	NDEBUG
/* This is for debug only.
 * For source of a ITC message.
 */
typedef enum {
	ITC_SRC_TASK_THREAD,
	ITC_SRC_SIGNAL_HANDLER1,
	ITC_SRC_SIGNAL_HANDLER2,
	ITC_SRC_SIGNAL_HANDLER3,
	ITC_SRC_DEFERRED_WORK_QUEUE1,
	ITC_SRC_DEFERRED_WORK_QUEUE2,
	ITC_SRC_OTHER
} ITCSrc_t;
#endif


/* Management thread index */
#define	MANAGEMENT_THREAD_INDEX	0x5A5A5A5A


typedef union {
	int pipefd[2];
	struct {
		int rfd;
		int wfd;
	};
} _pipefd_t;

typedef struct {
	_pipefd_t own;	/* own pipe file descriptors */
	_pipefd_t dst;	/* destination pipe file descriptors */
} ITC_t; 

typedef struct {
	long idx;			/* task thread index */
	ITCOp_t op;			/* Operation */
	union {
		void *param;	/* operation parameter */
		void *ret;		/* operation return value */
		struct {
			void (*bh)(void*);	/* bottom half function */
			void *param;		/* bottom half function parameter */
		} bh;
	};
#ifndef	NDEBUG
	struct {					/* This is for debug only */
		struct {
			long suspend_idx;	/* Index of the task to be suspended */
			long resume_idx;	/* Index of the task to be resumed */
		} ts;					/* Task switch */
		ITCSrc_t src;			/* Source of the message. */
		int add_time;			/* When the message is added to management ITC */
		int exe_time;			/* When the message is executed */
	} __debug;
#endif
} ITCMsg_t;


#define	ITC_CONSTRUCT_REQUEST( PMSG, IDX, OP, PARAM )	\
	do {												\
		ITCMsg_t *pmsg = ( ITCMsg_t *)( PMSG );			\
		pmsg->idx = ( IDX );							\
		pmsg->op = ( OP );								\
		pmsg->param = ( PARAM );						\
	} while(0)

#define	ITC_CONSTRUCT_ACK( PMSG, IDX, OP, RET )			\
	do {												\
		ITCMsg_t *pmsg = ( ITCMsg_t *)( PMSG );			\
		pmsg->idx = ( IDX );							\
		pmsg->op = ( OP );								\
		pmsg->ret = (void*)( RET );						\
	} while(0)

#define	ITC_CONSTRUCT_BOTTOM_HALF_REQUEST( PMSG, IDX, BH, BH_PARAM )	\
	do {																\
		ITCMsg_t *pmsg = ( ITCMsg_t *)( PMSG );							\
		pmsg->idx = ( IDX );											\
		pmsg->op = PORT_ADD_BOTTOM_HALF;								\
		pmsg->bh.bh = ( BH );											\
		pmsg->bh.param = ( BH_PARAM );									\
	} while(0)

/* Literally, this is the same as ITC_CONSTRUCT_REQUEST.
 * But this macro indicates the message is just a notice,
 * no further processing is required.
 */
#define	ITC_CONSTRUCT_NOTICE( PMSG, IDX, OP, PARAM )	\
	ITC_CONSTRUCT_REQUEST( PMSG, IDX, OP, PARAM )

#define	ITC_GET_IDX( PMSG )			( (PMSG)->idx )
#define	ITC_GET_OP( PMSG )			( (PMSG)->op )
#define	ITC_GET_PARAM( PMSG )		( (PMSG)->param )
#define	ITC_SET_RET( PMSG, VAL )	( (PMSG)->ret = (void*)(VAL) )
#define	ITC_GET_RET( PMSG )			( (PMSG)->ret )

#define	ITC_GET_BH( PMSG )			( (PMSG)->bh.bh )
#define	ITC_GET_BH_PARAM( PMSG )	( (PMSG)->bh.param )

#define	ITC_RUN_BH( PMSG )					\
	do {									\
		const ITCMsg_t *pmsg = ( PMSG );	\
		pmsg->bh.bh( pmsg->bh.param );		\
	} while(0)

#ifndef	NDEBUG

#define	ITC_DEBUG_ADD_SOURCE( PMSG, SRC )	( (PMSG)->__debug.src = (SRC) )

#define	ITC_DEBUG_ADD_TASK_SWITCH( PMSG, SUSPEND_INDEX, RESUME_INDEX )	\
	do {																\
		ITCMsg_t *pmsg = ( ITCMsg_t* )( PMSG );							\
		long suspend_index = ( SUSPEND_INDEX );							\
		long resume_index = ( RESUME_INDEX );							\
		pmsg->__debug.ts.suspend_idx = suspend_index;					\
		pmsg->__debug.ts.resume_idx = resume_index;						\
	} while(0)

#define	ITC_DEBUG_ADD_ADD_TIME( PMSG, TIME )	( (PMSG)->__debug.add_time = (TIME) )

#define	ITC_DEBUG_ADD_EXE_TIME( PMSG, TIME )	( (PMSG)->__debug.exe_time = (TIME) )

#else

#define	ITC_DEBUG_ADD_SOURCE( PMSG, SRC )
#define	ITC_DEBUG_ADD_TASK_SWITCH( PMSG, SUSPEND_INDEX, RESUME_INDEX )
#define	ITC_DEBUG_ADD_ADD_TIME( PMSG, TIME )
#define	ITC_DEBUG_ADD_EXE_TIME( PMSG, TIME )

#endif	/* NDEBUG */
		

/**
 * Initialize a ITC_t structure.
 * @param itc Pointer to an ITC_t structure.
 * @return Zero on success, -1 on error.
 */
/* Async-signal-safe: Yes */
int ITCInit( ITC_t *itc );

/**
 * Destroy a ITC_t structure.
 * @param itc Pointer to an ITC_t structure.
 */
/* Async-signal-safe: Yes */
void ITCDestroy( const ITC_t *itc );

/**
 * Bind destination ITC when sending.
 * @param itc Pointer to an ITC_t structure.
 * @param dst Pointer to destination ITC_t structure.
 * @return Zero on success, -1 on error.
 */
/* Async-signal-safe: Yes */
int ITCBind( ITC_t *itc, const ITC_t *dst );

/**
 * Receive message.
 * @param itc Pointer to an ITC_t structure.
 * @param msg Pointer to an ITCMsg_t structure for storing message.
 * @return Size of bytes received, -1 on error.
 */
/* Async-signal-safe: Yes */
int ITCRecv( const ITC_t *itc, ITCMsg_t *msg );

/**
 * Send message to a destination.
 * @param dst Pointer to a destination ITC_t structure.
 * @param msg Pointer to an ITCMsg_t structure.
 * @return Size of bytes sent, -1 on error.
 */
/* Async-signal-safe: Yes */
int ITCSendTo( const ITC_t *dst, const ITCMsg_t *msg );

/**
 * Send message to binded destination.
 * @param src Pointer to a source ITC_t structure.
 * @param msg Pointer to an ITCMsg_t structure.
 * @return Size of bytes sent, -1 on error.
 */
/* Async-signal-safe: Yes */
int ITCSend( const ITC_t *src, const ITCMsg_t *msg );

/**
 * Get read fd from ITC_t object.
 * @param itc Pointer to an ITC_t structure.
 * @return Read fd.
 */
/* Async-signal-safe: Yes */
static inline int ITCGetReadFd( const ITC_t *itc )
{
	return itc->own.rfd;
}

/**
 * Print ITCMsg_t structure.
 * @param pxMsg Pointer to a ITCMsg_t structure.
 */
/* Async-signal-safe: No */
void vITCMsgPrint( const ITCMsg_t *msg );

/**
 * Get ITCMsg_t operation name.
 * @param idx Operation index.
 * @return Pointer to the name of the operation, on success. NULL, on error.
 */
/* Async-signal-safe: Yes */
const char* xITCGetOpName( int idx );


#endif /* ITC_H_ */
