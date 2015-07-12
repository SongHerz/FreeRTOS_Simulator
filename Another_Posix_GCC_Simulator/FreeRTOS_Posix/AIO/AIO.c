/*
 * AIO.c
 *
 *  Created on: 2010-11-26
 *      Author: Hongzhi Song
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <aio.h>
#include <assert.h>
#ifdef __GCC_POSIX__
#include "FreeRTOS.h"
#endif
#include "util.h"
#include "AIO.h"


/*---------------------------------------------------------------------------*/

#ifndef __GCC_POSIX__
#define SIG_AIO_RX	(SIGRTMIN + 3)
#endif
/*---------------------------------------------------------------------------*/

typedef struct AIO_CALLBACK
{
	int iFileHandle;
	struct aiocb *pxAioCb;
   	AIOCallbackType pvFunction;
	void *pvContext;
	struct AIO_CALLBACK *pxNext;
} xAIOCallback;
/*---------------------------------------------------------------------------*/

void prvAIORunCallback( void *pvCallback );
void prvAIOSignalHandler( int signal, siginfo_t * info, void *pvParam );
void prvAIORegisterSignalHandler( void );
/*---------------------------------------------------------------------------*/

static xAIOCallback xHead = { 0, NULL, NULL, NULL, NULL };
static volatile int iAlreadyRegisteredHandler = 0;
/*---------------------------------------------------------------------------*/

int iAIORegisterCallback( int iFileDescriptor, AIOCallbackType pvFunction, void *pvContext )
{
xAIOCallback *pxIterator;
	if ( NULL != pvFunction)
	{
		/* Record the file against its call back. */
		for ( pxIterator = &xHead; pxIterator->pxNext != NULL; pxIterator = pxIterator->pxNext );

		pxIterator->pxNext = ( xAIOCallback * )malloc( sizeof( xAIOCallback ) );
		pxIterator->pxNext->iFileHandle = iFileDescriptor;
		pxIterator->pxNext->pxAioCb = ( struct aiocb * )malloc( sizeof( struct aiocb ));
		if( pxIterator->pxNext->pxAioCb )
			memset(pxIterator->pxNext->pxAioCb, 0, sizeof(struct aiocb));
		pxIterator->pxNext->pvFunction = pvFunction;
		pxIterator->pxNext->pvContext = pvContext;

		/* Register a signal to process received messages. */
		prvAIORegisterSignalHandler( );
	}
	return  ( NULL != pvFunction
			&& NULL != pxIterator->pxNext
			&& NULL != pxIterator->pxNext->pxAioCb ) ? 0 : -1;
}
/*---------------------------------------------------------------------------*/

void vAIOUnregisterCallback( int iFileDescriptor )
{
xAIOCallback *pxIterator;
xAIOCallback *pxDelete;

	for ( pxIterator = &xHead; ( pxIterator->pxNext != NULL ) && ( pxIterator->pxNext->iFileHandle != iFileDescriptor ); pxIterator = pxIterator->pxNext );
	if ( pxIterator->pxNext != NULL )
	{
		if ( pxIterator->pxNext->iFileHandle == iFileDescriptor )
		{
			if ( pxIterator->pxNext->pxNext != NULL )
			{
				pxDelete = pxIterator->pxNext;
				pxIterator->pxNext = pxDelete->pxNext;
			}
			else
			{
				pxDelete = pxIterator->pxNext;
				pxIterator->pxNext = NULL;
			}
			free( pxDelete->pxAioCb );
			free( pxDelete );
		}
	}
}
/*---------------------------------------------------------------------------*/

void prvAIORunCallback( void *pvCallback )
{
int iReturn;
xAIOCallback *pxCallback;

	pxCallback = ( xAIOCallback* )pvCallback;

	/* Did the request complete? */
	iReturn = aio_error( pxCallback->pxAioCb );
	if ( iReturn == 0 )
	{
		int iLength;
		/* Request completed successfully */
		/* Get the return status */
		iLength = aio_return( pxCallback->pxAioCb );

		/* Now, call callback function */
		( pxCallback->pvFunction )(
				pxCallback->iFileHandle,
				(void*) pxCallback->pxAioCb->aio_buf,
				iLength,
				pxCallback->pvContext
				);
	}
	else
	{
		char *pxErrMsg;
		/* Which error? */
		switch ( iReturn )
		{
			case EINPROGRESS:
				pxErrMsg = "AIO in progress.";
				break;
			case ECANCELED:
				pxErrMsg = "AIO canceled.";
				break;
			default:
				pxErrMsg = strerror( errno );
				break;
		}
		BlockOnError( pxErrMsg );
	}	/* End if ( iReturn == 0 ) */

	return;
}
/*---------------------------------------------------------------------------*/

void prvAIOSignalHandler( int signal, siginfo_t * info, void *pvParam )
{
int errno_saved;

	/* Save error number */
	errno_saved = errno;

	/* Are we in the correct signal handler? */
	if ( SIG_AIO_RX == info->si_signo )
	{
		xAIOCallback *pxCallback = ( xAIOCallback * )info->si_value.sival_ptr;
		/* Add actual processing to bottom half */
		portADD_INTERRUPT_BOTTOM_HALF( prvAIORunCallback, pxCallback );
		/*
		prvAIORunCallback( pxCallback );
		*/
	}

	/* Restore error number */
	errno = errno_saved;
	return;
}
/*---------------------------------------------------------------------------*/

void prvAIORegisterSignalHandler( void )
{
	if ( 1 != iAlreadyRegisteredHandler )
	{
		struct sigaction xAction;
		
		memset( &xAction, 0, sizeof(xAction) );

		xAction.sa_sigaction = prvAIOSignalHandler;
		xAction.sa_flags = SA_SIGINFO;
		sigfillset( &xAction.sa_mask );
		// sigdelset( &xAction.sa_mask, SIG_AIO_RX );
		/* For simplicity we have to mask SIG_TICK */
		/* sigdelset( &xAction.sa_mask, SIG_TICK );	*/
		/* unmask SIGFPE, SIGILL, SIGSEGV, SIGBUS, SIGINT */
		/* The kernel won't mask SIGKILL and SIGSTOP */
		sigdelset( &xAction.sa_mask, SIGFPE );
		sigdelset( &xAction.sa_mask, SIGILL );
		sigdelset( &xAction.sa_mask, SIGSEGV );
		sigdelset( &xAction.sa_mask, SIGBUS );
		sigdelset( &xAction.sa_mask, SIGINT );
		sigdelset( &xAction.sa_mask, SIGABRT);

		/* Register the signal handler. */
		if ( 0 != sigaction( SIG_AIO_RX, &xAction, NULL ) )
			BlockOnError( "Problem installing SIG_AIO_RX" );
		/* Register interrupt signal */
		if ( portREGISTER_INTERRUPT_SIGNAL( SIG_AIO_RX ) == pdFALSE )
			BlockOnError( "Cannot register interrupt signal SIG_AIO_RX" );

		iAlreadyRegisteredHandler = 1;
	}
}	
/*---------------------------------------------------------------------------*/

int iAIORead( int iFileDescriptor, void *pvBuf, int iLength )
{
int iReturn;
xAIOCallback *pxIterator;

	for ( pxIterator = &xHead; ( pxIterator->pxNext != NULL ) && ( pxIterator->pxNext->iFileHandle != iFileDescriptor ); pxIterator = pxIterator->pxNext );

	if ( pxIterator->pxNext != NULL )
	{
		struct aiocb *pxAioCb;

		pxAioCb = pxIterator->pxNext->pxAioCb;
		pxAioCb->aio_fildes = iFileDescriptor;
		pxAioCb->aio_buf = pvBuf;
		pxAioCb->aio_nbytes = iLength;
		pxAioCb->aio_sigevent.sigev_notify = SIGEV_SIGNAL;
		pxAioCb->aio_sigevent.sigev_signo = SIG_AIO_RX;
		pxAioCb->aio_sigevent.sigev_value.sival_ptr = pxIterator->pxNext;

		/* Check aio status */
		if ( aio_error( pxAioCb) != EINPROGRESS )
		{
			/* Initialize a read */
			assert( pxAioCb->aio_sigevent.sigev_value.sival_ptr != NULL );
			iReturn = aio_read( pxAioCb );
			if ( iReturn < 0 )
				printf( "%s from %s: %s\n", __func__, __FILE__, strerror(errno));
		}
		else
		{
			printf( "%s from %s: "
					"Request reading, while read in progress.\n",
					__func__, __FILE__);
			errno = EINPROGRESS;
			iReturn = -1;
		}
	}
	else
	{
		printf( "%s from %s: file descriptor %d not registered\n",
				__func__, __FILE__, iFileDescriptor );
		errno = EINVAL;
		iReturn = -1;
	}

	return iReturn;
}
/*---------------------------------------------------------------------------*/

int iAIOSuspend( int iFileDescriptor )
{
int iReturn;
xAIOCallback *pxIterator;

	for ( pxIterator = &xHead; ( pxIterator->pxNext != NULL ) && ( pxIterator->pxNext->iFileHandle != iFileDescriptor ); pxIterator = pxIterator->pxNext );

	if ( pxIterator->pxNext != NULL )
	{
		struct aiocb *pxAioCb = pxIterator->pxNext->pxAioCb;
		const struct aiocb *xCbList[1] = { pxAioCb };

		do {
			iReturn = aio_suspend( xCbList, 1, NULL );
		} while( iReturn < 0 && errno == EINTR );

		if ( iReturn < 0 )
			printf( "%s from %s: %s\n", __func__, __FILE__, strerror(errno));
	}
	else
	{
		printf( "%s from %s: file descriptor %d not registered\n",
				__func__, __FILE__, iFileDescriptor );
		errno = EINVAL;
		iReturn = -1;
	}

	return iReturn;
}
/*---------------------------------------------------------------------------*/

int iAIOWrite( int iFileDescriptor, const void *pvBuf, int iLength )
{
	int iWritten = 0;
	int iRet;

	do {
		if ( (iRet = write(
						iFileDescriptor,
						(const char*)(pvBuf) + iWritten,
						iLength - iWritten )) == -1 )
		{
			if ( errno == EINTR )
				continue;
			else
				return -1;
		}
		iWritten += iRet;
	} while ( iWritten < iLength );

	return iWritten;
}
/*---------------------------------------------------------------------------*/
