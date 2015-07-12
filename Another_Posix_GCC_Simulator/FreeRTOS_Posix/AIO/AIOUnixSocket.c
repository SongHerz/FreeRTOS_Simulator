/*
 * AIOUnixSocket.c
 *
 *  Created on: 2010-11-30
 *      Author: song
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include "AIOUnixSocket.h"


static xAIOUnixSocket xHead = {0, NULL, NULL};
/*---------------------------------------------------------------------------*/

/* Return 0 if inserted succefully, -1 on error */
static int prvAIOUnixSocketInsert ( const xAIOUnixSocket *pxAIOUnixSocket );
/* Return a pointer to a xAIOUnixSocket object, if mapped by key */
static xAIOUnixSocket * prvAIOUnixSocketSearch ( int iSocketFd );
/* Return 0 if removed succefully, -1 on error */
static int prvAIOUnixSocketRemove ( const xAIOUnixSocket *pxAIOUnixSocket );

static void prvUnixSocketCallback( int iSocketFd, void * pvBuf, int iLength, void *pvContext );
/*---------------------------------------------------------------------------*/

int iAIOUnixOpen( xAIOUnixSocket * pxAIOUnixSocket, const char *pcSrvPath, const char *pcCliPath )
{
	int iSocketFd;
	struct sockaddr_un xAddr;
	struct sockaddr_un xSrvAddr;
	
	/* Check parameters */
	if ( pxAIOUnixSocket == NULL || pcSrvPath == NULL || pcCliPath == NULL )
	{
		errno = EINVAL;
		goto ErrorOut;
	}

	/* Check pcSrvPath existence */
	if ( access( pcSrvPath, F_OK ) < 0 )
		goto ErrorOut;

	/* Create socket */
	iSocketFd = socket( AF_UNIX, SOCK_DGRAM, 0 );
	if ( iSocketFd < 0 )
		goto ErrorOut;

	/* Bind socket */
	xAddr.sun_family = AF_UNIX;
	if ( sizeof( xAddr.sun_path ) - 1 < strlen( pcCliPath ) )
	{
		printf("%s from %s: pcCliPath is too long.\n", __func__, __FILE__);
		goto ErrorForceOut;
	}
	strcpy( xAddr.sun_path, pcCliPath );

	if ( bind( iSocketFd, (struct sockaddr *)&xAddr, sizeof( xAddr ) ) < 0 )
		goto ErrorOut;

	/* Connect to server socket */
	xSrvAddr.sun_family = AF_UNIX;
	if ( sizeof( xSrvAddr.sun_path ) - 1 < strlen( pcSrvPath ) )
	{
		printf("%s from %s: pcSrvPath is too long.\n", __func__, __FILE__);
		goto ErrorForceOut;
	}
	strcpy( xSrvAddr.sun_path, pcSrvPath );

	if ( connect( iSocketFd, (struct sockaddr*)&xSrvAddr, sizeof( xSrvAddr ) ) < 0 )
		goto ErrorOut;

	/* Assign xAIOUnixSocket structure */
	pxAIOUnixSocket->iSocketFd = iSocketFd;
	pxAIOUnixSocket->pvReadFunction = NULL;
	pxAIOUnixSocket->pxNext = NULL;

	/* Insert that structure to chain */
	if ( prvAIOUnixSocketInsert( pxAIOUnixSocket ) < 0 )
	{
		errno = ENOMEM;
		goto ErrorOut;
	}

	return 0;

ErrorOut:
	printf( "%s from %s: ERROR: %s\n", __func__, __FILE__, strerror(errno) );
ErrorForceOut:
	return -1;
}
/*---------------------------------------------------------------------------*/

int iAIOUnixRegisterReadCallback( xAIOUnixSocket * pxAIOUnixSocket, UnixSocketReadCallbackType pvFunction, void *pvContext )
{
	xAIOUnixSocket * pxAIOUS;

	if (	pxAIOUnixSocket == NULL
			|| pxAIOUnixSocket->iSocketFd == 0
			|| pvFunction == NULL )
	{
		errno = EINVAL;
		goto ErrorOut;
	}

	pxAIOUnixSocket->pvReadFunction = pvFunction;
	/* Find the xAIOUnixSocket structure from chain */
	pxAIOUS = prvAIOUnixSocketSearch( pxAIOUnixSocket->iSocketFd );

	if ( pxAIOUS == NULL )
	{
		errno = EINVAL;
		goto ErrorOut;
	}
	pxAIOUS->pvReadFunction = pvFunction;


	return iAIORegisterCallback(
			pxAIOUnixSocket->iSocketFd,
			prvUnixSocketCallback,
			pvContext );

ErrorOut:
	printf( "%s from %s: ERROR: %s\n", __func__, __FILE__, strerror(errno) );
ErrorForceOut:
	return -1;
}
/*---------------------------------------------------------------------------*/

int iAIOUnixClose( const xAIOUnixSocket * pxAIOUnixSocket )
{
	xAIOUnixSocket *pxAIOUS;

	/* Find the xAIOUnixSocket structure from charin */
	pxAIOUS = prvAIOUnixSocketSearch( pxAIOUnixSocket->iSocketFd );

	if (	pxAIOUS != NULL
			&& pxAIOUnixSocket->pvReadFunction == pxAIOUS->pvReadFunction
		)
	{
		(void)close( pxAIOUS->iSocketFd );
		prvAIOUnixSocketRemove( pxAIOUnixSocket );

		return 0;
	}

	errno = EINVAL;
	printf( "%s from %s: ERROR: %s\n", __func__, __FILE__, strerror(errno) );
	return -1;
}
/*---------------------------------------------------------------------------*/

int iAIOUnixRead( const xAIOUnixSocket * pxAIOUnixSocket, void *pvBuf, int iLength )
{
	return iAIORead( pxAIOUnixSocket->iSocketFd, pvBuf, iLength );
}
/*---------------------------------------------------------------------------*/

int iAIOUnixSuspend( const xAIOUnixSocket * pxAIOUnixSocket )
{
	return iAIOSuspend( pxAIOUnixSocket->iSocketFd );
}
/*---------------------------------------------------------------------------*/

int iAIOUnixWrite( const xAIOUnixSocket * pxAIOUnixSocket, const void *pvBuf, int iLength )
{
	return iAIOWrite( pxAIOUnixSocket->iSocketFd, pvBuf, iLength );
}	
/*---------------------------------------------------------------------------*/

int prvAIOUnixSocketInsert( const xAIOUnixSocket *pxAIOUnixSocket )
{
	xAIOUnixSocket *pxIterator = &xHead;

	/* Iterate till the tail */
	for ( ; pxIterator->pxNext != NULL; pxIterator = pxIterator->pxNext );

	pxIterator->pxNext = malloc( sizeof( xAIOUnixSocket ) );

	if ( pxIterator->pxNext == NULL )
		return -1;

	pxIterator->pxNext->iSocketFd = pxAIOUnixSocket->iSocketFd;
	pxIterator->pxNext->pvReadFunction = pxAIOUnixSocket->pvReadFunction;
	pxIterator->pxNext->pxNext = NULL;

	return 0;
}
/*---------------------------------------------------------------------------*/

xAIOUnixSocket * prvAIOUnixSocketSearch ( int iSocketFd )
{
	xAIOUnixSocket *pxIterator = xHead.pxNext;

	for (	;
			pxIterator != NULL && pxIterator->iSocketFd != iSocketFd;
			pxIterator = pxIterator->pxNext
		);

	return pxIterator;
}
/*---------------------------------------------------------------------------*/

int prvAIOUnixSocketRemove( const xAIOUnixSocket *pxAIOUnixSocket )
{
	xAIOUnixSocket *pxIterator;

	/* Find the xAIOUnixSocket structure from chain */
	for(	pxIterator = &xHead;
			pxIterator->pxNext != NULL
			&& pxIterator->pxNext->iSocketFd != pxAIOUnixSocket->iSocketFd;
			pxIterator = pxIterator->pxNext
	   );


	if (	pxIterator->pxNext != NULL
			&& pxIterator->pxNext->pvReadFunction == pxAIOUnixSocket->pvReadFunction
		)
	{

		xAIOUnixSocket *pxDel = pxIterator->pxNext;
		pxIterator->pxNext = pxIterator->pxNext->pxNext;
		free( pxDel );

		return 0;
	}

	errno = EINVAL;
	return -1;
}
/*---------------------------------------------------------------------------*/

void prvUnixSocketCallback( int iSocketFd, void * pvBuf, int iLength, void *pvContext )
{
	xAIOUnixSocket *pxAIOUS;

	/* Find the xAIOUnixSocket structure from chain */
	pxAIOUS = prvAIOUnixSocketSearch( iSocketFd );

	if ( pxAIOUS != NULL )
		pxAIOUS->pvReadFunction( pvBuf, iLength, pvContext );
	else
		printf( "%s from %s: No read handler for file descriptor %d\n",
				__func__, __FILE__, iSocketFd );
}
/*---------------------------------------------------------------------------*/
