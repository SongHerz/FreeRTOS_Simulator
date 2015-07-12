/*
 * AIOUnixSocket.h
 *
 *  Created on: 2010-11-30
 *      Author: song
 */

#ifndef AIOUNIXSOCKET_H_
#define AIOUNIXSOCKET_H_

#include "AIO.h"

typedef void (*UnixSocketReadCallbackType)(void *pvBuf, int iLength, void *pvContext);

typedef struct AIO_UNIX_SOCKET
{
	int iSocketFd;
	UnixSocketReadCallbackType pvReadFunction;
	struct AIO_UNIX_SOCKET *pxNext;
} xAIOUnixSocket;

/**
 * Opens a Unix socket.
 * @param pxAIOUnixSocket A pointer to a xAIOUnixSocket object that stores information.
 * @param pcSrvPath A pathname that presents a server socket.
 * This socket should have been bind() by some process.
 * @param pcCliPath A pathname that presents a client socket for binding.
 * This socket should not be created by any process, and it is created by this function.
 * @return 0 on success, -1 on error.
 */
int iAIOUnixOpen( xAIOUnixSocket * pxAIOUnixSocket, const char *pcSrvPath, const char *pcCliPath );

/**
 * Used to register a callback function for notification when an asynchronous read is completed.
 * @param pxAIOUnixSocket A pointer to a xAIOUnixSocket object that stores information.
 * @param pvFunction The callback function that receives the file descriptor and pvContext when the IO completes.
 * @param pvContext A caller supplied parameter for the pvFunction.
 * @return 0 on success, -1 on error.
 */
int iAIOUnixRegisterReadCallback( xAIOUnixSocket * pxAIOUnixSocket, UnixSocketReadCallbackType pvFunction, void *pvContext );

/**
 * Closes the socket and removes the call back function.
 * @param pxAIOUnixSocket A pointer to a xAIOUnixSocket object that stores information.
 * @return 0 on success, -1 on error.
 */
int iAIOUnixClose( const xAIOUnixSocket * pxAIOUnixSocket );

/**
 * Used to initialize an AIO read. When read completes, a signal will be raised.
 * @param pxAIOUnixSocket A pointer to a xAIOUnixSocket object that stores information.
 * @param pvBuf The buffer that receives data.
 * @param iLength Length of the receiving buffer.
 * @return 0 on success, -1 on error.
 */
int	iAIOUnixRead( const xAIOUnixSocket *pxAIOUnixSocket, void *pvBuf, int iLength);

/**
 * Wait for AIO read.
 * @param pxAIOUnixSocket A pointer to a xAIOUnixSocket object that stores information.
 * @return 0 if read completes succefully, -1 on error.
 */
int iAIOUnixSuspend( const xAIOUnixSocket *pxAIOUnixSocket );

/**
 * Write data out to other process.
 * @param pxAIOUnixSocket A pointer to a xAIOUnixSocket object that stores information.
 * @param pvBuf A pointer to the data to be sent.
 * @param iLength Length of the data to be sent.
 * @return The number of bytes transmitted, -1 on error.
 */
int iAIOUnixWrite( const xAIOUnixSocket * pxAIOUnixSocket, const void *pvBuf, int iLength );

#endif /* AIOUNIXSOCKET_H_ */
