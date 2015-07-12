/*
 * AIO.h
 *
 *  Created on: 2010-11-26
 *      Author: Hongzhi Song
 */

#ifndef AIO_H_
#define AIO_H_

typedef void (*AIOCallbackType)( int iFileDescriptor, void * pvBuf, int iLength, void *pvContext );

/**
 * Used to register a callback function for notification when an asynchronous IO action is completed.
 * @param iFileDescriptor The File descriptor used for the asynchronous IO.
 * @param pvFunction The callback function that receives the file descriptor and pvContext when the IO completes.
 * @param pvContext A caller supplied parameter for the pvFunction.
 * @return 0 if registered successfully, -1 on error.
 */
int iAIORegisterCallback( int iFileDescriptor, AIOCallbackType pvFunction, void *pvContext );

/**
 * Removes the registered call back from the list.
 * @param iFileDescriptor The file descriptor for an already closed file handle.
 */
void vAIOUnregisterCallback( int iFileDescriptor );

/**
 * Used to initialize an AIO read. When read completes, a signal will be raised.
 * @param iFileDescriptor The File descriptor used for the asynchronous IO.
 * @param pvBuf The buffer that receives data.
 * @param iLength Length of the receiving buffer.
 * @return 0 if read operation initialized succefully, -1 on error.
 */
int iAIORead( int iFileDescriptor, void *pvBuf, int iLength );

/**
 * Wait for AIO read.
 * @param iFileDescriptor The File descriptor used for the asynchronous IO.
 * @return 0 if read completes succefully, -1 on error.
 */
int iAIOSuspend( int iFileDescriptor );

/**
 * This is a synchronous write function, and it will write all data.
 * @param iFileDescriptor A File descriptor.
 * @param pvBuf Pointer to a buffer.
 * @param iLength Length of the buffer.
 * @return Number of bytes written, -1 on error.
 */
int iAIOWrite( int iFileDescriptor, const void *pvBuf, int iLength );


#endif /* AIO_H_ */
