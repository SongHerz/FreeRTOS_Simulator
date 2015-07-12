/*
 * Device.h
 *
 *  Created on: 2010-12-3
 *      Author: Hongzhi Song
 */

#ifndef DEVICE_H_
#define DEVICE_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "devno.h"

/*---------------------------------------------------------------------------*/

typedef int (*piAIORead_t)	( const void *pxAIOObject, void *pvBuf, int iLen );
typedef int (*piAIOWrite_t)	( const void *pxAIOObject, const void *pvBuf, int iLen );
typedef int (*piAIOClose_t)	( const void *pxAIOObject );

/* Present a device */
typedef struct
{
	xSemaphoreHandle xReadSem;	/* Read semaphore */
	void * volatile pvBuf;		/* Read buffer pointer */
	volatile int iLen;			/* Read buffer length */
	volatile int iActLen;		/* Actual read length */
} xDevice_t;

/* Present a device aggregate, according to communication method */
typedef struct
{
	void *pxAIOObject;			/* Pointer to a real AIO object */
	piAIORead_t  piAIORead;		/* Asynchronous read function */
	piAIOWrite_t piAIOWrite;	/* Synchronous write function */
	piAIOClose_t piAIOClose;	/* Close function */
	/* Devices indexed by device descriptors */
	xDevice_t	 xDevices[ MAX_DEVICES ];
	/* Device Number to Device Descriptor map.
	 * Should be initialized to -1.
	 */ 
	int iDN2DD[ MAX_DEVICES ];
	/* Device Descriptor to Device Number map.
	 * Should be initialized to -1.
	 */
	int iDD2DN[ MAX_DEVICES ];
#define	GLOBAL_DEV_BUF_LEN	2048
	/* Global device buffer.
	 * Data is received and stored to this buffer first,
	 * and then dispatch to individual buffer for each device.
	 */
	char gcDevBuf[ GLOBAL_DEV_BUF_LEN ];
	/* Actual received length */
	int  iLen;
} xDeviceAggr_t;
/*---------------------------------------------------------------------------*/

enum {
	DM_UNIX = 1		/* Device Mode: Unix Socket */
};

/**
 * Initialize a device aggregate.
 * @param pxDevAggr Pointer to a device aggregate to be initialized.
 * @param iMode Specify communication method (now, only DM_UNIX mode is supported).
 * @param pxAIOObject A pointer to an AIO object according to iMode.
 * @return 0 if no error occurred, -1 on error.
 */
/* This function should be called at initialization time before scheduler starting */
int iDevAggrInit( xDeviceAggr_t *pxDevAggr, int iMode, const void *pxAIOObject );

/**
 * Destroy a device aggregate.
 * @param pxDevAggr Pointer to a device aggregate to be Destroied.
 * @return 0 if no error occurred, -1 on error.
 */
/* This function should be called at destruction time after scheduler ending */
int iDevAggrDestroy( xDeviceAggr_t *pxDevAggr );

/**
 * Opens a Unix socket.
 * @param pxDevAggr Pointer to a device aggregate.
 * @param iDevNum Specify device number. It identifies each unique device in the system.
 * @return Device descriptor if no error occurred, -1 on error.
 */
/* This function should be called at initialization time before scheduler starting */
int iDevOpen( xDeviceAggr_t *pxDevAggr, int iDevNum );

/**
 * Read from a device. Block until data is available.
 * @param pxDevAggr Pointer to a device aggregate.
 * @param iDevDescriptor Device descriptor of the device to read from.
 * @param pvBuf The buffer that receives data.
 * @param iLen Length of the receiving buffer.
 * @return Length of bytes read, -1 on error.
 */
/* This function should be CALLED IN A TASK ONLY */
int iDevRead( xDeviceAggr_t *pxDevAggr, int iDevDescriptor, void *pvBuf, int iLen );

/**
 * Write to a device.
 * @param pxDevAggr Pointer to a device aggregate.
 * @param iDevDescriptor Device descriptor of the device to write to.
 * @param pvBuf Pointer to the data buffer.
 * @param iLen Length of bytes to be written.
 * @return Length of bytes written, -1 on error.
 */
/* This function should be CALLED IN A TASK ONLY */
int iDevWrite( const xDeviceAggr_t *pxDevAggr, int iDevDescriptor, const void *pvBuf, int iLen );

/**
 * Close a device.
 * @param pxDevAggr Pointer to a device aggregate.
 * @param iDevDescriptor Device descriptor to the device to be closed.
 * @return 0 if removed succefully, -1 on error.
 */
/* This function should be called at destruction time after scheduler ending */
int iDevClose( xDeviceAggr_t *pxDevAggr, int iDevDescriptor );


#endif /* DEVICE_H_ */
