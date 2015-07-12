/*
 * Device.c
 *
 *  Created on: 2010-12-3
 *      Author: Hongzhi Song
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "AIO/AIOUnixSocket.h"
#include "Device.h"
#include "frame.h"


#undef portENTER_CRITICAL
#define portENTER_CRITICAL()
#undef portEXIT_CRITICAL
#define portEXIT_CRITICAL()

/*---------------------------------------------------------------------------*/

/* Get a free Device Descriptor
 * Return a device descriptor on success, -1 on error
 */ 
static int prvGetFreeDevDescriptor( const xDeviceAggr_t *pxDevAggr );
/* A read callback function */
static void prvReadCallback( void *pvBuf, int iLen, void *pvContext );
/* Make a map between a Device Number and a Device Descriptor.
 * Return 0 on success, -1 on error
 */ 
static int prvDevNumDescriptorMap( xDeviceAggr_t *pxDevAggr, int iDevNum, int iDevDes );
/* Get a device descriptor from a device number.
 * Return device descriptor on success, -1 on error.
 */ 
static inline int prvDN2DD( const xDeviceAggr_t *pxDevAggr, int iDevNum );
/* Get a device number from a device descriptor.
 * Return device number on success, -1 on error.
 */
static inline int prvDD2DN( const xDeviceAggr_t *pxDevAggr, int iDevDescriptor );

/*---------------------------------------------------------------------------*/

int prvGetFreeDevDescriptor( const xDeviceAggr_t *pxDevAggr )
{
	int iIndex;
	int iDevDescriptor = -1;

	for ( iIndex = 0; iIndex < MAX_DEVICES; iIndex++ )
	{
		if ( pxDevAggr->xDevices[ iIndex ].xReadSem == NULL )
		{
			iDevDescriptor = iIndex;
			break;
		}
	}

	return iDevDescriptor;
}
/*---------------------------------------------------------------------------*/

void prvReadCallback( void *pvBuf, int iLen, void *pvContext )
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xDeviceAggr_t *pxDevAggr = (xDeviceAggr_t*)pvContext;
	xDevHead_t *pxDevHead;
	int iFrameLen;		/* Frame Length */
	int iFrameDataLen;	/* Frame Data Length */
	int iDN;			/* Device Number */
	int iDD;			/* Device Descriptor */

	/* Check data Length */
	if ( iLen < sizeof( xDevHead_t ) )
	{
		printf( "%s from %s: Data is too short\n", __func__, __FILE__ );
		return;
	}

	/* Extract device header */
	pxDevHead = ( xDevHead_t * )pvBuf;
	/* Check frame length */
	iFrameLen = sizeof(*pxDevHead) + xFrameGetDataLen( pxDevHead );
	if ( iFrameLen != iLen )
	{
		printf( "%s from %s: "
				"Frame length (%d bytes) does not equal "
				"received data length (%d bytes).\n",
				__func__, __FILE__, iFrameLen, iLen );
		return;
	}
	/* Get device number */
	iDN = xFrameGetDst( pxDevHead );
	/* Check device number */
	if ( iDN < 0 || iDN >= MAX_DEVICES )
	{
		printf( "%s from %s: Invalid device number %d\n",
				__func__, __FILE__, iDN );
		return;
	}
	/* Map device number to device descriptor */
	if ( (iDD = prvDN2DD( pxDevAggr, iDN )) < 0 )
	{
		printf( "%s from %s: Cannot map device number %d to a device descriptor\n",
				__func__, __FILE__, iDN);
		return;
	}

	/*
	 * Dispatch data to a specific device
	 */
	iFrameDataLen = xFrameGetDataLen( pxDevHead );
	/* Check if the device can receive the data */
	if ( pxDevAggr->xDevices[ iDD ].iLen < iFrameDataLen )
	{
		printf( "%s from %s: Device %d read buffer (%d bytes) "
				"is smaller than frame data length (%d bytes).\n",
				__func__, __FILE__, iDN,
				pxDevAggr->xDevices[ iDD ].iLen,
				iFrameDataLen );
		return;
	}
	/* OK, now copy data to a specific device */
	memcpy( pxDevAggr->xDevices[ iDD ].pvBuf, pvBuf + sizeof(*pxDevHead), iFrameDataLen );
	/* Set actual read length */
	pxDevAggr->xDevices[ iDD ].iActLen = iFrameDataLen;


	/* Just give the semaphore */
	if ( xSemaphoreGiveFromISR ( pxDevAggr->xDevices[ iDD ].xReadSem, &xHigherPriorityTaskWoken ) != pdTRUE )
	{
		printf( "%s from %s: Cannot give semaphore\n",
				__func__, __FILE__ );
	}

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*---------------------------------------------------------------------------*/

int prvDevNumDescriptorMap( xDeviceAggr_t *pxDevAggr, int iDevNum, int iDevDes )
{
	if ( pxDevAggr->iDN2DD[ iDevNum ] != -1 && pxDevAggr->iDD2DN[ iDevDes ] != -1 )
	{
		/* The device number/descriptor has already been mapped */
		return -1;
	}
	else
	{
		pxDevAggr->iDN2DD[ iDevNum ] = iDevDes;
		pxDevAggr->iDD2DN[ iDevDes ] = iDevNum;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/

int prvDN2DD( const xDeviceAggr_t *pxDevAggr, int iDevNum )
{
	return pxDevAggr->iDN2DD[ iDevNum ];
}
/*---------------------------------------------------------------------------*/

int prvDD2DN( const xDeviceAggr_t *pxDevAggr, int iDevDescriptor )
{
	return pxDevAggr->iDD2DN[ iDevDescriptor ];
}
/*---------------------------------------------------------------------------*/

int iDevAggrInit( xDeviceAggr_t *pxDevAggr, int iMode, const void *pxAIOObject )
{
	int i;

	portENTER_CRITICAL();

	pxDevAggr->pxAIOObject = (void*)pxAIOObject;

	/* Assign proper read, write and close methods
	 * And, register callback for read.
	 */
	switch ( iMode )
	{
		case DM_UNIX:
			pxDevAggr->piAIORead = ( piAIORead_t )( iAIOUnixRead );
			pxDevAggr->piAIOWrite = ( piAIOWrite_t )( iAIOUnixWrite );
			pxDevAggr->piAIOClose = ( piAIOClose_t )( iAIOUnixClose );
			/* Register callback for read */
			if ( iAIOUnixRegisterReadCallback(
					( xAIOUnixSocket* )( pxAIOObject ),
					prvReadCallback,
					pxDevAggr
				) < 0 )
			{
				portEXIT_CRITICAL();
				return -1;
			}
			break;

		default:
			printf("%s from %s: Unknown mode\n", __func__, __FILE__);
			portEXIT_CRITICAL();
			return -1;
			break;
	}

	/* Clear devices */
	memset( pxDevAggr->xDevices, 0, sizeof( pxDevAggr->xDevices ));

	/* Initialize iDN2DD[], iDD2DN[] array */
	for( i = 0; i < MAX_DEVICES; i++ )
	{
		pxDevAggr->iDN2DD[ i ] = -1;
		pxDevAggr->iDD2DN[ i ] = -1;
	}

	portEXIT_CRITICAL();

	return 0;
}
/*---------------------------------------------------------------------------*/

int iDevAggrDestroy( xDeviceAggr_t *pxDevAggr )
{
	portENTER_CRITICAL();
	/* Just close communication method */
	if ( pxDevAggr->piAIOClose( pxDevAggr->pxAIOObject ) < 0 )
	{
		portEXIT_CRITICAL();
		return -1;
	}

	portEXIT_CRITICAL();
	return 0;
}
/*---------------------------------------------------------------------------*/

int iDevOpen( xDeviceAggr_t *pxDevAggr, int iDevNum )
{
	int iDD;	/* Device Descriptor */

	portENTER_CRITICAL();

	/* Get a device descriptor */
	iDD = prvGetFreeDevDescriptor( pxDevAggr );
	if ( iDD < 0 )
	{
		printf( "%s from %s: MAXIMUM devices has been open\n",
				__func__, __FILE__);
		portEXIT_CRITICAL();
		return -1;
	}

	/* Map iDevNum to the device descriptor */
	if ( prvDevNumDescriptorMap( pxDevAggr, iDevNum, iDD ) < 0 )
	{
		printf( "%s from %s: Device Number %d has already been mapped\n",
				__func__, __FILE__, iDevNum);
		portEXIT_CRITICAL();
		return -1;
	}


	/* Last, create a semaphore */
	vSemaphoreCreateBinary( pxDevAggr->xDevices[ iDD ].xReadSem );
	if ( pxDevAggr->xDevices[ iDD ].xReadSem == NULL )
	{
		portEXIT_CRITICAL();
		return -1;
	}
	/* Count down the semaphore to zero */
	(void)xSemaphoreTake( pxDevAggr->xDevices[ iDD ].xReadSem, 0 );

	portEXIT_CRITICAL();

	return iDD;
}
/*---------------------------------------------------------------------------*/

int iDevRead( xDeviceAggr_t *pxDevAggr, int iDevDescriptor, void *pvBuf, int iLen )
{
	int iRet;

	portENTER_CRITICAL();

	/* Set read parameters */
	pxDevAggr->xDevices[ iDevDescriptor ].pvBuf = pvBuf;
	pxDevAggr->xDevices[ iDevDescriptor ].iLen = iLen;

	/* Initialize a read */
	if ( pxDevAggr->piAIORead( pxDevAggr->pxAIOObject, pxDevAggr->gcDevBuf, GLOBAL_DEV_BUF_LEN ) < 0 )
	{
		portEXIT_CRITICAL();
		return -1;
	}

	/* Take and wait the the semaphore */
	while( xSemaphoreTake( pxDevAggr->xDevices[ iDevDescriptor ].xReadSem, portMAX_DELAY ) != pdTRUE );

	/* Return actual read length */
	iRet = pxDevAggr->xDevices[ iDevDescriptor ].iActLen;

	portEXIT_CRITICAL();

	return iRet;
}
/*---------------------------------------------------------------------------*/

int iDevWrite( const xDeviceAggr_t *pxDevAggr, int iDevDescriptor, const void *pvBuf, int iLen )
{
	static char cBuf[ GLOBAL_DEV_BUF_LEN ];
	xDevHead_t *pxHead;
	int iActLen;	/* Actual send data length */
	int iDN;		/* device number */
	int iRet;

	portENTER_CRITICAL();
	/*
	 * Write data out according to protocol
	 */
	/* Check if the frame will exccess the buffer length */
	if ( ( iActLen = sizeof( xDevHead_t ) + iLen ) > sizeof( cBuf ) )
	{
		printf( "%s from %s: Send data is too long\n",
				__func__, __FILE__ );
		portEXIT_CRITICAL();
		return -1;
	}
	/* Check if device number is valid */
	if ( (iDN = prvDD2DN( pxDevAggr, iDevDescriptor )) < 0 )
	{
		printf( "%s from %s: Invalid device descriptor %d\n",
				__func__, __FILE__, iDevDescriptor );
		portEXIT_CRITICAL();
		return -1;
	}
	/* Fill the frame header */
	pxHead = (xDevHead_t*)cBuf;
	/* When sending, destination and time stamp not used yet */
	vFrameSet( pxHead, 0, iDN, 0, iLen );
	memcpy( cBuf + sizeof(*pxHead), pvBuf, iLen );

	iRet = pxDevAggr->piAIOWrite( pxDevAggr->pxAIOObject, cBuf, iActLen );

	/* Bytes should be written out completely */
	assert( iRet == iActLen );

	portEXIT_CRITICAL();

	/* Return number of bytes written except device frame header */
	return iRet - sizeof( *pxHead );
}
/*---------------------------------------------------------------------------*/

int iDevClose( xDeviceAggr_t *pxDevAggr, int iDevDescriptor )
{
	int iDN;	/* Device Number */

	portENTER_CRITICAL();

	/* Find the device descriptor */
	if ( (iDN = prvDD2DN( pxDevAggr, iDevDescriptor )) < 0 )
	{
		printf( "%s from %s: Invalid device descriptor %d\n",
				__func__, __FILE__, iDevDescriptor );
		portEXIT_CRITICAL();
		return -1;
	}
	/* Clear device descriptor/number maps */
	pxDevAggr->iDD2DN[ iDevDescriptor ] = -1;
	pxDevAggr->iDN2DD[ iDN ] = -1;

	portEXIT_CRITICAL();
	return 0;
}
/*---------------------------------------------------------------------------*/
