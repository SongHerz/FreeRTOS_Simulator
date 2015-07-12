/*
 * net.c
 *
 *  Created on: 2010-12-13
 *      Author: Hongzhi Song
 */

#include <stdlib.h>
#include <stdio.h>
#include "Device.h"
#include "net.h"

/* Device aggregate pointer */
static xDeviceAggr_t *_pxDevAggr = NULL;
/* Net device descriptors */
static int *_piDevDes = NULL;
/* Number of net devices */
static int _iDevCount = 0;
/*---------------------------------------------------------------------------*/

int iNetInit( xDeviceAggr_t *pxDevAggr )
{
	int i;

	/* Reserve device aggregate pointer */
	_pxDevAggr = pxDevAggr;

	/* Calculate net devices count */
	_iDevCount = DEV_NET_LAST - DEV_NET0 + 1;

	/* Alloc memory for device descriptors */
	_piDevDes = (int*)calloc( _iDevCount, sizeof(int) );

	/* Open net devices */
	for( i = 0; i < _iDevCount; i++ )
	{
		int iDes = iDevOpen( _pxDevAggr, i + DEV_NET0 );
		if ( iDes < 0 )
		{
			printf( "%s from %s: Cannot open net device %d\n",
					__func__, __FILE__, i );
			return -1;
		}
		_piDevDes[ i ] = iDes;
	}

	return 0;
}
/*---------------------------------------------------------------------------*/

int iNetDestroy( void )
{
	int i;

	for( i = 0; i < _iDevCount; i++ )
	{
		if( iDevClose( _pxDevAggr, _piDevDes[ i ] ) < 0 ) 
		{
			printf( "%s from %s: Cannot close net device %d\n",
					__func__, __FILE__, i);
			return -1;
		}
	}

	return 0;
}
/*---------------------------------------------------------------------------*/

int iNetRead( int iNetDev, void *pvBuf, int iLen )
{
	return iDevRead( _pxDevAggr, _piDevDes[ iNetDev ], pvBuf, iLen );
}
/*---------------------------------------------------------------------------*/

int iNetWrite( int iNetDev, const void *pvBuf, int iLen )
{
	return iDevWrite( _pxDevAggr, _piDevDes[ iNetDev ], pvBuf, iLen );
}
/*---------------------------------------------------------------------------*/
