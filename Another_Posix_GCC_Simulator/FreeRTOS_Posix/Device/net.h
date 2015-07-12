/*
 * net.h
 *
 *  Created on: 2010-12-13
 *      Author: Hongzhi Song
 */

#ifndef NET_H_
#define NET_H_

#include "Device.h"

/**
 * Initialize net devices.
 * @param pxDevAggr Pointer to a already initialized device aggregate.
 * @param pxAIOObject A pointer to an AIO object according to iMode.
 * @return 0 if no error occurred, -1 on error.
 */
int iNetInit( xDeviceAggr_t *pxDevAggr );

/**
 * Destroy net devices.
 * @return 0 if no error occurred, -1 on error.
 */
int iNetDestroy( void );

/**
 * Read from a net device. Block until data is available.
 * @param iNetDev A net device number.
 * @param pvBuf The buffer that receives data.
 * @param iLen Length of the receiving buffer.
 * @return Length of bytes read, -1 on error.
 */
int iNetRead( int iNetDev, void *pvBuf, int iLen );

/**
 * Write to a net device.
 * @param iNetDev A net device number.
 * @param pvBuf Pointer to the data buffer.
 * @param iLen Length of bytes to be written.
 * @return Length of bytes written, -1 on error.
 */
int iNetWrite( int iNetDev, const void *pvBuf, int iLen );


#endif /* NET_H_ */
