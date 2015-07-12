/*
 * fifo.c
 *
 *  Created on: 2010-12-17
 *      Author: Hongzhi Song
 */

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "fifo.h"

/*
 * Adjust read/write pointer.
 * Return adjusted pointer.
 */
static inline void *prvAdjustPointer( const struct xFIFO *pxFIFO, void *pxp );
/* Tell if FIFO is full.
 * When: ( write pointer + element size == read pointer ) => full
 * Return Non-zero, if it is full. Otherwise, return zero.
 */
static inline int prvFull( const struct xFIFO *pxFIFO );
/* Tell if FIFO is empty.
 * When: ( write pointer == read pointer ) => empty
 * Return Non-zero, if it is empty. Otherwise, return zero.
 */
static inline int prvEmpty( const struct xFIFO *pxFIFO );


void *prvAdjustPointer( const struct xFIFO *pxFIFO, void *pxp )
{
	if ( pxp == pxFIFO->pxEnd )
		return pxFIFO->pxBegin;
	else
		return pxp;
}

int prvFull( const struct xFIFO *pxFIFO )
{
	void *pxWNext;	/* Next write pointer */

	pxWNext = (char*)pxFIFO->pxW + pxFIFO->iElemSize;
	pxWNext = prvAdjustPointer( pxFIFO, pxWNext );
	if( pxWNext == pxFIFO->pxR )
		return 1;
	else
		return 0;
}

int prvEmpty( const struct xFIFO *pxFIFO )
{
	return pxFIFO->pxW == pxFIFO->pxR;
}


int xFIFOInit( struct xFIFO *pxFIFO, int iElemSize, int iElemCount )
{
	/* One more element size should be allocated. */
	int iAllocatedElementCount = iElemCount + 1;
	pxFIFO->pxBegin = calloc( iAllocatedElementCount, iElemSize );

	if( pxFIFO->pxBegin == NULL )
		return -1;

	pxFIFO->pxEnd = (char*)pxFIFO->pxBegin + iElemSize * iAllocatedElementCount;
	pxFIFO->pxR = pxFIFO->pxBegin;
	pxFIFO->pxW = pxFIFO->pxBegin;
	pxFIFO->iElemSize = iElemSize;

	return 0;
}


void vFIFODestroy( struct xFIFO *pxFIFO )
{
	(void)free( pxFIFO->pxBegin );
}


int xFIFOPush( struct xFIFO *pxFIFO, const void *pxElem )
{
	void *pxWNext;

	/* Check if FIFO is full */
	if( prvFull( pxFIFO ) )
	{
		errno = ENOMEM;
		return -1;
	}

	/* OK, insert the element, and then alter write pointer */
	memcpy( pxFIFO->pxW, pxElem, pxFIFO->iElemSize );
	pxWNext = (char*)pxFIFO->pxW + pxFIFO->iElemSize;
	pxWNext = prvAdjustPointer( pxFIFO, pxWNext );
	pxFIFO->pxW = pxWNext;

	return 0;
}


void *xFIFOFront( struct xFIFO *pxFIFO )
{
	/* Check if FIFO is empty */
	if( prvEmpty( pxFIFO ) )
		return NULL;
	else
		return pxFIFO->pxR;
}


void vFIFOPop( struct xFIFO *pxFIFO )
{
	/* Check if FIFO is empty */
	if( prvEmpty( pxFIFO ) )
		return;
	else
	{
		void *pxRNext;

		/* Increase read pointer */
		pxRNext = (char*)pxFIFO->pxR + pxFIFO->iElemSize;
		pxFIFO->pxR = prvAdjustPointer( pxFIFO, pxRNext );
	}
	return;
}


int xFIFOEmpty( const struct xFIFO *pxFIFO )
{
	return prvEmpty( pxFIFO );
}


int xFIFOFull( const struct xFIFO *pxFIFO )
{
	return prvFull( pxFIFO );
}
