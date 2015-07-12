/*
 * fifo.h
 *
 *  Created on: 2010-12-17
 *      Author: Hongzhi Song
 */

#ifndef FIFO_H_
#define FIFO_H_


/*
 * Memory area:
 * ----------------------------------------------------------------
 * | Element0 | Element1 | ... | Element last | Other memory area
 * ----------------------------------------------------------------
 *  ^                                          ^
 *  |                                          |
 *  pxBegin                                    pxEnd
 */
struct xFIFO {
	void *pxBegin;	/* Pointer to the beginning of memory area */
	void *pxEnd;	/* Pointer to the end of memory area */
	void *pxR;		/* Read pointer */
	void *pxW;		/* Write pointer */
	int iElemSize;	/* Element size */
};


/**
 * Initialize a FIFO.
 * @param pxFIFO Pointer to an uninitialized xFIFO structure which saves newly created FIFO.
 * @param iNodeSize Size of an element in a FIFO.
 * @param iNodeCount Max number of elements in a FIFO.
 * @return 0 If initialize succefully, -1 on error.
 */
int xFIFOInit( struct xFIFO *pxFIFO, int iElemSize, int iElemMaxCount );

/**
 * Destroy a FIFO.
 * @param pxFIFO Pointer to an initialized xFIFO structure.
 */
void vFIFODestroy( struct xFIFO *pxFIFO );

/**
 * Push an element to a FIFO.
 * @param pxFIFO Pointer to a initialized xFIFO structure.
 * @param pxElem Pointer to an element to be inserted in to a FIFO.
 * @return 0 If inserted succefully, -1 on error.
 */
int xFIFOPush( struct xFIFO *pxFIFO, const void *pxElem );

/**
 * Return a pointer to the first element in a FIFO.
 * @param pxFIFO Pointer to an initialized xFIFO structure.
 * @return Pointer to an element, NULL if FIFO is empty.
 */
void *xFIFOFront( struct xFIFO *pxFIFO );

/**
 * Pop an element from a FIFO.
 * @param pxFIFO Pointer to an initialized xFIFO structure.
 */
void vFIFOPop( struct xFIFO *pxFIFO );

/**
 * Tell if a fifo is empty.
 * @param pxFIFO Pointer to an initialized xFIFO structure.
 * @return Non-zero if fifo is emtpy, otherwise, zero.
 */
int xFIFOEmpty( const struct xFIFO *pxFIFO );

/**
 * Tell if a fifo is full.
 * @param pxFIFO Pointer to an initialized xFIFO structure.
 * @return Non-zero if fifo is full, otherwise, zero.
 */
int xFIFOFull( const struct xFIFO *pxFIFO );

#endif /* FIFO_H_ */
