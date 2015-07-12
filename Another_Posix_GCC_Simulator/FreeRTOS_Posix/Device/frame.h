/*
 * frame.h
 *
 *  Created on: 2010-12-6
 *      Author: Hongzhi Song
 */

#ifndef FRAME_H_
#define FRAME_H_

#include <stdint.h>

/* Device communication header.
 * When receiving, time stamp is the time that data arrives a device.
 * When sending, time stamp is the time that data leaves a device.
 * The time stamp will be modified according to channel delay between devices.
 */
typedef struct {
	uint16_t	dst;	/* Destination device */
	uint16_t	src;	/* Source device */
	uint64_t	ts;		/* Time Stamp */
	uint32_t	len;	/* Length of rest data, except this header */
} xDevHead_t;

static inline void vFrameSetDst ( xDevHead_t *pxHead, uint16_t iDst )
{
	pxHead->dst = iDst;
}

static inline void vFrameSetSrc ( xDevHead_t *pxHead, uint16_t iSrc )
{
	pxHead->src = iSrc;
}

static inline void vFrameSetTS ( xDevHead_t *pxHead, uint64_t llTS )
{
	pxHead->ts = llTS;
}

static inline void vFrameSetDataLen ( xDevHead_t *pxHead, uint32_t iLen )
{
	pxHead->len = iLen;
}

static inline uint16_t xFrameGetDst ( const xDevHead_t *pxHead ) { return pxHead->dst; }
static inline uint16_t xFrameGetSrc ( const xDevHead_t *pxHead ) { return pxHead->src; }
static inline uint64_t xFrameGetTS  ( const xDevHead_t *pxHead ) { return pxHead->ts;  }
static inline uint64_t xFrameGetDataLen ( const xDevHead_t *pxHead ) { return pxHead->len; }
static inline void *xFrameGetData( const xDevHead_t *pxHead ) { return (void*)pxHead + sizeof( *pxHead ); }

static inline void vFrameSet ( xDevHead_t *pxHead, uint16_t iDst, uint16_t iSrc, uint64_t llTS, uint32_t iLen )
{
	pxHead->dst = iDst;
	pxHead->src = iSrc;
	pxHead->ts = llTS;
	pxHead->len = iLen;
}

static inline void vFrameGet ( xDevHead_t *pxDst, const xDevHead_t *pxSrc )
{
	pxDst->dst = pxSrc->dst;
	pxDst->src = pxSrc->src;
	pxDst->ts = pxSrc->ts;
	pxDst->len = pxSrc->len;
}



#endif /* FRAME_H_ */
