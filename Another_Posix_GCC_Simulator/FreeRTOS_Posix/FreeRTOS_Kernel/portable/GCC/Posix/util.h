/*
 * util.h
 *
 *  Created on: 2010-12-27
 *      Author: Hongzhi Song
 */

#ifndef UTIL_H_
#define UTIL_H_

#define BlockOnError( promption )		\
	do {								\
		printf( "%s from %s: %s\n",		\
				__func__,				\
				__FILE__,				\
				promption );			\
		abort();						\
		sleep( 100000 );				\
	} while(0)


/**
 * Write all bytes. Do not return until all bytes are written.
 * @param fd A File descriptor.
 * @param buf Pointer to a buffer.
 * @param len Length of the buffer.
 * @return Number of bytes written, -1 on error.
 */
/* Async-signal-safe: Yes */
int write_all( int fd, const void *buf, int len );

/**
 * Read all bytes. Do not return until all bytes are read.
 * @param fd A File descriptor.
 * @param buf Pointer to a buffer.
 * @param len Length of the buffer.
 * @return Number of bytes read, -1 on error.
 */
/* Async-signal-safe: Yes */
int read_all( int fd, void *buf, int len );

/**
 * A signal safe strlen.
 * @param str A pointer to a string.
 * @return Number of chars.
 */
/* Async-signal-safe: Yes */
int strlen_safe( const char *str );

#endif /* UTIL_H_ */
