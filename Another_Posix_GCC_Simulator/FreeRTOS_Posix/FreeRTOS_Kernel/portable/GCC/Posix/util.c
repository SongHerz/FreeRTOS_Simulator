/*
 * util.c
 *
 *  Created on: 2010-12-27
 *      Author: Hongzhi Song
 */

#include <unistd.h>
#include <errno.h>
#include "util.h"

int write_all( int fd, const void *buf, int len )
{
	int nwritten = 0;
	int ret;

	do {
		if ( (ret = write(
						fd,
						(const char*)(buf) + nwritten,
						len - nwritten )) == -1 )
		{
			if ( errno == EINTR )
				continue;
			else
				return -1;
		}
		nwritten += ret;
	} while ( nwritten < len );

	return nwritten;
}


int read_all( int fd, void *buf, int len )
{
	int nread = 0;
	int ret;

	do {
		if ( (ret = read( fd, buf + nread, len - nread )) == -1 )
		{
			if ( errno == EINTR )
				continue;
			else
				return -1;
		}
		nread += ret;
	} while ( nread < len );

	return nread;
}


int strlen_safe( const char *str )
{
	int i = 0;
	while( str[i] != '\0' )
		i++;

	return i;
}
