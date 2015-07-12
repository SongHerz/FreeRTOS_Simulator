/*
 * rbsem.c
 *
 *  Created on: 2011-1-7
 *      Author: Hongzhi Song
 */

#include <unistd.h>
#include <errno.h>
#include "util.h"
#include "rbsem.h"

#define	RBSEM_CHAR	'r'

int RBSemInit( RBSem_t *psem )
{
	return pipe( psem->_pipefd );
}

void RBSemDestroy( const RBSem_t *psem )
{
	(void) close( psem->_pipefd[0] );
	(void) close( psem->_pipefd[1] );
}

int RBSemPost( const RBSem_t *psem )
{
	char c = RBSEM_CHAR;

	return write_all( psem->_pipefd2.wfd, &c, 1 );
}

int RBSemWait( const RBSem_t *psem )
{
	char c[2];
	int ret = 0;

	switch( read( psem->_pipefd2.rfd, c, 2 ) )
	{
		case 1:
			/* Check if the character is RBSEM_CHAR */
			if ( c[0] != RBSEM_CHAR )
			{
				errno = EILSEQ;
				ret = -1;
			}
			break;

		case 2:
			/* We get 2 chars */
			errno = EOVERFLOW;
			ret = -1;
			break;

		default:
			/* read error */
			ret = -1;
			break;
	}

	return ret;
}
