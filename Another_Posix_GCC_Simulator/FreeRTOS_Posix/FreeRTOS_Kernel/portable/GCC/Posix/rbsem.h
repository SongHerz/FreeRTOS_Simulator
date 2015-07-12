/*
 * rbsem.h
 *
 *  Created on: 2011-1-7
 *      Author: Hongzhi Song
 */

#ifndef RBSEM_H_
#define RBSEM_H_

/* This is a restricted binary semaphore.
 * Semaphore unlock will success, only if it is
 * posted once.
 * And this is async-signal-safe.
 */

typedef union {
	int _pipefd[2];
	struct {
		int rfd;
		int wfd;
	} _pipefd2;
} RBSem_t;

#define	RBSemCleanUp( psem )		\
	do {							\
		RBSem_t *_psem = ( psem );	\
		_psem->_pipefd[0] = -1;		\
		_psem->_pipefd[1] = -1;		\
	} while(0)

/**
 * Initialize a restricted binary semaphore RBSem_t object.
 * @param psem Pointer to a RBSem_t object.
 * @return Zero on success, -1 on error.
 */
/* Async-signal-safe: Yes */
int RBSemInit( RBSem_t *psem );

/**
 * Destroy a RBSem_t object.
 * @param psem Pointer to a RBSem_t object.
 */
/* Async-signal-safe: Yes */
void RBSemDestroy( const RBSem_t *psem );

/**
 * Increase (unlock) restricted binary semaphore.
 * @param psem Pointer to a RBSem_t object.
 * @return Zero on success, -1 on error.
 */
/* Async-signal-safe: Yes */
int RBSemPost( const RBSem_t *psem );

/**
 * Decrease (lock) binary semaphore.
 * @param psem Pointer to a RBSem_t object.
 * @return zero, on success, means it has been posted once,
 *         -1, on error, means it has been posted more than once or other error occured.
 */
/* Async-signal-safe: Yes */
int RBSemWait( const RBSem_t *psem );

#endif /* RBSEM_H_ */
