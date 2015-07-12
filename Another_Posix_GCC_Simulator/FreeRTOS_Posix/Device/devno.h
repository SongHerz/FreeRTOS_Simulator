/*
 * devno.h
 *
 *  Created on: 2010-12-13
 *      Author: Hongzhi Song
 */

#ifndef DEVNO_H_
#define DEVNO_H_

#define	MAX_DEVICES	16

/* Maximum devices should not exceed MAX_DEVICES */
enum {
	DEV_NET0 = 0,	/* Net device 0 */
	DEV_NET1 = 1,	/* Net device 1 */
	DEV_NET2 = 2,	/* Net device 2 */
	DEV_NET3 = 3	/* Net device 3 */
#define	DEV_NET_LAST DEV_NET3
};

#endif /* DEVNO_H_ */
