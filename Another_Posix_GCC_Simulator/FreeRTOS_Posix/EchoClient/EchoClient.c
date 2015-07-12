/*
 * EchoClient.c
 *
 *  Created on: 2010-12-14
 *      Author: Hongzhi Song
 */

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include "EchoClient/EchoClient.h"
#include "Device/net.h"
#include "Device/frame.h"

#define BUF_SIZE 512

static void random_string(char *buf, int size)
{
	static int first = 1;
	int loops;

	if(first)
	{
		srand(time(NULL));
		first = 0;
	}

	loops = rand() % (size - 1);
	if (loops == 0 )
		loops = 1;
	for(; loops > 0; loops-- )
		*(buf++) = rand()%94 + 33;
	*buf = '\0';
	return;
}


portTASK_FUNCTION( vEchoClient, pvNetNum )
{
	int iNetDev = (int)pvNetNum;
	char buf[BUF_SIZE];

	while(1)
	{
		/* Generate random messages */
		random_string(buf, 50);
		/* TODO: Add some device tag to the message */

		strcpy(buf, "12345");

		/* Send the message out */
		if (iNetWrite( iNetDev, buf, strlen(buf) ) != strlen(buf))
		{
			printf("data written is not the same as data length\n");
		}

		/* Read the message */
		if ( iNetRead( iNetDev, buf, BUF_SIZE ) < 0 )
		{
			printf("data read error\n");
		}
		// printf("Message from server: %s\n", buf);
	}

}
