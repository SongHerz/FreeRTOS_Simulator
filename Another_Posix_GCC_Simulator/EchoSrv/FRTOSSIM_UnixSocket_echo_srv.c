#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <aio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <errno.h>
#include <string.h>
#include "../FreeRTOS_Posix/AIO/AIOUnixSocket.h"
#include "..//FreeRTOS_Posix/Device/frame.h"


#define handle_error(msg, errnum) \
  do {  \
    int _errnum = (errnum); \
    printf("%s: %s\n", msg, strerror(_errnum));  \
    exit(EXIT_FAILURE);\
} while (0)


#define	UNIXSOCKET_SRV_NAME	"/tmp/UNIXSOCKET_SRV_NAME"
#define	UNIXSOCKET_CLI_NAME	"/tmp/UNIXSOCKET_CLI_NAME"



#define	BUFSIZE	512
char buf[BUFSIZE];
  
void print_head( const xDevHead_t *pxHead );


void print_head( const xDevHead_t *pxHead )
{
	printf( "destination: %u\n"
		"source:      %u\n"
		"timestamp:   %llu\n"
		"data length: %u\n",
		pxHead->dst,
		pxHead->src,
		pxHead->ts,
		pxHead->len
	      );
}


int main()
{
	int ret;
	int fd;
	struct sockaddr_un sa, sa_cli;
	int not_connected = 1;
	xDevHead_t *pHead;
	long long nMessages = 0;

	(void)unlink( UNIXSOCKET_SRV_NAME );
	(void)unlink( UNIXSOCKET_CLI_NAME );

	sa.sun_family = AF_UNIX;
	strcpy( sa.sun_path, UNIXSOCKET_SRV_NAME );

	sa_cli.sun_family = AF_UNIX;
	strcpy( sa_cli.sun_path, UNIXSOCKET_CLI_NAME );
	

	fd = socket( AF_UNIX, SOCK_DGRAM, 0 );
	if ( fd < 0 )
		handle_error("socket", errno);

	ret = bind( fd, (struct sockaddr *)&sa, sizeof( sa ));
	if ( ret < 0 )
		handle_error( "bind", errno );
	printf( "Unix socket \"%s\" binded\n", UNIXSOCKET_SRV_NAME );

	printf("Waiting for client ...\n");

	while(1)
	{
		int nRead;

		uint16_t dst;
		uint16_t src;
		uint64_t ts;
		uint32_t len;
		void *pbuf;

		/* Read a message */
		printf("Reading ...\n");
		nRead = read(fd, buf, BUFSIZE);
		nMessages++;

		if ( nRead < 0 )
			handle_error("read", errno );
		pHead = (xDevHead_t *) buf;

		print_head( pHead );
		printf("messages: %lld\n", nMessages);

		dst = xFrameGetDst( pHead );
		src = xFrameGetSrc( pHead );
		ts = xFrameGetTS( pHead );
		len = xFrameGetDataLen( pHead );
		pbuf = xFrameGetData( pHead );

		buf[ nRead ] = '\0';

		if(not_connected)
		{
			/* Connect to client */
			ret = connect( fd, (struct sockaddr *)&sa_cli, sizeof( sa_cli ));
			if ( ret < 0 )
				handle_error( "connect", errno );
			printf( "Unix socket \"%s\" connected\n", UNIXSOCKET_CLI_NAME );

			not_connected = 0;
		}

		/* Echo message back */
		printf("Writing:  ");
		pHead = (xDevHead_t*)buf;
		vFrameSetDst( pHead, src );
		vFrameSetSrc( pHead, 0 );
		vFrameSetTS( pHead, 0 );
		vFrameSetDataLen( pHead, strlen( pbuf ));
		ret = write(fd, buf, sizeof(*pHead) + strlen( pbuf ));
		if ( ret < 0 )
			handle_error("write", errno );
		puts( pbuf );

	}

	return 0;
}
