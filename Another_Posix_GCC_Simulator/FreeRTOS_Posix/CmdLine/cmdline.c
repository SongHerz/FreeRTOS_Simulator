#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include "cmdline.h"

#define	NO_ARGUMENT			0
#define	REQUIRED_ARGUMENT	1
#define	OPTIONAL_AGRUMENT	2


enum {
	HELP = 'h',
	UNIX_SERVER = 0x1FF,
	UNIX_CLIENT,
};


static struct option options[] = {
	{ "unix-server", REQUIRED_ARGUMENT, NULL, UNIX_SERVER },
	{ "unix-client", REQUIRED_ARGUMENT, NULL, UNIX_CLIENT },
	{ "help",  NO_ARGUMENT, NULL, HELP },
	{ 0, 0, NULL, 0 }
};


static void Help(const char *name )
{
	printf("Usage: %s [OPTIONS]\n", name);
	puts(	"  --unix-server        server unix socket path\n"
			"  --unix-client        client unix socket path\n"
			"  --help, -h           this help message\n"
		);
	return;
}


/* Check CmdLine structure, after assigning to it.
 * Return 0 on success, -1 on error.
 * And messages will be displayed on error.
 */
static int CmdLineCheck(const struct CmdLine *pxCmdLine)
{
	int ret = 0;

	if ( pxCmdLine->pcUnixSrv == NULL )
	{
		ret = -1;
		fprintf(stderr, "Unix socket server path not specified\n");
	}

	if( pxCmdLine->pcUnixCli == NULL )
	{
		ret = -1;
		fprintf(stderr, "Unix socket client path not specified\n");
	}

	return ret;
}


void CmdLineParse(struct CmdLine *pxCmdLine, int argc, char *argv[])
{
	int c;

	/* Clear CmdLine structure */
	pxCmdLine->pcUnixSrv = NULL;
	pxCmdLine->pcUnixCli = NULL;

	/* Iterate command line options */
	while( (c = getopt_long(argc, argv, "h", options, NULL)) != -1 )
	{
		switch (c)
		{
			case 'h':
				Help(argv[0]);
				exit( EXIT_SUCCESS );
				break;

			case UNIX_SERVER:
				pxCmdLine->pcUnixSrv = optarg;
				break;

			case UNIX_CLIENT:
				pxCmdLine->pcUnixCli = optarg;
				break;

			case '?':
				exit( EXIT_FAILURE );
				break;

			default:
				fprintf(stderr, "Impossible to be here\n");
				break;
		}	/* end switch */
	}	/* end while */

	/* Check if there are non-option argument(s) */
	if (optind < argc)
	{
		fprintf(stderr, "non-option elements: ");
		while (optind < argc)
			fprintf(stderr, "%s ", argv[ optind++ ]);
		fprintf(stderr, "\n");
		exit( EXIT_FAILURE );
	}

	/* Check CmdLine structure */
	if ( CmdLineCheck( pxCmdLine ) < 0 )
		exit( EXIT_FAILURE );
}
