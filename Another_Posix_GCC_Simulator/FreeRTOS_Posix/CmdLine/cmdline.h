#ifndef	_CMDLINE_H_
#define	_CMDLINE_H_


struct CmdLine {
	char *pcUnixSrv;	/* Unix socket server path */
	char *pcUnixCli;	/* Unix socket client path */
};


/**
 * Parse program command line, and fill CmdLine structure.
 * @param pxCmdLine Pointer to a CmdLine structure.
 * @param argc Program argument count.
 * @param argv Program argument vector.
 * @return None. This function will terminate the program on command line error or on help.
 */
void CmdLineParse(struct CmdLine *pxCmdLine, int argc, char *argv[]);

#endif
