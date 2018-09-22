/*
 * Copyright 2011 Ayla Networks, Inc.  All rights reserved.
 */
#include <string.h>
#include <ayla/cmd.h>
#include <ayla/parse.h>

int cmd_handle(const struct cmd_info *cmds, char *buf)
{
	char *argv[CMD_ARGV_LIMIT];
	int argc;

	argc = parse_argv(argv, CMD_ARGV_LIMIT, buf);
	if (argc >= CMD_ARGV_LIMIT - 1) {
		return -1;
	}
	return cmd_handle_argv(cmds, argc, argv);
}

int cmd_handle_argv(const struct cmd_info *cmds, int argc, char **argv)
{
	const struct cmd_info *cmd;

	if (argc > 0) {
		for (cmd = cmds; cmd->name; cmd++) {
			if (!strcmp(cmd->name, argv[0])) {
				break;
			}
		}
		cmd->handler(argc, argv);
	}
	return 0;
}
