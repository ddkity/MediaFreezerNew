/*
 * Copyright 2014 Ayla Networks, Inc.  All rights reserved.
 */
#include <ayla/parse.h>

/*
 * Parse string into arguments.
 * Modifies string in buffer to change white space to NULs.
 * Handles simple quoting cases.
 * Doesn't use isspace().
 * Ignores leading spaces.
 */
int parse_argv(char **argv, int argv_len, char *buf)
{
	int argc = 0;
	char *cp = buf;
	char *dest = buf;
	char quote;
	char c;

	while (argc < argv_len && *cp != '\0') {
		/*
		 * Skip leading blanks and tabs.
		 */
		while ((c = *cp) != '\0' && (c == ' ' || c == '\t')) {
			cp++;
		}
		if (c == '\0') {
			break;
		}

		/*
		 * copy arg and skip until next blank, tab, or end of string.
		 * If quoted, look for matching quote.
		 */
		quote = 0;
		*argv++ = cp;
		argc++;
		for (dest = cp; (c = *cp) != '\0'; cp++) {
			if (quote) {
				if (c == quote) {
					quote = 0;
					continue;
				}
			} else {
				/*
				 * Look for start quote.
				 */
				if (c == '\'' || c == '\"') {
					quote = c;
					continue;
				}
				if (c == ' ' || c == '\t') {
					break;
				}
			}
			*dest++ = c;
		}
		*dest = '\0';
		if (c == '\0') {
			break;
		}
		cp++;
	}
	if (argc < argv_len) {
		*argv = 0;
	}
	return argc;
}

