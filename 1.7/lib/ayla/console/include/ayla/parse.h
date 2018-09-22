/*
 * Copyright 2014 Ayla Networks, Inc.  All rights reserved.
 */
#ifndef __AYLA_PARSE_H__
#define __AYLA_PARSE_H__

#include <ayla/utypes.h>
#include <stddef.h>

int parse_argv(char **argv, int argv_len, char *buf);
int parse_hex(void *buf, size_t len, const char *hex, size_t hex_len);

#endif /* __AYLA_PARSE_H__ */
