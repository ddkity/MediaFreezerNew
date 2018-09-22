/*
 * Copyright 2014 Ayla Networks, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Ayla Networks, Inc.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ayla/mcu_platform.h>
#include <stm32.h>
#include <ayla/console.h>
#include <ayla/atty.h>
#include <ayla/cmd.h>
#include <ayla/ayla_proto_mcu.h>
#include <ayla/props.h>
#include <mcu_io.h>
#include <ayla/byte_order.h>
#include <ayla/conf_token.h>
#include <ayla/conf_access.h>
#include <ayla/serial_msg.h>
#include <ayla/parse.h>

#ifdef DEMO_UART
#define CONSOLE_HOST "uart"
#else
#include <ayla/spi.h>
#define CONSOLE_HOST "spi"
#endif

static void console_help_cli(int argc, char **argv);

/*
 * See DEV-1649 regarding VERSION
 */
#ifdef VERSION
#define CONSOLE_VER VERSION
#else
#define CONSOLE_VER "1.4"
#endif
/*
 * Max console command size for printout
 */
#define STR2(x) #x
#define STR(x) STR2(x)
#define MAXC 15

#define CONSOLE_PROMPT "mcu-> "

struct console_state {
	u8	inited;
	int	(*get_tx)(void);
	void	(*rx_intr)(u8);
};

struct console_rx_entry {
	int seq_no;
	size_t len;
	char *rx_buf;
};

static struct console_state console_state;
u8 console_echo_off;
enum source console_src;
static char console_rx_buf[MAX_CONSOLE_RX_ENTRIES][400];
static struct console_rx_entry console[MAX_CONSOLE_RX_ENTRIES];
static int console_rx_done = 1;
static int console_rx_bdry = MAX_CONSOLE_RX_ENTRIES - 1;


void console_recv(u8 dr)
{
	if (!console_state.inited) {
		return;
	}
	console_state.rx_intr(dr);
}

int console_tx(void)
{
	if (!console_state.inited) {
		return -1;
	}
	return console_state.get_tx();
}

/*
 * Setup the console state
 */
void console_init(int (*get_tx)(void), void (*rx_intr)(u8))
{
	struct console_state *console = &console_state;

	console_platform_init();
	memset(console, 0, sizeof(console_state));
	console->get_tx = get_tx;
	console_state.rx_intr = rx_intr;
	console_state.inited = 1;
}

static void console_unknown(int argc, char **argv)
{
	printf("unrecognized command \"%s\"\n", argv[0]);
}

static void console_echo_cli(int argc, char **argv)
{
	if (argc != 2) {
		print("usage: [off|on]");
		return;
	}
	if (!strcmp(argv[1], "off")) {
		console_echo_off = 1;
	} else {
		console_echo_off = 0;
	}
}


static void console_reset(int argc, char **argv)
{
	stm32_reset_module();
}

static void console_ver_cli(int argc, char **argv)
{
	printf("%s %s\n", CONSOLE_VER, CONSOLE_HOST);
}

/*
 * Please keep this table in alphabetical order.
 */
static const struct cmd_info console_cmds[] = {
	CMD_INIT("echo", console_echo_cli, "Turn echo on/off: echo [on|off]"),
	CMD_INIT("help", console_help_cli, "Show commands"),
	CMD_INIT("reset", console_reset, "Reset host MCU and the module"),
	CMD_INIT("ver", console_ver_cli, "Show version"),
	CMD_END_DEFAULT(console_unknown)
};

static void console_help_cli(int argc, char **argv)
{
	const struct cmd_info *cmd;
	char name[MAXC + 1];

	for (cmd = console_cmds; cmd->name; cmd++) {
		/*
		 * MAXC is set to 15
		 * printf and snprintf specifier %-15s not supported, using sprintf
		 */
		sprintf(name, "%-"STR(MAXC)"s", cmd->name);
		printf("%s - %s\n", name, cmd->help);
	}
}

/*
 * Callback function to access received raw packets from module;
 * ignores acks in UART mode.
 */
int console_rx_buf_cb(u8 *data_ptr, size_t recv_len)
{
	static int cnt;
	static int i;
	char *buf;
	int bdry = console_rx_bdry;
	int end = MAX_CONSOLE_RX_ENTRIES - 1;

	if (console_rx_done) {
		cnt = 1;
		i = 0;
		console_rx_done = 0;
	}

	if (i >= MAX_CONSOLE_RX_ENTRIES) {
		i = bdry;
	}

#ifdef DEMO_UART
	if ((data_ptr[1] & 0xff) == 0x15) {
		return i;
	}
#endif

	if (console[i].len == 0) {
		memcpy(console_rx_buf[i], data_ptr, recv_len);
		console[i].seq_no = cnt;
		console[i].len = recv_len;
		console[i].rx_buf = console_rx_buf[i];
	} else if (i >= bdry && bdry < end) {
		buf = console[bdry].rx_buf;
		memcpy(buf, data_ptr, recv_len);
		if (end - bdry) {
			memmove(&console[bdry], &console[bdry + 1],
			    (end - bdry) * sizeof(struct console_rx_entry));
		}
		console[end].seq_no = cnt;
		console[end].len = recv_len;
		console[end].rx_buf = buf;
	}
	cnt++;
	i++;
	return i;
}

/*
 * Console command handler.
 */
void console_cli(char *cmd)
{
	const char *prompt = CONSOLE_PROMPT;

	if (cmd_handle(console_cmds, cmd)) {
		print("warning: too many args, cmd ignored");
	}
	if (!console_echo_off) {
		puts(prompt);
	}
}
