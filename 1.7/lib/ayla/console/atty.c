/*
 * Copyright 2014 Ayla Networks, Inc.  All rights reserved.
 */
#include <string.h>
#include <stdlib.h>
#include <ayla/mcu_platform.h>
#include <ayla/atty.h>
#include <ayla/console.h>

#define	CR	'\r'
#define	LF	'\n'
#define BS	0x08
#define	TAB	0x09
#define	CTL_U	0x15
#define	CTL_C	0x03
#define DEL	0x7f
#define ESC	0x1B
#define UP	0x41
#define DOWN	0x42
#define RIGHT	0x43
#define LEFT	0x44

#define TTY_BUF_LEN	1024	/* small for now - not much type-ahead */
#define TTY_COOKED_LEN	1024

enum atty_in_state {
	TTY_NONE,
	TTY_ESC,
	TTY_LTBR
};

/*
 * State of the console TTY.
 */
struct atty_buf {
	volatile unsigned int in;		/* index of next empty byte in buffer */
	volatile unsigned int out;	/* index of first non-empty byte in buffer */
	int size;		/* buffer size */
	u8* buf;
};

/* --- each buffer length may be different */

struct atty {
	u16 intrs;		/* stats */
	u16 tx_full_err;	/* stats */
	void (*rx_raw)(u8);
	void (*cmd_handler)(char *);
	struct {
		struct atty_buf buf;
		u8 area[TTY_BUF_LEN];
	} raw;
	struct {
		struct atty_buf buf;
		u8 area[TTY_BUF_LEN];
	} out;
	struct {
		struct atty_buf buf;
		u8 area[TTY_COOKED_LEN];
	} cooked;
	enum atty_in_state atty_state;
};

static void atty_rx(struct atty *atty);

static struct atty atty_cons;

static void atty_buf_reset(struct atty_buf *bp)
{
	bp->in = 0;
	bp->out = 0;
}

static void atty_buf_init(struct atty_buf *bp, u8 *ap, size_t size)
{
	bp->size = size;
	bp->in = 0;
	bp->out = 0;
	bp->buf = ap;
}

#if 0
/*
 * Returns non-zero if buffer is full.
 */
static int atty_buf_full(struct atty_buf *bp)
{
	return ((bp->in == bp->size - 1) ? 0 : bp->in + 1) == bp->out;
}

/*
 * Returns non-zero if buffer is empty.
 */
static int atty_buf_empty(struct atty_buf *bp)
{
	return bp->in == bp->out;
}
#endif

/*
 * Put a character into a circular buffer.
 * Called with interrupts blocked.
 */
static int atty_buf_enq(struct atty_buf *bp, u8 data)
{
	unsigned int in;

	in = bp->in;
	bp->buf[in] = data;		/* use the wasted slot if full */
	if (++in >= bp->size) {
		in = 0;
	}
	if (in == bp->out) {
		return -1;
	}
	bp->in = in;
	return 0;
}

/*
 * Take a character off the end of the buffer (e.g., for backspace).
 * Called with interrupts blocked.
 */
static int atty_buf_deq_tail(struct atty_buf *bp)
{
	if (bp->in != bp->out) {
		bp->in = (bp->in ? bp->in : bp->size) - 1;
		return 0;
	}
	return -1;
}

/*
 * Take a character off the front of the buffer.
 * Caller must block interrupts.
 * Returns -1 if nothing in the buffer.
 */
static int atty_buf_deq(struct atty_buf *bp)
{
	u8 data;

	if (bp->in != bp->out) {
		data = bp->buf[bp->out];
		bp->out = (bp->out == bp->size - 1) ? 0 : (bp->out + 1);
		return (int)data;
	}
	return -1;
}

/*
 * Handle received character.
 * Called directly from interrupt handler.
 */
static void atty_rx_intr(u8 data)
{
	struct atty *atty = &atty_cons;

	atty->intrs++;
	if (atty_buf_enq(&atty->raw.buf, data) == 0) {
		atty_rx(atty);
	}
}

/*
 * Get next character to be transmitted.
 * Called from interrupt handler when transmit buffer is empty.
 * Returns -1 if nothing to be sent.
 */
static int atty_get_tx(void)
{
	struct atty *atty = &atty_cons;

	return atty_buf_deq(&atty->out.buf);
}

/*
 * Transmit a byte.
 * Called with interrupts disabled.
 * May enable interrupts to wait if output buffer is full.
 */
static void atty_tx_masked(u8 data)
{
	struct atty *atty = &atty_cons;
	
	while (atty_buf_enq(&atty->out.buf, data) < 0) {
		console_platform_poll();
	}
}

/*
 * Transmit a byte.
 * This may be binary data, so don't change LF to LF, CR.
 */
void atty_tx(u8 data)
{
	atty_tx_masked(data);
}

/*
 * Transmit a character.
 * Change LF to CR, LF.
 * Should expand tabs, too. (later)
 *
 * Return negative value on error.
 */
static int atty_putchar(char c)
{
	if (c == LF) {
		atty_tx(CR);
	}
	atty_tx(c);
	return (int)(unsigned char)c;
}

#if 0
/*
 * Transmit a string.
 * Does not add a newline like puts(3) does.
 * Could hang with interrupts enabled while output drains.
 */
static void atty_puts(const char *buf)
{
	while (*buf) {
		atty_putchar(*buf++);
	}
}
#endif

/*
 * Handle receive buffer.
 * Called with interrupts disabled.
 * Enables them but returns with them disabled.
 */
static void atty_rx(struct atty *atty)
{
	int data;
	u8 byte;

	for (;;) {
		data = atty_buf_deq(&atty->raw.buf);
		if (data < 0) {
			break;
		}
		byte = data;
		if (atty->rx_raw) {
			atty->rx_raw(byte);
			continue;
		}
		switch (byte) {
		case ESC:
			atty->atty_state = TTY_ESC;
			return;
		case '[':
			if (atty->atty_state == TTY_ESC) {
				atty->atty_state = TTY_LTBR;
				return;
			}
			goto put;
		case UP:
		case DOWN:
		case RIGHT:
			if (atty->atty_state == TTY_LTBR) {
				atty->atty_state = TTY_NONE;
				continue;
			}
			goto put;
		case LEFT:
			if (atty->atty_state == TTY_LTBR) {
				atty->atty_state = TTY_NONE;
				goto delchar;
			}
			goto put;
		case BS:
		case DEL:
delchar:		if (atty_buf_deq_tail(&atty->cooked.buf) == 0) {
				if (console_echo_off) {
					break;
				}
				atty_tx_masked(BS);
				atty_tx_masked(' ');
				atty_tx_masked(BS);
			}
			break;
		case CTL_C:
			/* XXX generate interrupt to upper layer */
		case CTL_U:
			atty_buf_reset(&atty->cooked.buf);
			/* XXX clear displayed line */
			if (!console_echo_off) {
				atty_tx_masked(CR);
			}
			atty->atty_state = TTY_NONE;
			break;
		case CR:
		case LF:
			if (!console_echo_off) {
				atty_tx_masked(CR);
				atty_tx_masked(LF);
			}
			/* read done - deliver cooked buf */
			atty_buf_enq(&atty->cooked.buf, '\0');
			atty_buf_reset(&atty->cooked.buf);
			if (atty->cmd_handler) {
				atty->cmd_handler((char *)atty->cooked.buf.buf);
			}
			atty->atty_state = TTY_NONE;
			break;
		case TAB:
			byte = ' ';		/* XXX for now */
			/* fall-through */
		default:
			/*
			 * Ignore other control characters?
			 */
put:
			atty_buf_enq(&atty->cooked.buf, byte);
			if (!console_echo_off) {
				atty_tx_masked(byte);
			}
			atty->atty_state = TTY_NONE;
			break;
		}
	}
}

/*
 * Set or clear raw mode handler.
 */
void atty_set_raw(void (*rx_handler)(u8))
{
	struct atty *atty = &atty_cons;

	atty->rx_raw = rx_handler;
}

/*
 * Initialize the atty + console
 */
void atty_init(void (*cmd_handler)(char *))
{
	struct atty *atty = &atty_cons;

	atty_buf_init(&atty->raw.buf, &atty->raw.area[0], sizeof(atty->raw.area));
	atty_buf_init(&atty->out.buf, &atty->out.area[0], sizeof(atty->out.area));
	atty_buf_init(&atty->cooked.buf, &atty->cooked.area[0], sizeof(atty->cooked.area));
	putchar_init(atty_putchar);
	atty->cmd_handler = cmd_handler;
	console_init(atty_get_tx, atty_rx_intr);
}
