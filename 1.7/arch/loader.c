/*
 * Copyright 2011-2012 Ayla Networks, Inc.  All rights reserved.
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

#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
#include <ayla/mcu_platform.h>
#elif defined(MCU_NXP_KINETIS_CM0P)
#include "common.h"
#include "ftfa.h"
#ifdef CMSIS
#include "start.h"
#endif
#endif
#include <flash_layout.h>
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
#include "flash_app.h"
#endif

#define VERSION "1.0"


static int img_mgmt_mark_progress(int off);

#ifdef AYLA_KEIL
extern void bb_enter_prog(void *addr);
#endif

static void boot(void)
{
#ifdef AYLA_KEIL
	bb_enter_prog((void *)MCU_IMG_ACTIVE);
#elif defined(AYLA_GCC)
	u32 addr = *((u32 *)MCU_IMG_ACTIVE);

	/*
	 * Set stack pointer.
	 */
	__asm__( "mov sp, %0"
	    :
	    : "r" (addr));

	/*
	 * Jump to newly loaded image. Offset 4 contains the address of the
	 * reset handler from the vector table.
	 */
	void (**func)(void) = (void (**)(void))(MCU_IMG_ACTIVE + 4);
	(*func)();
#else
#error loader not supported by this compiler
#endif
}

static void img_mgmt_set_boot_state(enum mcu_boot_state mbs)
{
	int off;

#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	FLASH_Unlock();
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif
	switch (mbs) {
	case MCU_BOOT_TEST:
		off = MCU_PBAR_OFF_TEST;
		break;
	case MCU_BOOT_FALLBACK_START:
		off = MCU_PBAR_OFF_FALLBACK_START;
		break;
	case MCU_BOOT_FALLBACK:
		off = MCU_PBAR_OFF_FALLBACK;
		break;
	default:
		/* xXXXx */
		off = MCU_PBAR_OFF_FALLBACK;;
		break;
	}
	img_mgmt_mark_progress(off);
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	FLASH_Lock();
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif
}

static enum mcu_boot_state img_mgmt_get_boot_state(void)
{
	if (MCU_PBAR_ISSET(MCU_PBAR_OFF_FALLBACK)) {
		return MCU_BOOT_FALLBACK;
	}
	if (MCU_PBAR_ISSET(MCU_PBAR_OFF_FALLBACK_START)) {
		return MCU_BOOT_FALLBACK_START;
	}
	if (MCU_PBAR_ISSET(MCU_PBAR_OFF_TEST)) {
		return MCU_BOOT_TEST;
	}
	if (MCU_PBAR_ISSET(MCU_PBAR_OFF_INACTIVE)) {
		return MCU_BOOT_INACTIVE;
	}
	return MCU_BOOT_OK;
}

static int img_mgmt_progress(void)
{
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	u16 *ptr;
	int i;

	ptr = (u16 *)MCU_IMG_PROG_BAR;
	for (i = 0; i <= MCU_PBAR_OFF_MAX; i++) {
		if (ptr[i] == 0xffff) {
			break;
		}
	}
	return i;
#elif defined(MCU_NXP_KINETIS_CM0P)
	u32 *ptr;
	int i;

	ptr = (u32 *)MCU_IMG_PROG_BAR;
	for (i = 0; i <= MCU_PBAR_OFF_MAX; i++) {
		if (ptr[i] == 0xffffffff) {
			break;
		}
	}
	return i;
#endif
}

static int img_mgmt_mark_progress(int off)
{
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	int rc;

	rc = FLASH_ProgramHalfWord(MCU_IMG_PROG_BAR + off * sizeof(u16), 0);
#elif defined(MCU_NXP_KINETIS_CM0P)
	int rc = 0;
	flash_app_memcpy_32bit((void *)(MCU_IMG_PROG_BAR + off * sizeof(u32)),	&rc , 4);
#endif
	return rc;
}

static int img_copy_block(u32 src, u32 dst)
{
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	int i, rc;
	uint16_t val;

	for (i = 0; i < MCU_IMG_SCRATCH_SZ / sizeof(u16); i++) {
		val = ((u16 *)src)[i];
		rc = FLASH_ProgramHalfWord(dst + i * sizeof(u16), val);
	}
#elif defined(MCU_NXP_KINETIS_CM0P)
	int rc = 0;
	flash_app_memcpy_32bit((void *)dst,  (void *)src , MCU_IMG_SCRATCH_SZ);
#endif
	return rc;
}

static void img_mgmt_erase_block(u32 start)
{
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	int i;

	for (i = 0; i < MCU_IMG_SCRATCH_SZ / MCU_IMG_BLK_SZ; i++) {
		FLASH_ErasePage(start + i * MCU_IMG_BLK_SZ);		
	}
#elif defined(MCU_NXP_KINETIS_CM0P)
	flash_app_erase((void *)start, MCU_IMG_SCRATCH_SZ);
#endif
}

static void img_mgmt_swap(enum mcu_boot_state state)
{
	int i, j, step;
	u32 src, dst, scratch;

	/*
	 * Swap is done 4 pages at a time.
	 *
	 * - erase scratch area
	 * - copy from source to scratch
	 * - mark progress
	 * - erase source
	 * - copy from dest to source
	 * - mark progress
	 * - erase dest
	 * - copy from scratch to dest
	 * - mark progress
	 *
	 * Copy progress is stored as an array of 16-bit values.
	 * 
	 */
	i = img_mgmt_progress();
	if (state == MCU_BOOT_INACTIVE) {
		j = i - 1;
	} else if (state == MCU_BOOT_FALLBACK_START) {
		j = i - (MCU_PBAR_OFF_FALLBACK_START + 1);
	} else {
		/* xXXXx */
		return;
	}
	step = j % 3;
	src = MCU_IMG_INACTIVE + (j / 3) * MCU_IMG_BLK_SZ;
	dst = MCU_IMG_ACTIVE + (j / 3) * MCU_IMG_BLK_SZ;
	scratch = MCU_IMG_SCRATCH;

	/*
	 * Copying. If not finished with a page, jump to middle of the loop.
	 */
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	FLASH_Unlock();
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif
	if (step == 0) {
		goto step0;
	} else if (step == 1) {
		goto step1;
	} else if (step == 2) {
		goto step2;
	}

	while (src < MCU_IMG_INACTIVE + MCU_IMG_MAX_SZ) {
	step0:
		img_mgmt_erase_block(scratch);
		img_copy_block(src, scratch);
		img_mgmt_mark_progress(i++);
	step1:
		img_mgmt_erase_block(src);
		img_copy_block(dst, src);
		img_mgmt_mark_progress(i++);
	step2:
		img_mgmt_erase_block(dst);
		img_copy_block(scratch, dst);
		img_mgmt_mark_progress(i++);

		src += MCU_IMG_SCRATCH_SZ;
		dst += MCU_IMG_SCRATCH_SZ;
	}
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
	FLASH_Lock();
#elif defined(MCU_NXP_KINETIS_CM0P)
#endif
}

int main(int argc, char **argv)
{
	enum mcu_boot_state state;

#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
#ifdef CMSIS			/* If we are conforming to CMSIS, we need to call start here */
	start();
#endif
	flash_app_init( 0x40020000u, 1024);	/* Initial flash app API */
#endif
	state = img_mgmt_get_boot_state();
	switch (state) {
	case MCU_BOOT_OK:
	case MCU_BOOT_FALLBACK:
		break;
	case MCU_BOOT_INACTIVE:
		/*
		 * Boot to image in inactive slot.
		 */
		img_mgmt_swap(state);
		img_mgmt_set_boot_state(MCU_BOOT_TEST);
		break;
	case MCU_BOOT_TEST:
		/*
		 * Boot failed! Swap the old image back.
		 */
		img_mgmt_set_boot_state(MCU_BOOT_FALLBACK_START);
		state = MCU_BOOT_FALLBACK_START;
		/* fall-through */
	case MCU_BOOT_FALLBACK_START:
		/*
		 * Boot to image in inactive slot.
		 */
		img_mgmt_swap(state);
		img_mgmt_set_boot_state(MCU_BOOT_FALLBACK);
		break;
	}
#if   defined(MCU_ST_STM32_CM4) || defined(MCU_ST_STM32_CM0)
#elif defined(MCU_NXP_KINETIS_CM0P)
	/*Relocate vector table*/
	SCB_VTOR = MCU_IMG_ACTIVE;
#endif
	boot();
	/* never reached */
	return 0;
}
