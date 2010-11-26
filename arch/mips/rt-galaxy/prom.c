/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>

#include <asm/bootinfo.h>
#include <linux/io.h>
#include <asm/system.h>
#include <asm/cacheflush.h>
#include <asm/traps.h>

#include <asm/mips-boards/prom.h>
#include <asm/mips-boards/generic.h>

#include <rt-galaxy-board.h>
#include <rt-galaxy-soc.h>
#include <rt-galaxy-io.h>

extern void platform_init(void);
extern void platform_setup(void);
extern void rtgalaxy_detect_soc(void);

//#define DEBUG

static int prom_argc;
static int *_prom_argv, *_prom_envp;
unsigned long _prom_memsize;

/*
 * YAMON (32-bit PROM) pass arguments and environment as 32-bit pointer.
 * This macro take care of sign extension, if running in 64-bit mode.
 */
#define prom_envp(index) ((char *)(long)_prom_envp[(index)])

char *prom_getenv(char *envname)
{
	char *result = NULL;

	if (_prom_envp != NULL) {
		/*
		 * Return a pointer to the given environment variable.
		 * In 64-bit mode: we're using 64-bit pointers, but all pointers
		 * in the PROM structures are only 32-bit, so we need some
		 * workarounds, if we are running in 64-bit mode.
		 */
		int i, index = 0;

		i = strlen(envname);

		while (prom_envp(index)) {
			if (strncmp(envname, prom_envp(index), i) == 0) {
				result = prom_envp(index + 1);
				break;
			}
			index += 2;
		}
	}

	return result;
}

/*
 * YAMON (32-bit PROM) pass arguments and environment as 32-bit pointer.
 * This macro take care of sign extension.
 */
#define prom_argv(index) ((char *)(long)_prom_argv[(index)])

char *__init prom_getcmdline(void)
{
	return &(arcs_cmdline[0]);
}

void __init prom_init_cmdline(void)
{
	char *cp;
	int actr;

	actr = 1;		/* Always ignore argv[0] */

	cp = &(arcs_cmdline[0]);
	while (actr < prom_argc) {
		strcpy(cp, prom_argv(actr));
		cp += strlen(prom_argv(actr));
		*cp++ = ' ';
		actr++;
	}
	if (cp != &(arcs_cmdline[0])) {
		/* get rid of trailing space */
		--cp;
		*cp = '\0';
	}
}

/* TODO: Verify on linux-mips mailing list that the following two  */
/* functions are correct                                           */
/* TODO: Copy NMI and EJTAG exception vectors to memory from the   */
/* BootROM exception vectors. Flush their cache entries. test it.  */

static void __init mips_nmi_setup(void)
{
	void *base;
#if defined(CONFIG_CPU_MIPS32_R1)
	base = cpu_has_veic ?
	    (void *)(CAC_BASE + 0xa80) : (void *)(CAC_BASE + 0x380);
#elif defined(CONFIG_CPU_MIPS32_R2)
	base = (void *)0xbfc00000;
#else
#error NMI exception handler address not defined
#endif
}

static void __init mips_ejtag_setup(void)
{
	void *base;
#if defined(CONFIG_CPU_MIPS32_R1)
	base = cpu_has_veic ?
	    (void *)(CAC_BASE + 0xa00) : (void *)(CAC_BASE + 0x300);
#elif defined(CONFIG_CPU_MIPS32_R2)
	base = (void *)0xbfc00480;
#else
#error EJTAG exception handler address not defined
#endif
}

void __init rtgalaxy_env_get_bootrev(void)
{
	char *envp;
	unsigned short v0, v1, v2;

	envp = prom_getenv("bootrev");
	if (envp) {
		strcpy(rtgalaxy_board_info->bootrev, envp);
		sscanf(envp, "%hx.%hx.%hx", &v0, &v1, &v2);

		rtgalaxy_board_info->company_id = v0;
		rtgalaxy_board_info->board_id = (v0 << 16) | v1;

		/* old bootrev format : aa.bb.ccc */
		/* new bootrev format : aaaa.bbbb.cccc */
		if (envp[2] == '.')
			rtgalaxy_board_info->cpu_id = (v1 & 0xf0) >> 4;
		else
			rtgalaxy_board_info->cpu_id = (v1 & 0xff00) >> 8;
#ifdef DEBUG
		printk
		    ("bootrev = '%s' => company_id = %04x, cpu_id = %02x, board_id = %08x\n",
		     rtgalaxy_board_info->bootrev,
		     rtgalaxy_board_info->company_id,
		     rtgalaxy_board_info->cpu_id,
		     rtgalaxy_board_info->board_id);
#endif
	}
}

void __init prom_init(void)
{
	prom_argc = fw_arg0;
	_prom_argv = (int *)fw_arg1;
	_prom_envp = (int *)fw_arg2;
	_prom_memsize = (unsigned long)fw_arg3;

	board_nmi_handler_setup = mips_nmi_setup;
	board_ejtag_handler_setup = mips_ejtag_setup;

	rtgalaxy_board_info->memory_size = _prom_memsize;

	rtgalaxy_env_get_bootrev();

	set_io_port_base(KSEG1ADDR(RTGALAXY_REG_BASE));
	rtgalaxy_detect_soc();

	platform_init();
	prom_init_cmdline();
	prom_meminit();
}
