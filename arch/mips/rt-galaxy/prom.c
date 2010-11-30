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
		strcpy(rtgalaxy_info.bootrev, envp);
		sscanf(envp, "%hx.%hx.%hx", &v0, &v1, &v2);

		rtgalaxy_info.company_id = v0;
		rtgalaxy_info.board_id = (v0 << 16) | v1;

		/* old bootrev format : aa.bb.ccc */
		/* new bootrev format : aaaa.bbbb.cccc */
		if (envp[2] == '.')
			rtgalaxy_info.cpu_id = (v1 & 0xf0) >> 4;
		else
			rtgalaxy_info.cpu_id = (v1 & 0xff00) >> 8;
#ifdef DEBUG
		printk
		    ("bootrev = '%s' => company_id = %04x, cpu_id = %02x, board_id = %08x\n",
		     rtgalaxy_info.bootrev,
		     rtgalaxy_info.company_id,
		     rtgalaxy_info.cpu_id, rtgalaxy_info.board_id);
#endif
	}
}

void __init rtgalaxy_env_get_modetty(int tty)
{
	char console_string[40];
	char param[16];
	int baud = 0;
	char parity = '\0', bits = '\0', flow = '\0';
	char *s;

	sprintf(param, "console=ttyS%d", tty);
	if ((strstr(prom_getcmdline(), param)) == NULL) {
		sprintf(param, "modetty%d", tty);
		s = prom_getenv(param);
		if (s) {
			while (*s >= '0' && *s <= '9')
				baud = baud * 10 + *s++ - '0';
			if (*s == ',')
				s++;
			if (*s)
				parity = *s++;
			if (*s == ',')
				s++;
			if (*s)
				bits = *s++;
			if (*s == ',')
				s++;
			if (*s == 'h')
				flow = 'r';
		}
		if (baud == 0)
			baud = 115200;
		if (parity != 'n' && parity != 'o' && parity != 'e')
			parity = 'n';
		if (bits != '7' && bits != '8')
			bits = '8';
		if (flow == '\0')
			flow = 'r';
		sprintf(console_string, " console=ttyS%d,%d%c%c%c", tty, baud,
			parity, bits, flow);
		strcat(prom_getcmdline(), console_string);
		pr_info("Config serial console:%s\n", console_string);
	}
}

static inline unsigned char str2hexnum(unsigned char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return 0;		/* foo */
}

static inline void str2eaddr(unsigned char *ea, unsigned char *str)
{
	int i;

	for (i = 0; i < 6; i++) {
		unsigned char num;

		if ((*str == '.') || (*str == ':'))
			str++;
		num = str2hexnum(*str++) << 4;
		num |= (str2hexnum(*str++));
		ea[i] = num;
	}
}

void __init rtgalaxy_env_get_ethaddr(void)
{
	char *s;
	int i;

	s = prom_getenv("ethaddr");
	if (!s) {
		printk("ethaddr not set in boot prom, using default\n");
		str2eaddr(rtgalaxy_info.ethaddr, "00:01:ca:fe:ba:be");
		return;
	}
	str2eaddr(rtgalaxy_info.ethaddr, s);

	pr_info("Config ether addr: ");
	for (i = 0; i < 5; i++)
		printk("%02x:", (unsigned char)rtgalaxy_info.ethaddr[i]);
	printk("%02x\n", (unsigned char)rtgalaxy_info.ethaddr[i]);

	return;
}

void __init prom_init(void)
{
	prom_argc = fw_arg0;
	_prom_argv = (int *)fw_arg1;
	_prom_envp = (int *)fw_arg2;
	_prom_memsize = (unsigned long)fw_arg3;

	board_nmi_handler_setup = mips_nmi_setup;
	board_ejtag_handler_setup = mips_ejtag_setup;

	rtgalaxy_info.memory_size = _prom_memsize;

	rtgalaxy_env_get_bootrev();
	rtgalaxy_env_get_modetty(0);
	rtgalaxy_env_get_modetty(1);
	rtgalaxy_env_get_ethaddr();

	rtgalaxy_detect_soc();

	platform_init();
	prom_init_cmdline();
	prom_meminit();
}
