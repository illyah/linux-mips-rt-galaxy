/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#ifndef _RTGALAXY_BOARD_H_
#define _RTGALAXY_BOARD_H_

#include <linux/types.h>

enum rtgalaxy_board_type {
	RTGALAXY_BOARD_UNKNOWN = 0,
	RTGALAXY_BOARD_EM7080 = 1,
	RTGALAXY_BOARD_XTREAMER = 2,
};

struct rtgalaxy_board {
	char name[16];
	u32 ext_freq;
	unsigned int has_eth0:1;
	unsigned int has_pci:1;
	unsigned int has_pccard:1;
	unsigned int has_ohci0:1;
	unsigned int has_ehci0:1;
	unsigned int has_sata0:1;
	unsigned int has_sata1:1;
	unsigned int has_uart0:1;
	unsigned int has_uart1:1;
	unsigned int has_vfd:1;

	void (*machine_restart) (char *);
	void (*machine_halt) (void);
	void (*machine_poweroff) (void);
};

struct rtgalaxy_board_info {
	enum rtgalaxy_board_type board_type;
	struct rtgalaxy_board *board;

	/*
	 * Autodetected / PROM values
	 */

	phys_t memory_size;
	char bootrev[16];
	u16 company_id;
	u32 board_id;
	u8 cpu_id;
	u8 chip_rev;
	u16 chip_id;
};

extern struct rtgalaxy_board_info rtgalaxy_info;

#endif
