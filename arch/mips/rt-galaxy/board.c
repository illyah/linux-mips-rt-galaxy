/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/string.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <rt-galaxy-board.h>
#include <rt-galaxy-soc.h>
#include <rt-galaxy-io.h>

struct rtgalaxy_board_info rtgalaxy_info;

static struct rtgalaxy_board rtgalaxy_em7080_board = {
	.name = "em7080",
	.ext_freq = 27000000,
	.has_eth0 = 1,
	.has_pci = 0,
	.has_pccard = 0,
	.has_ohci0 = 1,
	.has_ehci0 = 1,
	.has_sata0 = 1,
	.has_sata1 = 1,
	.has_uart0 = 1,
	.has_uart1 = 1,
	.has_vfd = 0,
	.machine_restart = NULL,
	.machine_halt = NULL,
	.machine_poweroff = NULL,
};

/*
 * rt-galaxy reset and halt handlers
 */

static void rtgalaxy_common_machine_restart(char *cmd)
{
#ifdef CONFIG_RTGALAXY_WATCHDOG
	/*
	 * Use Watchdog to reset SoC
	 */
	kill_watchdog();
#else
	/*
	 * TODO: Find a way to reset the SoC
	 */
	outl(0x0, RTGALAXY_TIMR_TCWCR);
#endif
	while (1) ;
}

static void rtgalaxy_common_machine_halt(void)
{
	while (1) ;
}

/*
 * rt-galaxy soc
 */

static const char *rtgalaxy_get_soc_name(void)
{
	switch (rtgalaxy_info.chip_id) {
	case RTGALAXY_CHIPID_VENUS:
		return "Venus";
	case RTGALAXY_CHIPID_NEPTUNE:
		return "Neptune";
	case RTGALAXY_CHIPID_MARS:
		return "Mars";
	case RTGALAXY_CHIPID_JUPITER:
		return "Jupiter";
	}
	return "unknown";
}

void rtgalaxy_detect_soc(void)
{
	u32 id, rev;

	id = inl(RTGALAXY_SB2_CHIP_ID);
	rev = inl(RTGALAXY_SB2_CHIP_INFO);

	rtgalaxy_info.chip_id = id & 0xffff;
	rtgalaxy_info.chip_rev = (id >> 16) & 0xffff;

	printk("Detected rtd%04x rev %x SoC (%s)\n",
	       rtgalaxy_info.chip_id,
	       rtgalaxy_info.chip_rev, rtgalaxy_get_soc_name());
}

/*
 * rt-galaxy boards
 */

static enum rtgalaxy_board_type rtgalaxy_detect_board(void)
{
	return RTGALAXY_BOARD_EM7080;

	/*
	 * TODO: Detect different board types
	 */

	return RTGALAXY_BOARD_UNKNOWN;
}

void rtgalaxy_board_setup(void)
{
	rtgalaxy_info.board_type = rtgalaxy_detect_board();
	switch (rtgalaxy_info.board_type) {
	case RTGALAXY_BOARD_EM7080:
		rtgalaxy_info.board = &rtgalaxy_em7080_board;
		break;
	default:
		/* unknown rt-galaxy board */
		BUG();
	}

	if (rtgalaxy_info.board->machine_restart == NULL) {
		rtgalaxy_info.board->machine_restart =
		    rtgalaxy_common_machine_restart;
	}

	if (rtgalaxy_info.board->machine_halt == NULL) {
		rtgalaxy_info.board->machine_halt =
		    rtgalaxy_common_machine_halt;
	}

	if (rtgalaxy_info.board->machine_poweroff == NULL) {
		rtgalaxy_info.board->machine_poweroff =
		    rtgalaxy_common_machine_halt;
	}

	printk("Detected %s board (type=%d)\n", rtgalaxy_info.board->name,
	       rtgalaxy_info.board_type);
}
