/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/serial_reg.h>
#include <rt-galaxy-io.h>

static void __init wait_xfered(void)
{
	unsigned int val;

	/* wait for any previous char to be transmitted */
	do {
		val = inl(RTGALAXY_UART_U0LSR);
		if ((val & (UART_LSR_TEMT | UART_LSR_THRE)) ==
		    (UART_LSR_TEMT | UART_LSR_THRE))
			break;
	} while (1);
}

void __init prom_putchar(char c)
{
	wait_xfered();
	outl(c, RTGALAXY_UART_U0RBR_THR_DLL);
}
