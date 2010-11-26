/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <asm/time.h>

#include <rt-galaxy-board.h>
#include <rt-galaxy-soc.h>
#include <rt-galaxy-irq.h>
#include <rt-galaxy-io.h>

/*
 * rt-galaxy uart
 */

static unsigned int rtgalaxy_serial_in(struct uart_port *p, int offset)
{
	offset <<= p->regshift;
	return readl(p->membase + offset) & 0xff;
}

static void rtgalaxy_serial_out(struct uart_port *p, int offset, int value)
{
	offset <<= p->regshift;
	writel(value & 0xff, p->membase + offset);
}

static struct plat_serial8250_port rtgalaxy_serial_data[] = {
	[0] = {},
	[1] = {},
	{}
};

static struct platform_device rtgalaxy_serial8250_device = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = rtgalaxy_serial_data,
		},
};

static void __init rtgalaxy_register_uart(void)
{
	int n = 0;

	if (rtgalaxy_board_info->has_uart0) {
		rtgalaxy_serial_data[n].iobase = RTGALAXY_UART0_BASE;
		rtgalaxy_serial_data[n].membase =
		    (unsigned char __iomem *)KSEG1ADDR(RTGALAXY_REG_BASE +
						       RTGALAXY_UART0_BASE);
		rtgalaxy_serial_data[n].mapbase =
		    RTGALAXY_REG_BASE + RTGALAXY_UART0_BASE;
		rtgalaxy_serial_data[n].irq = RTGALAXY_IRQ_UART0;
		rtgalaxy_serial_data[1].uartclk = rtgalaxy_board_info->ext_freq;
		rtgalaxy_serial_data[n].iotype = UPIO_MEM;
		rtgalaxy_serial_data[n].flags = UPF_BOOT_AUTOCONF;
		rtgalaxy_serial_data[n].regshift = 2;
		rtgalaxy_serial_data[n].serial_in = &rtgalaxy_serial_in;
		rtgalaxy_serial_data[n].serial_out = &rtgalaxy_serial_out;
		n++;
	}

	if (rtgalaxy_board_info->has_uart1) {
		rtgalaxy_serial_data[n].iobase = RTGALAXY_UART1_BASE;
		rtgalaxy_serial_data[n].membase =
		    (unsigned char __iomem *)KSEG1ADDR(RTGALAXY_REG_BASE +
						       RTGALAXY_UART1_BASE);
		rtgalaxy_serial_data[n].mapbase =
		    RTGALAXY_REG_BASE + RTGALAXY_UART1_BASE;
		rtgalaxy_serial_data[n].irq = RTGALAXY_IRQ_UART1;
		rtgalaxy_serial_data[0].uartclk = rtgalaxy_board_info->ext_freq;
		rtgalaxy_serial_data[n].iotype = UPIO_MEM;
		rtgalaxy_serial_data[n].flags = UPF_BOOT_AUTOCONF;
		rtgalaxy_serial_data[n].regshift = 2;
		rtgalaxy_serial_data[n].serial_in = &rtgalaxy_serial_in;
		rtgalaxy_serial_data[n].serial_out = &rtgalaxy_serial_out;
		n++;
	}

	if (n)
		platform_device_register(&rtgalaxy_serial8250_device);
}

/*
 * platform and device init
 */

void __init platform_init(void)
{
}

static int __init rtgalaxy_devinit(void)
{
	rtgalaxy_register_uart();
	return 0;
}

device_initcall(rtgalaxy_devinit);
