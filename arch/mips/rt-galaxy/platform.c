/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/dma-mapping.h>
#include <asm/time.h>

#include <rt-galaxy-board.h>
#include <rt-galaxy-soc.h>
#include <rt-galaxy-irq.h>
#include <rt-galaxy-io.h>

/*
 * rt-galaxy uart
 */

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

	if (rtgalaxy_info.board->has_uart0) {
		rtgalaxy_serial_data[n].iobase = RTGALAXY_UART0_BASE;
		rtgalaxy_serial_data[n].membase =
						(unsigned char __iomem *)KSEG1ADDR(RTGALAXY_REG_BASE +
																						RTGALAXY_UART0_BASE);
		rtgalaxy_serial_data[n].mapbase = KSEG1ADDR(RTGALAXY_REG_BASE + RTGALAXY_UART0_BASE);
		rtgalaxy_serial_data[n].irq = RTGALAXY_IRQ_UART0;
		rtgalaxy_serial_data[n].uartclk = rtgalaxy_info.board->ext_freq;
		rtgalaxy_serial_data[n].iotype = UPIO_MEM32;
		rtgalaxy_serial_data[n].flags = UPF_BOOT_AUTOCONF;
		rtgalaxy_serial_data[n].regshift = 2;
		n++;
	}

	if (rtgalaxy_info.board->has_uart1) {
		rtgalaxy_serial_data[n].iobase = RTGALAXY_UART1_BASE;
		rtgalaxy_serial_data[n].membase =
		    (unsigned char __iomem *)KSEG1ADDR(RTGALAXY_REG_BASE +
																				RTGALAXY_UART1_BASE);
		rtgalaxy_serial_data[n].mapbase = KSEG1ADDR(RTGALAXY_REG_BASE +	RTGALAXY_UART1_BASE);
		rtgalaxy_serial_data[n].irq = RTGALAXY_IRQ_UART1;
		rtgalaxy_serial_data[n].uartclk = rtgalaxy_info.board->ext_freq;
		rtgalaxy_serial_data[n].iotype = UPIO_MEM32;
		rtgalaxy_serial_data[n].flags = UPF_BOOT_AUTOCONF;
		rtgalaxy_serial_data[n].regshift = 2;
		n++;
	}

	if (n)
		platform_device_register(&rtgalaxy_serial8250_device);
}

/*
 * rt-galaxy eth
 */

static struct resource rtgalaxy_eth_resource[] = {
	{
					.name = "rtgalaxy-eth",
					.start	= RTGALAXY_REG_BASE + RTGALAXY_ETH_BASE_OFFSET,
					.end	= RTGALAXY_REG_BASE + RTGALAXY_ETH_BASE_OFFSET + 0x1000 - 1,
					.flags	= IORESOURCE_MEM,
	},
	{
					.name = "rtgalaxy-eth",
					.start	= RTGALAXY_IRQ_ETH,
					.end	= RTGALAXY_IRQ_ETH,
					.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device rtgalaxy_eth_device = {
	.id 		= 0,
	.name		= "rtgalaxy-eth",
	.dev = {
		.platform_data	= NULL,
	},
	.resource	= rtgalaxy_eth_resource,
	.num_resources	= ARRAY_SIZE(rtgalaxy_eth_resource),
};

static void __init rtgalaxy_register_eth(void)
{
	if (!rtgalaxy_info.board->has_eth0)
		return;

	rtgalaxy_eth_device.dev.platform_data = rtgalaxy_info.ethaddr;
	platform_device_register(&rtgalaxy_eth_device);
}

/*
 * rt-galaxy ohci and ehci
 */

static struct resource rtgalaxy_ohci_resource[] = {
				[0] = {
								.start	= RTGALAXY_REG_BASE + RTGALAXY_OHCI_BASE_OFFSET,
								.end		= RTGALAXY_REG_BASE + RTGALAXY_OHCI_BASE_OFFSET + 0x100 - 1,
								.flags	= IORESOURCE_MEM,
				},
				[1] = {
								.start	= RTGALAXY_IRQ_USB,
								.end		= RTGALAXY_IRQ_USB,
								.flags	= IORESOURCE_IRQ,
				},
};

static u64 rtgalaxy_ohci_dmamask = DMA_BIT_MASK(32);
static struct platform_device rtgalaxy_ohci_device = {
				.name						= "rtgalaxy-ohci",
				.id							= -1,
				.resource				= rtgalaxy_ohci_resource,
				.num_resources	= ARRAY_SIZE(rtgalaxy_ohci_resource),
				.dev = {
								.dma_mask						= &rtgalaxy_ohci_dmamask,
								.coherent_dma_mask	= DMA_BIT_MASK(32),
				},
};

static struct resource rtgalaxy_ehci_resource[] = {
				[0] = {
								.start	= RTGALAXY_REG_BASE + RTGALAXY_EHCI_BASE_OFFSET,
								.end		= RTGALAXY_REG_BASE + RTGALAXY_EHCI_BASE_OFFSET + 0x100 - 1,
								.flags	= IORESOURCE_MEM,
				},
				[1] = {
								.start	= RTGALAXY_IRQ_USB,
								.end		= RTGALAXY_IRQ_USB,
								.flags	= IORESOURCE_IRQ,
				},
};

static u64 rtgalaxy_ehci_dmamask = DMA_BIT_MASK(32);
static struct platform_device rtgalaxy_ehci_device = {
				.name						= "rtgalaxy-ehci",
				.id							= -1,
				.resource				= rtgalaxy_ehci_resource,
				.num_resources	= ARRAY_SIZE(rtgalaxy_ehci_resource),
				.dev = {
								.dma_mask						= &rtgalaxy_ehci_dmamask,
								.coherent_dma_mask	= DMA_BIT_MASK(32),
				},
};

static void __init rtgalaxy_register_usb(void)
{
				if (rtgalaxy_info.board->has_ohci0)
								platform_device_register(&rtgalaxy_ohci_device);

				if (rtgalaxy_info.board->has_ehci0)
								platform_device_register(&rtgalaxy_ehci_device);
}

/*
 * platform and device init
 */

void __init platform_init(void)
{
}

static int __init rtgalaxy_devinit(void)
{
				set_io_port_base(KSEG1ADDR(RTGALAXY_REG_BASE));
				rtgalaxy_register_uart();
				rtgalaxy_register_eth();
				rtgalaxy_register_usb();
				return 0;
}

device_initcall(rtgalaxy_devinit);
