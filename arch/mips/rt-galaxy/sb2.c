/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/io.h>

#include <rt-galaxy-io.h>
#include <rt-galaxy-irq.h>

static irqreturn_t rtgalaxy_sb2_irq_handler(int irq, void *dev_id)
{
	unsigned int addr;

	if (!(inl(RTGALAXY_SB2_INV_INTSTAT) & 0x2))
		return IRQ_NONE;

	/*
	 * The seems to be a problem on Mars with prefetching
	 * specific memory regions. This patch should circumvent
	 * this bug.
	 */
	addr = inl(RTGALAXY_SB2_INV_ADDR);
	if (addr > 0x8001000 && ((addr & 0xfffff000) != 0x1800c000)) {
		printk("Access to invalid hw address 0x%x\n", addr);
	}
	outl(0xE, RTGALAXY_SB2_INV_INTSTAT);

	return IRQ_HANDLED;
}

static struct irqaction rtgalaxy_sb2_irq_action = {
	.handler = rtgalaxy_sb2_irq_handler,
	.flags = IRQF_SHARED,
	.name = "rt-galaxy-sb2",
};

void __init rtgalaxy_sb2_setup(void)
{
	outl(0x3, RTGALAXY_SB2_INV_INTEN);
	outl(0xe, RTGALAXY_SB2_INV_INTSTAT);
	setup_irq(RTGALAXY_IRQ_SB2, &rtgalaxy_sb2_irq_action);
}
