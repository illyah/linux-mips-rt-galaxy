#include <linux/init.h>
#include <asm/io.h>
#include <asm/gic.h>
#include <asm/gcmpregs.h>
#include <asm/mips-boards/maltaint.h>
#include <asm/irq.h>
#include <linux/hardirq.h>
#include <asm-generic/bitops/find.h>
#include <linux/bitmap.h>

static unsigned long _gic_base;
static unsigned int _irqbase, _mapsize, numvpes, numintrs;
static gic_intr_map_t *_intrmap;

static gic_pcpu_mask_t pcpu_masks[NR_CPUS];
static gic_pending_regs_t pending_regs[NR_CPUS];
static gic_intrmask_regs_t intrmask_regs[NR_CPUS];
static DEFINE_SPINLOCK(gic_lock);

void gic_send_ipi(unsigned int intr)
{
	pr_debug("CPU%d: %s status %08x\n", smp_processor_id(), __FUNCTION__, read_c0_status());
	GIC_REG(SHARED, GIC_SH_WEDGE) = (0x80000000 | intr);
}

/* This is Malta specific and needs to be exported */
static void vpe_local_setup(unsigned int numvpes) 
{
	int i;
    	unsigned long timer_interrupt = 5, perf_interrupt = 5;

	/*
	 * Setup the default performance counter timer interrupts 
	 * for all VPEs
	 */
	for (i = 0; i < numvpes; i++) {
		GIC_REG(VPE_LOCAL, GIC_VPE_OTHER_ADDR) = i;

		/* Are Interrupts locally routable? */
		if (GIC_REG(VPE_OTHER, GIC_VPE_CTL) & GIC_VPE_CTL_TIMER_RTBL_MSK)
			GIC_REG(VPE_OTHER, GIC_VPE_TIMER_MAP) =
				GIC_MAP_TO_PIN_MSK | timer_interrupt;

		if (GIC_REG(VPE_OTHER, GIC_VPE_CTL) & GIC_VPE_CTL_PERFCNT_RTBL_MSK)
			GIC_REG( VPE_OTHER, GIC_VPE_PERFCTR_MAP) =
				GIC_MAP_TO_PIN_MSK | perf_interrupt;
    }
}

unsigned int gic_get_int(void)
{
	unsigned int i;
	unsigned long *pending, *intrmask, *pcpu_mask;
	unsigned long *pending_abs, *intrmask_abs;

	/* Get per-cpu bitmaps */
	pending = pending_regs[smp_processor_id()].pending;
	intrmask = intrmask_regs[smp_processor_id()].intrmask;
	pcpu_mask = pcpu_masks[smp_processor_id()].pcpu_mask;

	pending_abs = (unsigned long *)GIC_REG_ABS_ADDR(SHARED, GIC_SH_PEND_31_0_OFS);
	intrmask_abs = (unsigned long *)GIC_REG_ABS_ADDR(SHARED, GIC_SH_MASK_31_0_OFS);

	bitmap_zero(pending, GIC_NUM_INTRS);
	bitmap_zero(intrmask, GIC_NUM_INTRS);

	bitmap_copy(pending, pending_abs, GIC_NUM_INTRS);
	bitmap_copy(intrmask, intrmask_abs, GIC_NUM_INTRS);

	bitmap_and(pending, pending, intrmask, GIC_NUM_INTRS);
	bitmap_and(pending, pending, pcpu_mask, GIC_NUM_INTRS);

	i = find_first_bit(pending, GIC_NUM_INTRS);

	pr_debug("CPU%d: %s pend=%d\n", smp_processor_id(), __FUNCTION__, i);
	return i;
}

static unsigned int gic_irq_startup(unsigned int irq)
{
	pr_debug("CPU%d: %s: irq%d\n", smp_processor_id(), __FUNCTION__, irq);
	irq -= _irqbase;
	GIC_REG_ADDR(SHARED, (GIC_SH_SMASK_31_0_OFS + (irq / 32))) = (1 << (irq % 32));
	return (0);
}

static void gic_irq_ack(unsigned int irq)
{
	pr_debug("CPU%d: %s: irq%d\n", smp_processor_id(), __FUNCTION__, irq);
	irq -= _irqbase;
	GIC_REG_ADDR(SHARED, (GIC_SH_RMASK_31_0_OFS + (irq / 32))) = (1 << (irq % 32));

	if (_intrmap[irq].trigtype == GIC_TRIG_EDGE)
		GIC_REG(SHARED, GIC_SH_WEDGE) = irq; 
}

static void gic_mask_irq(unsigned int irq)
{
	pr_debug("CPU%d: %s: irq%d\n", smp_processor_id(), __FUNCTION__, irq);
	irq -= _irqbase;
	GIC_REG_ADDR(SHARED, (GIC_SH_RMASK_31_0_OFS + (irq / 32))) = (1 << (irq % 32)); 
}

static void gic_unmask_irq(unsigned int irq)
{
	pr_debug("CPU%d: %s: irq%d\n", smp_processor_id(), __FUNCTION__, irq);
	irq -= _irqbase;
	GIC_REG_ADDR(SHARED, (GIC_SH_SMASK_31_0_OFS + (irq / 32))) = (1 << (irq % 32)); 
}

#ifdef CONFIG_SMP
static void gic_set_affinity(unsigned int irq, cpumask_t cpumask)
{
	cpumask_t	tmp = CPU_MASK_NONE;
	unsigned long	flags;
	int		i;

	pr_debug(KERN_DEBUG "%s called\n", __FUNCTION__);
	irq -= _irqbase;

	cpus_and(tmp, cpumask, cpu_online_map);
	if (cpus_empty(tmp))
		return;

	/* Assumption : cpumask refers to a single CPU */	
	spin_lock_irqsave(&gic_lock, flags);
	for (;;) {
		/* Re-route this IRQ */
		GIC_SH_MAP_TO_VPE_SMASK(irq, first_cpu(tmp));

		/* Update the intr_map */
		_intrmap[irq].cpunum = first_cpu(tmp);

		/* Update the pcpu_masks */
		for (i = 0; i < NR_CPUS; i++)
			clear_bit(irq, pcpu_masks[i].pcpu_mask);
		set_bit(irq, pcpu_masks[first_cpu(tmp)].pcpu_mask);
		
	}
	irq_desc[irq].affinity = cpumask;
	spin_unlock_irqrestore(&gic_lock, flags);
	
}
#endif

static struct irq_chip gic_irq_controller = {
	.name		=	"MIPS GIC",
	.startup	=	gic_irq_startup,
	.ack		=	gic_irq_ack,
	.mask		=	gic_mask_irq,
	.mask_ack	=	gic_mask_irq,
	.unmask		=	gic_unmask_irq,
	.eoi		=	gic_unmask_irq,
#ifdef CONFIG_SMP
	.set_affinity	=	gic_set_affinity,
#endif
};

static void __init setup_intr(unsigned int intr, unsigned int cpu, unsigned int pin, 
				unsigned int polarity, unsigned int trigtype)
{
	/* Setup Intr to Pin mapping */
	GIC_REG_ADDR(SHARED, GIC_SH_MAP_TO_PIN(intr)) = (GIC_MAP_TO_PIN_MSK | pin);		

	/* Setup Intr to CPU mapping */
	GIC_SH_MAP_TO_VPE_SMASK(intr, cpu);

	/* Setup Intr Polarity */
	GIC_SET_POLARITY(intr, polarity);

	/* Setup Intr Trigger Type */
	GIC_SET_TRIGGER(intr, trigtype);	

	/* Init Intr Masks */
	GIC_SET_INTR_MASK(intr, 0);
}

static void __init gic_basic_init(void)
{
	unsigned int i, cpu;

	/* Setup defaults */
	for (i = 0; i < GIC_NUM_INTRS; i++) {
		GIC_SET_POLARITY(i, GIC_POL_POS);
		GIC_SET_TRIGGER(i, GIC_TRIG_LEVEL);
		GIC_SET_INTR_MASK(i, 0);
	}

	/* Setup specifics */
	for (i = 0; i < _mapsize; i++) {
		cpu = _intrmap[i].cpunum;
		if (cpu == X)
			continue;

		setup_intr(_intrmap[i].intrnum, 
				_intrmap[i].cpunum, 
			   	_intrmap[i].pin, 
				_intrmap[i].polarity, 
		  	   	_intrmap[i].trigtype);
		/* Initialise per-cpu Interrupt software masks */
		set_bit(_intrmap[i].intrnum, pcpu_masks[cpu].pcpu_mask);
	}

	vpe_local_setup(numvpes);

	for (i = _irqbase; i < (_irqbase + numintrs); i++)
		set_irq_chip(i, &gic_irq_controller);
}

void __init gic_init(unsigned long gic_base_addr, unsigned long gic_addrspace_size, 
			gic_intr_map_t *intr_map, unsigned int intr_map_size,
			unsigned int irqbase)
{
	_gic_base = (unsigned long) ioremap_nocache(gic_base_addr, gic_addrspace_size);
	_irqbase = irqbase;
	_intrmap = intr_map;
	_mapsize = intr_map_size;

	numintrs = (GIC_REG(SHARED, GIC_SH_CONFIG) &
		GIC_SH_CONFIG_NUMINTRS_MSK) >> GIC_SH_CONFIG_NUMINTRS_SHF;
	numintrs = ((numintrs + 1) * 8);

	numvpes = (GIC_REG(SHARED, GIC_SH_CONFIG) &
		GIC_SH_CONFIG_NUMVPES_MSK) >> GIC_SH_CONFIG_NUMVPES_SHF;

	pr_debug("%s called\n", __FUNCTION__);

	gic_basic_init();
}
