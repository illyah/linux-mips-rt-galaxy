/*
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * Copyright (C) 2007 MIPS Technologies, Inc.
 *    Chris Dearman (chris@mips.com)
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/interrupt.h>
#include <linux/compiler.h>

#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/cpu.h>
#include <asm/processor.h>
#include <asm/system.h>
#include <asm/hardirq.h>
#include <asm/mmu_context.h>
#include <asm/smp.h>
#include <asm/time.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/mips_mt.h>

/*
 * Common setup before any secondaries are started
 */
void __init cmp_smp_setup(void)
{
	int ncpu = 0;

	pr_debug("SMPCMP: CPU%d: plat_smp_setup\n", smp_processor_id());

#ifdef CONFIG_MIPS_MT_FPAFF
	/* If we have an FPU, enroll ourselves in the FPU-full mask */
	if (cpu_has_fpu)
		cpu_set(0, mt_fpu_cpumask);
#endif /* CONFIG_MIPS_MT_FPAFF */

	printk(KERN_INFO "Detected %i available secondary CPU(s)\n", ncpu);
}

void __init cmp_prepare_cpus(unsigned int max_cpus)
{
	pr_debug("SMPCMP: CPU%d: plat_prepare_cpus %d\n", smp_processor_id(), max_cpus);

	/* FIXME: some of these options are per-system, some per-core and some per-cpu */
	mips_mt_set_cpuoptions();
}

/*
 * Setup the PC, SP, and GP of a secondary processor and start it running
 * smp_bootstrap is the place to resume from
 * __KSTK_TOS(idle) is apparently the stack pointer
 * (unsigned long)idle->thread_info the gp
 */
void cmp_boot_secondary(int cpu, struct task_struct *idle)
{
	struct thread_info *gp = task_thread_info(idle);
	unsigned long sp = __KSTK_TOS(idle);
	unsigned long pc = (unsigned long)&smp_bootstrap;
	unsigned long a0 = 0;

	pr_debug("SMPCMP: CPU%d: prom_boot_secondary cpu %d\n", smp_processor_id(), cpu);

#if 0
	/* Needed? */
	flush_icache_range((unsigned long)gp,
	                   (unsigned long)(gp + sizeof(struct thread_info)));
#endif

#if 0
	amon_cpu_start(cpu, pc, sp, gp, a0);
#else
	gp = gp;
	sp = sp;
	pc = pc;
	a0 = a0;
#endif
}

void cmp_init_secondary(void)
{
	pr_debug("SMPCMP: CPU%d: prom_init_secondary\n", smp_processor_id());
	/* Enable per-cpu interrupts: platform specific */
}

void cmp_smp_finish(void)
{
	pr_debug("SMPCMP: CPU%d: prom_smp_finish\n", smp_processor_id());

#ifdef CONFIG_MIPS_MT_FPAFF
	/* If we have an FPU, enroll ourselves in the FPU-full mask */
	if (cpu_has_fpu)
		cpu_set(smp_processor_id(), mt_fpu_cpumask);
#endif /* CONFIG_MIPS_MT_FPAFF */

	local_irq_enable();
}

void cmp_cpus_done(void)
{
	pr_debug("SMPCMP: CPU%d: prom_cpus_done\n", smp_processor_id());
}

