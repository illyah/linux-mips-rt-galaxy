#ifndef __ASM_BUG_H
#define __ASM_BUG_H

#include <asm/sgidefs.h>

#ifdef CONFIG_HWTRIGGER

/* Hardware assisted debugging */

extern void hwtriggerinfo(char *file, int line, void *regs, unsigned int code, char *why);
extern int chwtrigger(void *regs);

#define _HWTRIGGER(code)						\
do {									\
	*(volatile unsigned int *)0xbfc00000 = code;			\
	__asm__ __volatile__ ("sync");					\
} while (0)

#define HWTRIGGER(regs, code, why)					\
do {									\
	_HWTRIGGER(code);						\
	hwtriggerinfo(__FILE__, __LINE__, regs, code, why);		\
} while (0)

#define CHWTRIGGER(regs, code, why)					\
do {									\
	if (chwtrigger(regs))						\
		HWTRIGGER (regs, code, why);				\
} while (0)
#else
#define _HWTRIGGER(code)
#define HWTRIGGER(regs, code, why)
#define CHWTRIGGER(regs, code, why)
#endif

#ifdef CONFIG_BUG

#include <asm/break.h>

#define BUG()								\
do {									\
	_HWTRIGGER(0xffff);						\
	__asm__ __volatile__("break %0" : : "i" (BRK_BUG));		\
} while (0)

#define HAVE_ARCH_BUG

#if (_MIPS_ISA > _MIPS_ISA_MIPS1)

#define BUG_ON(condition)						\
do {									\
	if (condition) _HWTRIGGER(0xffff);				\
	__asm__ __volatile__("tne $0, %0, %1"				\
			     : : "r" (condition), "i" (BRK_BUG));	\
} while (0)

#define HAVE_ARCH_BUG_ON

#endif /* _MIPS_ISA > _MIPS_ISA_MIPS1 */

#endif

#include <asm-generic/bug.h>

#endif /* __ASM_BUG_H */
