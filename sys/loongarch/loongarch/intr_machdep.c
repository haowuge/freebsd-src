/*-
 * Copyright (c) 2015-2017 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Portions of this software were developed by SRI International and the
 * University of Cambridge Computer Laboratory under DARPA/AFRL contract
 * FA8750-10-C-0237 ("CTSRD"), as part of the DARPA CRASH research programme.
 *
 * Portions of this software were developed by the University of Cambridge
 * Computer Laboratory as part of the CTSRD Project, with support from the
 * UK Higher Education Innovation Fund (HEIF).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/module.h>
#include <sys/cpuset.h>
#include <sys/interrupt.h>
#include <sys/smp.h>

#include <machine/bus.h>
#include <machine/clock.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/frame.h>
#include <machine/intr.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#ifdef SMP
#include <machine/smp.h>
#endif

void intr_irq_handler(struct trapframe *tf, uint32_t rootnum);

struct intc_irqsrc {
	struct intr_irqsrc	isrc;
	u_int			irq;
};

struct intc_irqsrc isrcs[INTC_NIRQS];

static void
loongarch_mask_irq(void *source)
{
	int irq;
	uint32_t value;

	irq = (int)(uintptr_t)source;
	value = csr_read32(LOONGARCH_CSR_ECFG);

	switch (irq) {
	case IRQ_SWI0:
	case IRQ_SWI1:
	case IRQ_HWI0:
	case IRQ_HWI1:
	case IRQ_HWI2:
	case IRQ_HWI3:
	case IRQ_HWI4:
	case IRQ_HWI5:
	case IRQ_HWI6:
	case IRQ_HWI7:
	case IRQ_PCOV:
	case IRQ_TI:
	case IRQ_IPI:
		value &= ~(1 << irq);
		csr_write32(value, LOONGARCH_CSR_ECFG);
		break;

	default:
		panic("Unknown irq %d\n", irq);
	}
}

static void
loongarch_unmask_irq(void *source)
{
	int irq;
	uint32_t value;

	irq = (int)(uintptr_t)source;
	value = csr_read32(LOONGARCH_CSR_ECFG);

	switch (irq) {
	case IRQ_SWI0:
	case IRQ_SWI1:
	case IRQ_HWI0:
	case IRQ_HWI1:
	case IRQ_HWI2:
	case IRQ_HWI3:
	case IRQ_HWI4:
	case IRQ_HWI5:
	case IRQ_HWI6:
	case IRQ_HWI7:
	case IRQ_PCOV:
	case IRQ_TI:
	case IRQ_IPI:
		value |= (1 << irq);
		csr_write32(value, LOONGARCH_CSR_ECFG);
		break;

	default:
		panic("Unknown irq %d\n", irq);
	}
}

int
loongarch_setup_intr(const char *name, driver_filter_t *filt,
    void (*handler)(void*), void *arg, int irq, int flags, void **cookiep)
{
	struct intr_irqsrc *isrc;
	int error;

	if (irq < 0 || irq >= INTC_NIRQS)
		panic("%s: unknown intr %d", __func__, irq);

	isrc = &isrcs[irq].isrc;
	if (isrc->isrc_event == NULL) {
		error = intr_event_create(&isrc->isrc_event, isrc, 0, irq,
		    loongarch_mask_irq, loongarch_unmask_irq, NULL, NULL, "int%d", irq);
		if (error)
			return (error);
		loongarch_unmask_irq((void*)(uintptr_t)irq);
	}

	error = intr_event_add_handler(isrc->isrc_event, name,
	    filt, handler, arg, intr_priority(flags), flags, cookiep);
	if (error) {
		printf("Failed to setup intr: %d\n", irq);
		return (error);
	}

	return (0);
}

int
loongarch_teardown_intr(void *ih)
{

	/* TODO */

	return (0);
}

/* extract irq num */
static inline int get_irq(uint32_t estat)
{
	int ret = IRQ_IPI;
	int val = estat & ((1 << IRQ_NMI) - 1);

	while ((((1 << ret) & val) == 0) && ret > 0)
		ret--;
	
	return ret;
}

void
loongarch_cpu_intr(struct trapframe *frame)
{
	struct intr_irqsrc *isrc;
	int active_irq;

	active_irq = get_irq(frame->tf_estat);

	CTR3(KTR_TRAP, "%s: irq=%d, umode=%d", __func__, active_irq,
	    TRAPF_USERMODE(frame));

	switch (active_irq) {
	case IRQ_SWI0:
	case IRQ_SWI1:
	case IRQ_TI:
		critical_enter();
		isrc = &isrcs[active_irq].isrc;
		if (intr_isrc_dispatch(isrc, frame) != 0)
			printf("stray interrupt %d\n", active_irq);
		critical_exit();
		break;
	case IRQ_HWI0:
	case IRQ_HWI1:
	case IRQ_HWI2:
	case IRQ_HWI3:
	case IRQ_HWI4:
	case IRQ_HWI5:
	case IRQ_HWI6:
	case IRQ_HWI7:
	case IRQ_PCOV:
	case IRQ_IPI:
		intr_irq_handler(frame, INTR_ROOT_IRQ);
		break;
	}
}

#ifdef SMP
void
loongarch_setup_ipihandler(driver_filter_t *filt)
{

	loongarch_setup_intr("ipi", filt, NULL, NULL, IRQ_IPI,
	    INTR_TYPE_MISC, NULL);
}

void
loongarch_unmask_ipi(void)
{
	loongarch_unmask_irq((void*)IRQ_IPI);
}

/* Send mailbox buffer via Mail_Send */
void csr_mail_send(uint64_t data, int cpu, int mailbox)
{
	uint64_t val;

	/* Send high 32 bits */
	val = IOCSR_MBUF_SEND_BLOCKING;
	val |= (IOCSR_MBUF_SEND_BOX_HI(mailbox) << IOCSR_MBUF_SEND_BOX_SHIFT);
	val |= (cpu << IOCSR_MBUF_SEND_CPU_SHIFT);
	val |= (data & IOCSR_MBUF_SEND_H32_MASK);
	iocsr_write64(val, LOONGARCH_IOCSR_MBUF_SEND);

	/* Send low 32 bits */
	val = IOCSR_MBUF_SEND_BLOCKING;
	val |= (IOCSR_MBUF_SEND_BOX_LO(mailbox) << IOCSR_MBUF_SEND_BOX_SHIFT);
	val |= (cpu << IOCSR_MBUF_SEND_CPU_SHIFT);
	val |= (data << IOCSR_MBUF_SEND_BUF_SHIFT);
	iocsr_write64(val, LOONGARCH_IOCSR_MBUF_SEND);
};

uint32_t ipi_read_clear(int cpu)
{
	uint32_t action;

	/* Load the ipi register to figure out what we're supposed to do */
	action = iocsr_read32(LOONGARCH_IOCSR_IPI_STATUS);
	/* Clear the ipi register to clear the interrupt */
	iocsr_write32(action, LOONGARCH_IOCSR_IPI_CLEAR);
	wbflush();

	return action;
}

static void ipi_write_action(int cpu, uint32_t action)
{
	unsigned int irq = 0;

	while ((irq = ffs(action))) {
		uint32_t val = IOCSR_IPI_SEND_BLOCKING;

		val |= (irq - 1);
		val |= (cpu << IOCSR_IPI_SEND_CPU_SHIFT);
		iocsr_write32(val, LOONGARCH_IOCSR_IPI_SEND);
		action &= ~((irq - 1) << 1);
	}
}

/* Sending IPI */
static void
ipi_send(struct pcpu *pc, int ipi)
{
	CTR3(KTR_SMP, "%s: cpu: %d, ipi: %x", __func__, pc->pc_cpuid, ipi);

	atomic_set_32(&pc->pc_pending_ipis, ipi);
	ipi_write_action(pc->pc_hart, (uint32_t)ipi);

	CTR1(KTR_SMP, "%s: sent", __func__);
}

void
ipi_all_but_self(u_int ipi)
{
	cpuset_t other_cpus;

	other_cpus = all_cpus;
	CPU_CLR(PCPU_GET(cpuid), &other_cpus);

	CTR2(KTR_SMP, "%s: ipi: %x", __func__, ipi);
	ipi_selected(other_cpus, ipi);
}

void
ipi_cpu(int cpu, u_int ipi)
{
	ipi_send(cpuid_to_pcpu[cpu], ipi);
}

void
ipi_selected(cpuset_t cpus, u_int ipi)
{
	struct pcpu *pc;

	CTR1(KTR_SMP, "ipi_selected: ipi: %x", ipi);

	STAILQ_FOREACH(pc, &cpuhead, pc_allcpu) {
		if (CPU_ISSET(pc->pc_cpuid, &cpus)) {
			CTR3(KTR_SMP, "%s: pc: %p, ipi: %x\n", __func__, pc,
			    ipi);
			atomic_set_32(&pc->pc_pending_ipis, ipi);
			ipi_send(pc, (uint32_t)ipi);
		}
	}
}
#endif

/* Interrupt machdep initialization routine. */
static void
intc_init(void *dummy __unused)
{
	int error;
	int i;

	for (i = 0; i < INTC_NIRQS; i++) {
		isrcs[i].irq = i;
		error = intr_isrc_register(&isrcs[i].isrc, NULL,
		    0, "intc,%u", i);
		if (error != 0)
			printf("Can't register interrupt %d\n", i);
	}
}

SYSINIT(intc_init, SI_SUB_INTR, SI_ORDER_MIDDLE, intc_init, NULL);
