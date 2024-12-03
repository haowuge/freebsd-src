/*-
 * Copyright (c) 2015-2016 Ruslan Bukin <br@bsdpad.com>
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
#include <sys/assym.h>
#include <sys/proc.h>
#include <sys/mbuf.h>
#include <sys/vmmeter.h>
#include <sys/bus.h>
#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>
#include <vm/vm_map.h>

#include <machine/loongarchreg.h>
#include <machine/frame.h>
#include <machine/pcb.h>
#include <machine/cpu.h>
#include <machine/proc.h>
#include <machine/cpufunc.h>
#include <machine/pte.h>
#include <machine/intr.h>
#include <machine/machdep.h>
#include <machine/vmparam.h>

ASSYM(KERNBASE, KERNBASE);
ASSYM(VM_MAXUSER_ADDRESS, VM_MAXUSER_ADDRESS);
ASSYM(VM_MAX_KERNEL_ADDRESS, VM_MAX_KERNEL_ADDRESS);
ASSYM(VM_EARLY_DTB_ADDRESS, VM_EARLY_DTB_ADDRESS);
ASSYM(PMAP_MAPDEV_EARLY_SIZE, PMAP_MAPDEV_EARLY_SIZE);

ASSYM(PCB_ONFAULT, offsetof(struct pcb, pcb_onfault));
ASSYM(PCB_SIZE, sizeof(struct pcb));
ASSYM(PCB_ERA, offsetof(struct pcb, pcb_era));
ASSYM(PCB_A0, offsetof(struct pcb, pcb_a0));
ASSYM(PCB_EUEN, offsetof(struct pcb, pcb_euen));
ASSYM(PCB_CRMD, offsetof(struct pcb, pcb_crmd));
ASSYM(PCB_PRMD, offsetof(struct pcb, pcb_prmd));
ASSYM(PCB_ECFG, offsetof(struct pcb, pcb_ecfg));
ASSYM(PCB_ESTAT, offsetof(struct pcb, pcb_estat));
ASSYM(PCB_FCSR, offsetof(struct pcb, uf.pcb_fcsr0));
ASSYM(PCB_S, offsetof(struct pcb, pcb_regs[23]));
ASSYM(PCB_RA, offsetof(struct pcb, pcb_regs[1]));
ASSYM(PCB_SP, offsetof(struct pcb, pcb_regs[3]));
ASSYM(PCB_TP, offsetof(struct pcb, pcb_regs[2]));
ASSYM(PCB_FP, offsetof(struct pcb, pcb_regs[22]));
ASSYM(PCB_F, offsetof(struct pcb, pcb_fregs));

ASSYM(SF_UC, offsetof(struct sigframe, sf_uc));

ASSYM(PC_CURPCB, offsetof(struct pcpu, pc_curpcb));
ASSYM(PC_CURTHREAD, offsetof(struct pcpu, pc_curthread));

ASSYM(TD_PCB, offsetof(struct thread, td_pcb));
ASSYM(TD_FLAGS, offsetof(struct thread, td_flags));
ASSYM(TD_AST, offsetof(struct thread, td_ast));
ASSYM(TD_PROC, offsetof(struct thread, td_proc));
ASSYM(TD_FRAME, offsetof(struct thread, td_frame));
ASSYM(TD_MD, offsetof(struct thread, td_md));
ASSYM(TD_LOCK, offsetof(struct thread, td_lock));

ASSYM(TF_SIZE, roundup2(sizeof(struct trapframe), STACKALIGNBYTES + 1));
ASSYM(TF_REGS, offsetof(struct trapframe, tf_regs));
ASSYM(TF_R0, offsetof(struct trapframe, tf_regs[0]));
ASSYM(TF_R1, offsetof(struct trapframe, tf_regs[1]));
ASSYM(TF_R2, offsetof(struct trapframe, tf_regs[2]));
ASSYM(TF_R3, offsetof(struct trapframe, tf_regs[3]));
ASSYM(TF_R4, offsetof(struct trapframe, tf_regs[4]));
ASSYM(TF_R5, offsetof(struct trapframe, tf_regs[5]));
ASSYM(TF_R6, offsetof(struct trapframe, tf_regs[6]));
ASSYM(TF_R7, offsetof(struct trapframe, tf_regs[7]));
ASSYM(TF_R8, offsetof(struct trapframe, tf_regs[8]));
ASSYM(TF_R9, offsetof(struct trapframe, tf_regs[9]));
ASSYM(TF_R10, offsetof(struct trapframe, tf_regs[10]));
ASSYM(TF_R11, offsetof(struct trapframe, tf_regs[11]));
ASSYM(TF_R12, offsetof(struct trapframe, tf_regs[12]));
ASSYM(TF_R13, offsetof(struct trapframe, tf_regs[13]));
ASSYM(TF_R14, offsetof(struct trapframe, tf_regs[14]));
ASSYM(TF_R15, offsetof(struct trapframe, tf_regs[15]));
ASSYM(TF_R16, offsetof(struct trapframe, tf_regs[16]));
ASSYM(TF_R17, offsetof(struct trapframe, tf_regs[17]));
ASSYM(TF_R18, offsetof(struct trapframe, tf_regs[18]));
ASSYM(TF_R19, offsetof(struct trapframe, tf_regs[19]));
ASSYM(TF_R20, offsetof(struct trapframe, tf_regs[20]));
ASSYM(TF_R21, offsetof(struct trapframe, tf_regs[21]));
ASSYM(TF_R22, offsetof(struct trapframe, tf_regs[22]));
ASSYM(TF_R23, offsetof(struct trapframe, tf_regs[23]));
ASSYM(TF_R24, offsetof(struct trapframe, tf_regs[24]));
ASSYM(TF_R25, offsetof(struct trapframe, tf_regs[25]));
ASSYM(TF_R26, offsetof(struct trapframe, tf_regs[26]));
ASSYM(TF_R27, offsetof(struct trapframe, tf_regs[27]));
ASSYM(TF_R28, offsetof(struct trapframe, tf_regs[28]));
ASSYM(TF_R29, offsetof(struct trapframe, tf_regs[29]));
ASSYM(TF_R30, offsetof(struct trapframe, tf_regs[30]));
ASSYM(TF_R31, offsetof(struct trapframe, tf_regs[31]));

ASSYM(TF_A, offsetof(struct trapframe, tf_regs[4]));
ASSYM(TF_T, offsetof(struct trapframe, tf_regs[12]));
ASSYM(TF_S, offsetof(struct trapframe, tf_regs[23]));
ASSYM(TF_RA, offsetof(struct trapframe, tf_regs[1]));
ASSYM(TF_TP, offsetof(struct trapframe, tf_regs[2]));
ASSYM(TF_SP, offsetof(struct trapframe, tf_regs[3]));

ASSYM(TF_A0, offsetof(struct trapframe, tf_a0));
ASSYM(TF_ERA, offsetof(struct trapframe, tf_era));
ASSYM(TF_BADVADDR, offsetof(struct trapframe, tf_badvaddr));
ASSYM(TF_CRMD, offsetof(struct trapframe, tf_crmd));
ASSYM(TF_PRMD, offsetof(struct trapframe, tf_prmd));
ASSYM(TF_EUEN, offsetof(struct trapframe, tf_euen));
ASSYM(TF_MISC, offsetof(struct trapframe, tf_misc));
ASSYM(TF_ECFG, offsetof(struct trapframe, tf_ecfg));
ASSYM(TF_ESTAT, offsetof(struct trapframe, tf_estat));

ASSYM(LOONGARCH_BOOTPARAMS_SIZE, sizeof(struct loongarch_bootparams));
ASSYM(LOONGARCH_BOOTPARAMS_KERN_L1PT, offsetof(struct loongarch_bootparams, kern_l1pt));
ASSYM(LOONGARCH_BOOTPARAMS_KERN_PHYS, offsetof(struct loongarch_bootparams, kern_phys));
ASSYM(LOONGARCH_BOOTPARAMS_KERN_STACK, offsetof(struct loongarch_bootparams,
    kern_stack));
ASSYM(LOONGARCH_BOOTPARAMS_DTBP_VIRT, offsetof(struct loongarch_bootparams, dtbp_virt));
ASSYM(LOONGARCH_BOOTPARAMS_DTBP_PHYS, offsetof(struct loongarch_bootparams, dtbp_phys));
ASSYM(LOONGARCH_BOOTPARAMS_MODULEP, offsetof(struct loongarch_bootparams, modulep));
