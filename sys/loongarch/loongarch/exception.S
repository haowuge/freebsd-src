/*-
 * Copyright (c) 2015-2018 Ruslan Bukin <br@bsdpad.com>
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

#include <machine/asm.h>
#include "assym.inc"

#include <machine/trap.h>
#include <machine/loongarchreg.h>

.altmacro
.macro save_registers mode

.if \mode == 0	/* We came from userspace. */
	csrrd	$t0, LOONGARCH_CSR_KS0

	move	$t1, $sp
	move	$sp, $t0	/* switch to kernel sp */

	/* save user sp */
	st.d	$t1, $sp, TF_R3
.endif
	// save old stack pointer
	csrwr	$sp, LOONGARCH_CSR_KS0
	csrrd	$sp, LOONGARCH_CSR_KS0

	// push GP registers
	addi.d	$sp, $sp, -(GP_REG_CONTEXT_SIZE + FP_REG_CONTEXT_SIZE + CSR_REG_CONTEXT_SIZE)

	st.d	$zero, $sp, 0 * RSIZE
	st.d	$ra, $sp, 1 * RSIZE
	st.d	$tp, $sp, 2 * RSIZE
	st.d	$a0, $sp, 4 * RSIZE
	st.d	$a1, $sp, 5 * RSIZE
	st.d	$a2, $sp, 6 * RSIZE
	st.d	$a3, $sp, 7 * RSIZE
	st.d	$a4, $sp, 8 * RSIZE
	st.d	$a5, $sp, 9 * RSIZE
	st.d	$a6, $sp, 10 * RSIZE
	st.d	$a7, $sp, 11 * RSIZE

	st.d	$t0, $sp, 12 * RSIZE
	st.d	$t1, $sp, 13 * RSIZE
	st.d	$t2, $sp, 14 * RSIZE
	st.d	$t3, $sp, 15 * RSIZE
	st.d	$t4, $sp, 16 * RSIZE
	st.d	$t5, $sp, 17 * RSIZE
	st.d	$t6, $sp, 18 * RSIZE
	st.d	$t7, $sp, 19 * RSIZE
	st.d	$t8, $sp, 20 * RSIZE
	st.d	$r21, $sp, 21 * RSIZE
	st.d	$fp, $sp, 22 * RSIZE

	/* save static regiter */
	st.d	$s0, $sp, 23 * RSIZE
	st.d	$s1, $sp, 24 * RSIZE
	st.d	$s2, $sp, 25 * RSIZE
	st.d	$s3, $sp, 26 * RSIZE
	st.d	$s4, $sp, 27 * RSIZE
	st.d	$s5, $sp, 28 * RSIZE
	st.d	$s6, $sp, 29 * RSIZE
	st.d	$s7, $sp, 30 * RSIZE
	st.d	$s8, $sp, 31 * RSIZE

.if \mode == 1
	/* Store kernel sp */
	csrrd	$t0, LOONGARCH_CSR_KS0	// read old sp
	st.d	$t0, $sp, 3 * RSIZE
.endif

	// push CSR registers
	addi.d	$sp, $sp, GP_REG_CONTEXT_SIZE

	csrrd	$t0, LOONGARCH_CSR_CRMD
	st.d	$t0, $sp, LOONGARCH_CSR_CRMD * RSIZE
	csrrd	$t0, LOONGARCH_CSR_PRMD
	st.d	$t0, $sp, LOONGARCH_CSR_PRMD * RSIZE
	csrrd	$t0, LOONGARCH_CSR_EUEN
	st.d	$t0, $sp, LOONGARCH_CSR_EUEN * RSIZE
	csrrd	$t0, LOONGARCH_CSR_MISC
	st.d	$t0, $sp, LOONGARCH_CSR_MISC * RSIZE
	csrrd	$t0, LOONGARCH_CSR_ECFG
	st.d	$t0, $sp, LOONGARCH_CSR_ECFG * RSIZE
	csrrd	$t0, LOONGARCH_CSR_ESTAT
	st.d	$t0, $sp, LOONGARCH_CSR_ESTAT * RSIZE
	csrrd	$t0, LOONGARCH_CSR_ERA
	st.d	$t0, $sp, LOONGARCH_CSR_ERA * RSIZE
	csrrd	$t0, LOONGARCH_CSR_BADV
	st.d	$t0, $sp, LOONGARCH_CSR_BADV * RSIZE
	csrrd	$t0, LOONGARCH_CSR_BADI
	st.d	$t0, $sp, LOONGARCH_CSR_BADI * RSIZE

	// push FP registers
	addi.d	$sp, $sp, CSR_REG_CONTEXT_SIZE
	csrrd	$t0, LOONGARCH_CSR_EUEN
	andi	$t0, $t0, CSR_EUEN_FPEN
	beqz	$t0, PushRegDone_\@

	fst.d  $fa0, $sp, 0 * RSIZE
	fst.d  $fa1, $sp, 1 * RSIZE
	fst.d  $fa2, $sp, 2 * RSIZE
	fst.d  $fa3, $sp, 3 * RSIZE
	fst.d  $fa4, $sp, 4 * RSIZE
	fst.d  $fa5, $sp, 5 * RSIZE
	fst.d  $fa6, $sp, 6 * RSIZE
	fst.d  $fa7, $sp, 7 * RSIZE
	fst.d  $ft0, $sp, 8 * RSIZE
	fst.d  $ft1, $sp, 9 * RSIZE
	fst.d  $ft2, $sp, 10 * RSIZE
	fst.d  $ft3, $sp, 11 * RSIZE
	fst.d  $ft4, $sp, 12 * RSIZE
	fst.d  $ft5, $sp, 13 * RSIZE
	fst.d  $ft6, $sp, 14 * RSIZE
	fst.d  $ft7, $sp, 15 * RSIZE
	fst.d  $ft8, $sp, 16 * RSIZE
	fst.d  $ft9, $sp, 17 * RSIZE
	fst.d  $ft10, $sp, 18 * RSIZE
	fst.d  $ft11, $sp, 19 * RSIZE
	fst.d  $ft12, $sp, 20 * RSIZE
	fst.d  $ft13, $sp, 21 * RSIZE
	fst.d  $ft14, $sp, 22 * RSIZE
	fst.d  $ft15, $sp, 23 * RSIZE
	fst.d  $fs0, $sp, 24 * RSIZE
	fst.d  $fs1, $sp, 25 * RSIZE
	fst.d  $fs2, $sp, 26 * RSIZE
	fst.d  $fs3, $sp, 27 * RSIZE
	fst.d  $fs4, $sp, 28 * RSIZE
	fst.d  $fs5, $sp, 29 * RSIZE
	fst.d  $fs6, $sp, 30 * RSIZE
	fst.d  $fs7, $sp, 31 * RSIZE

	movfcsr2gr  $t3, $fcsr0
	st.d        $t3, $sp, 32 * RSIZE  // Push the FCSR0 register.

	// Push the fcc0-fcc7 registers.
	movcf2gr    $t3, $fcc0
	or          $t2, $t3, $zero
	movcf2gr    $t3, $fcc1
	bstrins.d   $t2, $t3, 0xf, 0x8
	movcf2gr    $t3, $fcc2
	bstrins.d   $t2, $t3, 0x17, 0x10
	movcf2gr    $t3, $fcc3
	bstrins.d   $t2, $t3, 0x1f, 0x18
	movcf2gr    $t3, $fcc4
	bstrins.d   $t2, $t3, 0x27, 0x20
	movcf2gr    $t3, $fcc5
	bstrins.d   $t2, $t3, 0x2f, 0x28
	movcf2gr    $t3, $fcc6
	bstrins.d   $t2, $t3, 0x37, 0x30
	movcf2gr    $t3, $fcc7
	bstrins.d   $t2, $t3, 0x3f, 0x38
	st.d        $t2, $sp, 33 * RSIZE
	// Push exception context down
PushRegDone_\@:

	// point sp to stack top
	addi.d	$sp, $sp, -(GP_REG_CONTEXT_SIZE + CSR_REG_CONTEXT_SIZE)

.endm

.macro load_registers mode

	ld.d	$t1, $sp, (LOONGARCH_CSR_EUEN * RSIZE + GP_REG_CONTEXT_SIZE)
	csrrd	$t0, LOONGARCH_CSR_EUEN
	andi	$t0, $t0, CSR_EUEN_FPEN
	andi	$t1, $t1, CSR_EUEN_FPEN
	beq	$t0, $t1, PopRegs_\@
	beqz	$t1, CloseFP_\@

CloseFP_\@:
	// FIXME
PopRegs_\@:

	// pop CSR registers
	addi.d  $sp, $sp, GP_REG_CONTEXT_SIZE

	ld.d    $t0, $sp, LOONGARCH_CSR_CRMD * RSIZE
	csrwr   $t0, LOONGARCH_CSR_CRMD
	ld.d    $t0, $sp, LOONGARCH_CSR_PRMD * RSIZE
	csrwr   $t0, LOONGARCH_CSR_PRMD
	ld.d    $t0, $sp, LOONGARCH_CSR_ECFG * RSIZE
	csrwr   $t0, LOONGARCH_CSR_ECFG
	ld.d    $t0, $sp, LOONGARCH_CSR_ERA * RSIZE
	csrwr   $t0, LOONGARCH_CSR_ERA

	addi.d  $sp, $sp, CSR_REG_CONTEXT_SIZE  // Fource change the stack pointer befor pop the FP registers.

	beqz    $t1, PopGP_\@                      // If the FPE not set, only pop the GP registers.

	// Pop FP registers
	fld.d  $fa0, $sp, 0 * RSIZE
	fld.d  $fa1, $sp, 1 * RSIZE
	fld.d  $fa2, $sp, 2 * RSIZE
	fld.d  $fa3, $sp, 3 * RSIZE
	fld.d  $fa4, $sp, 4 * RSIZE
	fld.d  $fa5, $sp, 5 * RSIZE
	fld.d  $fa6, $sp, 6 * RSIZE
	fld.d  $fa7, $sp, 7 * RSIZE
	fld.d  $ft0, $sp, 8 * RSIZE
	fld.d  $ft1, $sp, 9 * RSIZE
	fld.d  $ft2, $sp, 10 * RSIZE
	fld.d  $ft3, $sp, 11 * RSIZE
	fld.d  $ft4, $sp, 12 * RSIZE
	fld.d  $ft5, $sp, 13 * RSIZE
	fld.d  $ft6, $sp, 14 * RSIZE
	fld.d  $ft7, $sp, 15 * RSIZE
	fld.d  $ft8, $sp, 16 * RSIZE
	fld.d  $ft9, $sp, 17 * RSIZE
	fld.d  $ft10, $sp, 18 * RSIZE
	fld.d  $ft11, $sp, 19 * RSIZE
	fld.d  $ft12, $sp, 20 * RSIZE
	fld.d  $ft13, $sp, 21 * RSIZE
	fld.d  $ft14, $sp, 22 * RSIZE
	fld.d  $ft15, $sp, 23 * RSIZE
	fld.d  $fs0, $sp, 24 * RSIZE
	fld.d  $fs1, $sp, 25 * RSIZE
	fld.d  $fs2, $sp, 26 * RSIZE
	fld.d  $fs3, $sp, 27 * RSIZE
	fld.d  $fs4, $sp, 28 * RSIZE
	fld.d  $fs5, $sp, 29 * RSIZE
	fld.d  $fs6, $sp, 30 * RSIZE
	fld.d  $fs7, $sp, 31 * RSIZE

	ld.d        $t0, $sp, 32 * RSIZE
	movgr2fcsr  $fcsr0, $t0	// pop fcsr0 register

	// Pop the fcc0-fcc7 registers.
	ld.d        $t0, $sp, 33 * RSIZE
	bstrpick.d  $t1, $t0, 7, 0
	movgr2cf    $fcc0, $t1
	bstrpick.d  $t1, $t0, 15, 8
	movgr2cf    $fcc1, $t1
	bstrpick.d  $t1, $t0, 23, 16
	movgr2cf    $fcc2, $t1
	bstrpick.d  $t1, $t0, 31, 24
	movgr2cf    $fcc3, $t1
	bstrpick.d  $t1, $t0, 39, 32
	movgr2cf    $fcc4, $t1
	bstrpick.d  $t1, $t0, 47, 40
	movgr2cf    $fcc5, $t1
	bstrpick.d  $t1, $t0, 55, 48
	movgr2cf    $fcc6, $t1
	bstrpick.d  $t1, $t0, 63, 56
	movgr2cf    $fcc7, $t1

PopGP_\@:
	// pop GP registers
	addi.d  $sp, $sp, -(GP_REG_CONTEXT_SIZE + CSR_REG_CONTEXT_SIZE)
	ld.d    $ra, $sp, 1 * RSIZE
	ld.d    $tp, $sp, 2 * RSIZE
	ld.d    $a0, $sp, 4 * RSIZE
	ld.d    $a1, $sp, 5 * RSIZE
	ld.d    $a2, $sp, 6 * RSIZE
	ld.d    $a3, $sp, 7 * RSIZE
	ld.d    $a4, $sp, 8 * RSIZE
	ld.d    $a5, $sp, 9 * RSIZE
	ld.d    $a6, $sp, 10 * RSIZE
	ld.d    $a7, $sp, 11 * RSIZE
	ld.d    $t0, $sp, 12 * RSIZE
	ld.d    $t1, $sp, 13 * RSIZE
	ld.d    $t2, $sp, 14 * RSIZE
	ld.d    $t3, $sp, 15 * RSIZE
	ld.d    $t4, $sp, 16 * RSIZE
	ld.d    $t5, $sp, 17 * RSIZE
	ld.d    $t6, $sp, 18 * RSIZE
	ld.d    $t7, $sp, 19 * RSIZE
	ld.d    $t8, $sp, 20 * RSIZE
	ld.d    $r21, $sp, 21 * RSIZE
	ld.d    $fp, $sp, 22 * RSIZE
	ld.d    $s0, $sp, 23 * RSIZE
	ld.d    $s1, $sp, 24 * RSIZE
	ld.d    $s2, $sp, 25 * RSIZE
	ld.d    $s3, $sp, 26 * RSIZE
	ld.d    $s4, $sp, 27 * RSIZE
	ld.d    $s5, $sp, 28 * RSIZE
	ld.d    $s6, $sp, 29 * RSIZE
	ld.d    $s7, $sp, 30 * RSIZE
	ld.d    $s8, $sp, 31 * RSIZE

.if \mode == 0
	/* We go to userspace. Load user sp */
	ld.d	$sp, $sp, TF_R3

.endif
.endm

	.align 12
ENTRY(cpu_exception_handler)
	csrrd	$t0, LOONGARCH_CSR_PRMD
	andi	$t0, $t0, 0x3
	beqz	$t0, 1f
	/* User mode detected */
	b	cpu_exception_handler_user
1:
	/* Supervisor mode detected */
	b	cpu_exception_handler_supervisor
END(cpu_exception_handler)

ENTRY(cpu_exception_handler_supervisor)
	save_registers 1
	move	$a0, $sp
	bl	_C_LABEL(do_trap_supervisor)
	load_registers 1
	ertn
END(cpu_exception_handler_supervisor)

ENTRY(cpu_exception_handler_user)
	save_registers 0
	move	$a0, $sp
	bl	_C_LABEL(do_trap_user)
	load_registers 0
	ertn
END(cpu_exception_handler_user)
