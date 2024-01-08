/*	$NetBSD: asm.h,v 1.29 2000/12/14 21:29:51 jeffs Exp $	*/

/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 1992, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Ralph Campbell.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)machAsmDefs.h	8.1 (Berkeley) 6/10/93
 *	JNPR: asm.h,v 1.10 2007/08/09 11:23:32 katta
 */

/*
 * machAsmDefs.h --
 *
 *	Macros used when writing assembler programs.
 *
 *	Copyright (C) 1989 Digital Equipment Corporation.
 *	Permission to use, copy, modify, and distribute this software and
 *	its documentation for any purpose and without fee is hereby granted,
 *	provided that the above copyright notice appears in all copies.
 *	Digital Equipment Corporation makes no representations about the
 *	suitability of this software for any purpose.  It is provided "as is"
 *	without express or implied warranty.
 *
 * from: Header: /sprite/src/kernel/mach/ds3100.md/RCS/machAsmDefs.h,
 *	v 1.2 89/08/15 18:28:24 rab Exp  SPRITE (DECWRL)
 */

#ifndef _MACHINE_ASM_H_
#define	_MACHINE_ASM_H_

#include <machine/abi.h>
#include <machine/regdef.h>
#include <machine/endian.h>

#undef __FBSDID
#if !defined(lint) && !defined(STRIP_FBSDID)
#define	__FBSDID(s)	.ident s
#else
#define	__FBSDID(s)	/* nothing */
#endif

/* LoongArch pref instruction. */
#ifdef CONFIG_CPU_HAS_PREFETCH

#define PREF(hint, addr, offs)                          \
                preld   hint, addr, offs;               \

#define PREFX(hint, addr, index)                        \
                preldx  hint, addr, index;              \

#else /* !CONFIG_CPU_HAS_PREFETCH */

#define PREF(hint, addr, offs)
#define PREFX(hint, addr, index)

#endif /* !CONFIG_CPU_HAS_PREFETCH */


/*
 * Define -pg profile entry code.
 * Must always be noreorder, must never use a macro instruction
 * Final addiu to t9 must always equal the size of this _KERN_MCOUNT
 */
#define	_KERN_MCOUNT			\
	.set	push;			\
	.set	noreorder;		\
	.set	noat;			\
	subu	sp,sp,16;		\
	sw	t9,12(sp);		\
	move	AT,ra;			\
	lui	t9,%hi(_mcount);	\
	addiu	t9,t9,%lo(_mcount);	\
	jalr	t9;			\
	nop;				\
	lw	t9,4(sp);		\
	addiu	sp,sp,8;		\
	addiu	t9,t9,40;		\
	.set	pop;

#ifdef GPROF
#define	MCOUNT _KERN_MCOUNT
#else
#define	MCOUNT
#endif

#define	_C_LABEL(x)	x

#ifdef USE_AENT
#define	AENT(x)		\
	.aent	x, 0
#else
#define	AENT(x)
#endif

/*
 * WARN_REFERENCES: create a warning if the specified symbol is referenced
 */
#define	WARN_REFERENCES(_sym,_msg)				\
	.section .gnu.warning. ## _sym ; .ascii _msg ; .text

/*
 * WEAK_ALIAS: create a weak alias.
 */
#define	WEAK_ALIAS(alias,sym)						\
	.weak alias;							\
	alias = sym

/*
 * STRONG_ALIAS: create a strong alias.
 */
#define STRONG_ALIAS(alias,sym)						\
	.globl alias;							\
	alias = sym

#define	GLOBAL(sym)						\
	.globl sym; sym:

#define	ENTRY(sym)						\
	.text; .globl sym; .type sym,@function; sym:

/*
 * LEAF
 *	A leaf routine does
 *	- call no other function,
 *	- never use any register that callee-saved (S0-S8), and
 *	- not use any local stack storage.
 */
#define	LEAF(x)			\
	.text;			\
	.globl x;		\
	.align 3;		\
	.cfi_startproc;		\
	.type _C_LABEL(x), @function;	\
_C_LABEL(x): 			\
	MCOUNT

/*
 * LEAF_NOPROFILE
 *	No profilable leaf routine.
 */
#define	LEAF_NOPROFILE(x)	\
	.globl	_C_LABEL(x);	\
	.ent	_C_LABEL(x), 0;	\
_C_LABEL(x): ;			\
	.frame	sp, 0, ra

/*
 * XLEAF
 *	declare alternate entry to leaf routine
 */
#define	XLEAF(x)		\
	.globl	_C_LABEL(x);	\
	AENT (_C_LABEL(x));	\
_C_LABEL(x):

/*
 * NESTED
 *	A function calls other functions and needs
 *	therefore stack space to save/restore registers.
 */
#define	NESTED(x, fsize, retpc)		\
	.globl	_C_LABEL(x);		\
	.ent	_C_LABEL(x), 0;		\
_C_LABEL(x): ;				\
	.frame	sp, fsize, retpc;	\
	MCOUNT

/*
 * NESTED_NOPROFILE(x)
 *	No profilable nested routine.
 */
#define	NESTED_NOPROFILE(x, fsize, retpc)	\
	.globl	_C_LABEL(x);			\
	.ent	_C_LABEL(x), 0;			\
_C_LABEL(x): ;					\
	.frame	sp, fsize, retpc

/*
 * XNESTED
 *	declare alternate entry point to nested routine.
 */
#define	XNESTED(x)		\
	.globl	_C_LABEL(x);	\
	AENT (_C_LABEL(x));	\
_C_LABEL(x):

/*
 * END
 *	Mark end of a procedure.
 */
#undef	END
#define	END(x)			\
	.cfi_endproc;		\
	.size x, .- x;		\

/*
 * IMPORT -- import external symbol
 */
#define	IMPORT(sym, size)	\
	.extern _C_LABEL(sym),size

/*
 * EXPORT -- export definition of symbol
 */
#define	EXPORT(x)		\
	.globl	_C_LABEL(x);	\
_C_LABEL(x):

/*
 * VECTOR
 *	exception vector entrypoint
 *	XXX: regmask should be used to generate .mask
 */
#define	VECTOR(x, regmask)	\
	.ent	_C_LABEL(x),0;	\
	EXPORT(x);		\

#define	VECTOR_END(x)		\
	EXPORT(x ## End);	\
	END(x)

/*
 * Macros to panic and printf from assembly language.
 */
#define	PANIC(msg)			\
	PTR_LA	a0, 9f;			\
	jal	_C_LABEL(panic);	\
	nop;				\
	MSG(msg)

#define	PANIC_KSEG0(msg, reg)	PANIC(msg)

#define	PRINTF(msg)			\
	PTR_LA	a0, 9f;			\
	jal	_C_LABEL(printf);	\
	nop;				\
	MSG(msg)

#define	MSG(msg)			\
	.rdata;				\
9:	.asciiz	msg;			\
	.text

#define	ASMSTR(str)			\
	.asciiz str;			\
	.align	3

#define	FP_L	ldc1
#define	FP_S	sdc1

/*
 * Use the following macros in assemblercode to load/store registers,
 * pointers etc.
 */
#if (SZREG == 4)
#define REG_L           ld.w
#define REG_S           st.w
#define REG_ADD         add.w
#define REG_SUB         sub.w
#define BSTRINS		bstrins.w
#define	REG_SCALESHIFT	2
#elif (SZREG == 8)
#define REG_L           ld.d
#define REG_S           st.d
#define REG_ADD         add.d
#define REG_SUB         sub.d
#define BSTRINS		bstrins.d
#define	REG_SCALESHIFT	3
#endif


/*
 * How to add/sub/load/store/shift pointers.
 */
#if (__SIZEOF_POINTER__ == 4)
#define PTR_ADD         add.w
#define PTR_ADDI        addi.w
#define PTR_SUB         sub.w
#define PTR_L           ld.w
#define PTR_S           st.w
#define PTR_LI          li.w
#define PTR_SLL         slli.w
#define PTR_SLLV        sll.w
#define PTR_SRL         srli.w
#define PTR_SRLV        srl.w
#define PTR_SRA         srai.w
#define PTR_SRAV        sra.w
#define PTR_SCALESHIFT  2
#define PTRSIZE         4
#define PTRLOG          2
#ifdef __ASSEMBLY__
#define PTR_WORD        .word
#endif
#elif (__SIZEOF_POINTER__ == 8)
#define PTR_ADD         add.d
#define PTR_ADDI        addi.d
#define PTR_SUB         sub.d
#define PTR_L           ld.d
#define PTR_S           st.d
#define PTR_LI          li.d
#define PTR_SLL         slli.d
#define PTR_SLLV        sll.d
#define PTR_SRL         srli.d
#define PTR_SRLV        srl.d
#define PTR_SRA         srai.d
#define PTR_SRAV        sra.d
#define PTR_SCALESHIFT  3
#define PTRSIZE         8
#define PTRLOG          3
#ifdef __ASSEMBLY__
#define PTR_WORD        .dword
#endif
#endif /* SIZEOF*/

/*
 * How to add/sub/load/store/shift C int variables.
*/
#if (__SIZEOF_INT__ == 4)
#define INT_ADD         add.w
#define INT_ADDI        addi.w
#define INT_SUB         sub.w
#define INT_L           ld.w
#define INT_S           st.w
#define INT_SLL         slli.w
#define INT_SLLV        sll.w
#define INT_SRL         srli.w
#define INT_SRLV        srl.w
#define INT_SRA         srai.w
#define INT_SRAV        sra.w
#ifdef __ASSEMBLY__
#define	INT_WORD	.word
#endif
#define	INT_SCALESHIFT	2
#elif (__SIZEOF_INT__ == 8)
#define INT_ADD         add.d
#define INT_ADDI        addi.d
#define INT_SUB         sub.d
#define INT_L           ld.d
#define INT_S           st.d
#define INT_SLL         slli.d
#define INT_SLLV        sll.d
#define INT_SRL         srli.d
#define INT_SRLV        srl.d
#define INT_SRA         srai.d
#define INT_SRAV        sra.d
#ifdef __ASSEMBLY__
#define	INT_WORD	.dword
#endif
#define	INT_SCALESHIFT	3
#endif

/*
 * How to add/sub/load/store/shift C long variables.
 */
#if (__SIZEOF_LONG__ == 4)
#define LONG_ADD        add.w
#define LONG_ADDI       addi.w
#define LONG_SUB        sub.w
#define LONG_L          ld.w
#define LONG_S          st.w
#define LONG_SLL        slli.w
#define LONG_SLLV       sll.w
#define LONG_SRL        srli.w
#define LONG_SRLV       srl.w
#define LONG_SRA        srai.w
#define LONG_SRAV       sra.w
#define LONGSIZE        4
#define LONGMASK        3
#define LONGLOG         2
#define	LONG_SCALESHIFT	2
#ifdef __ASSEMBLY__
#define LONG            .word
#endif
#elif(__SIZEOF_LONG__ == 8)
#define LONG_ADD        add.d
#define LONG_ADDI       addi.d
#define LONG_SUB        sub.d
#define LONG_L          ld.d
#define LONG_S          st.d
#define LONG_SLL        slli.d
#define LONG_SLLV       sll.d
#define LONG_SRL        srli.d
#define LONG_SRLV       srl.d
#define LONG_SRA        srai.d
#define LONG_SRAV       sra.d
#define LONGSIZE        8
#define LONGMASK        7
#define LONGLOG         3
#define	LONG_SCALESHIFT	3
#ifdef __ASSEMBLY__
#define LONG            .dword
#endif
#endif

#endif /* !_MACHINE_ASM_H_ */
