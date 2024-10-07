/*-
 * Copyright (c) 2014 Andrew Turner
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

#include "opt_platform.h"

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/boot.h>
#include <sys/buf.h>
#include <sys/bus.h>
#include <sys/cons.h>
#include <sys/cpu.h>
#include <sys/devmap.h>
#include <sys/efi.h>
#include <sys/exec.h>
#include <sys/imgact.h>
#include <sys/kdb.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/limits.h>
#include <sys/linker.h>
#include <sys/msgbuf.h>
#include <sys/pcpu.h>
#include <sys/physmem.h>
#include <sys/proc.h>
#include <sys/ptrace.h>
#include <sys/reboot.h>
#include <sys/reg.h>
#include <sys/rwlock.h>
#include <sys/sched.h>
#include <sys/signalvar.h>
#include <sys/syscallsubr.h>
#include <sys/sysent.h>
#include <sys/sysproto.h>
#include <sys/tslog.h>
#include <sys/ucontext.h>
#include <sys/vmmeter.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_kern.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_phys.h>
#include <vm/pmap.h>
#include <vm/vm_map.h>
#include <vm/vm_pager.h>

#include <machine/cpu.h>
#include <machine/fpe.h>
#include <machine/intr.h>
#include <machine/kdb.h>
#include <machine/machdep.h>
#include <machine/metadata.h>
#include <machine/pcb.h>
#include <machine/pte.h>
#include <machine/loongarchreg.h>
#include <machine/trap.h>
#include <machine/vmparam.h>

#ifdef FDT
#include <contrib/libfdt/libfdt.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#endif

struct pcpu __pcpu[MAXCPU];

static struct trapframe proc0_tf;

int early_boot = 1;
int cold = 1;

#define	DTB_SIZE_MAX	(1024 * 1024)

struct kva_md_info kmi;

int64_t dcache_line_size;	/* The minimum D cache line size */
int64_t icache_line_size;	/* The minimum I cache line size */
int64_t idcache_line_size;	/* The minimum cache line size */

#define BOOT_HART_INVALID	0xffffffff
uint32_t boot_hart = BOOT_HART_INVALID;	/* The hart we booted on. */

/*
 * Physical address of the EFI System Table. Stashed from the metadata hints
 * passed into the kernel and used by the EFI code to call runtime services.
 */
vm_paddr_t efi_systbl_phys;
static struct efi_map_header *efihdr;

cpuset_t all_harts;

extern int *end;

static char static_kenv[PAGE_SIZE];

static void
cpu_startup(void *dummy)
{

	printcpuinfo(0);

	printf("real memory  = %ju (%ju MB)\n", ptoa((uintmax_t)realmem),
	    ptoa((uintmax_t)realmem) / (1024 * 1024));

	/*
	 * Display any holes after the first chunk of extended memory.
	 */
	if (bootverbose) {
		int indx;

		printf("Physical memory chunk(s):\n");
		for (indx = 0; phys_avail[indx + 1] != 0; indx += 2) {
			vm_paddr_t size;

			size = phys_avail[indx + 1] - phys_avail[indx];
			printf(
			    "0x%016jx - 0x%016jx, %ju bytes (%ju pages)\n",
			    (uintmax_t)phys_avail[indx],
			    (uintmax_t)phys_avail[indx + 1] - 1,
			    (uintmax_t)size, (uintmax_t)size / PAGE_SIZE);
		}
	}

	vm_ksubmap_init(&kmi);

	printf("avail memory = %ju (%ju MB)\n",
	    ptoa((uintmax_t)vm_free_count()),
	    ptoa((uintmax_t)vm_free_count()) / (1024 * 1024));
	if (bootverbose)
		devmap_print_table();

	bufinit();
	vm_pager_bufferinit();
}

SYSINIT(cpu, SI_SUB_CPU, SI_ORDER_FIRST, cpu_startup, NULL);

int
cpu_idle_wakeup(int cpu)
{
	return (0);
}

void
cpu_idle(int busy)
{

	spinlock_enter();
	if (!busy)
		cpu_idleclock();
	if (!sched_runnable()) {
		// FIXME
		mb();
		__asm __volatile("idle 0");
	}
	if (!busy)
		cpu_activeclock();
	spinlock_exit();
}

void
cpu_halt(void)
{
	intr_disable();

	for (;;)
		__asm __volatile("idle 0");
	/* NOTREACHED */
}

/*
 * Flush the D-cache for non-DMA I/O so that the I-cache can
 * be made coherent later.
 */
void
cpu_flush_dcache(void *ptr, size_t len)
{

	/* TBD */
}

/* Get current clock frequency for the given CPU ID. */
int
cpu_est_clockrate(int cpu_id, uint64_t *rate)
{

	panic("cpu_est_clockrate");
}

void
cpu_pcpu_init(struct pcpu *pcpu, int cpuid, size_t size)
{
}

void
spinlock_enter(void)
{
	struct thread *td;
	register_t reg;

	td = curthread;
	if (td->td_md.md_spinlock_count == 0) {
		reg = intr_disable();
		td->td_md.md_spinlock_count = 1;
		td->td_md.md_saved_sstatus_ie = reg;
		critical_enter();
	} else
		td->td_md.md_spinlock_count++;
}

void
spinlock_exit(void)
{
	struct thread *td;
	register_t sstatus_ie;

	td = curthread;
	sstatus_ie = td->td_md.md_saved_sstatus_ie;
	td->td_md.md_spinlock_count--;
	if (td->td_md.md_spinlock_count == 0) {
		critical_exit();
		intr_restore(sstatus_ie);
	}
}

/*
 * Construct a PCB from a trapframe. This is called from kdb_trap() where
 * we want to start a backtrace from the function that caused us to enter
 * the debugger. We have the context in the trapframe, but base the trace
 * on the PCB. The PCB doesn't have to be perfect, as long as it contains
 * enough for a backtrace.
 */
void
makectx(struct trapframe *tf, struct pcb *pcb)
{

	memcpy(pcb->pcb_regs, tf->tf_regs, sizeof(tf->tf_regs));

	pcb->pcb_era = tf->tf_era;
	pcb->pcb_a0 = tf->tf_a0;
	pcb->pcb_crmd = tf->tf_crmd;
	pcb->pcb_prmd = tf->tf_prmd;
	pcb->pcb_ecfg = tf->tf_ecfg;
	pcb->pcb_estat = tf->tf_estat;
}

static void
init_proc0(vm_offset_t kstack)
{
	struct pcpu *pcpup;

	pcpup = &__pcpu[0];

	proc_linkup0(&proc0, &thread0);
	thread0.td_kstack = kstack;
	thread0.td_kstack_pages = kstack_pages;
	thread0.td_pcb = (struct pcb *)(thread0.td_kstack +
	    thread0.td_kstack_pages * PAGE_SIZE) - 1;
	thread0.td_pcb->pcb_fpflags = 0;
	thread0.td_frame = &proc0_tf;
	pcpup->pc_curpcb = thread0.td_pcb;
}

typedef void (*efi_map_entry_cb)(struct efi_md *, void *argp);

static void
foreach_efi_map_entry(struct efi_map_header *efihdr, efi_map_entry_cb cb, void *argp)
{
	struct efi_md *map, *p;
	size_t efisz;
	int ndesc, i;

	/*
	 * Memory map data provided by UEFI via the GetMemoryMap
	 * Boot Services API.
	 */
	efisz = (sizeof(struct efi_map_header) + 0xf) & ~0xf;
	map = (struct efi_md *)((uint8_t *)efihdr + efisz);

	if (efihdr->descriptor_size == 0)
		return;
	ndesc = efihdr->memory_size / efihdr->descriptor_size;

	for (i = 0, p = map; i < ndesc; i++,
	    p = efi_next_descriptor(p, efihdr->descriptor_size)) {
		cb(p, argp);
	}
}

/*
 * Handle the EFI memory map list.
 *
 * We will make two passes at this, the first (exclude == false) to populate
 * physmem with valid physical memory ranges from recognized map entry types.
 * In the second pass we will exclude memory ranges from physmem which must not
 * be used for general allocations, either because they are used by runtime
 * firmware or otherwise reserved.
 *
 * Adding the runtime-reserved memory ranges to physmem and excluding them
 * later ensures that they are included in the DMAP, but excluded from
 * phys_avail[].
 *
 * Entry types not explicitly listed here are ignored and not mapped.
 */
static void
handle_efi_map_entry(struct efi_md *p, void *argp)
{
	bool exclude = *(bool *)argp;

	switch (p->md_type) {
	case EFI_MD_TYPE_RECLAIM:
		/*
		 * The recomended location for ACPI tables. Map into the
		 * DMAP so we can access them from userspace via /dev/mem.
		 */
	case EFI_MD_TYPE_RT_CODE:
		/*
		 * Some UEFI implementations put the system table in the
		 * runtime code section. Include it in the DMAP, but will
		 * be excluded from phys_avail.
		 */
	case EFI_MD_TYPE_RT_DATA:
		/*
		 * Runtime data will be excluded after the DMAP
		 * region is created to stop it from being added
		 * to phys_avail.
		 */
		if (exclude) {
			physmem_exclude_region(p->md_phys,
			    p->md_pages * EFI_PAGE_SIZE, EXFLAG_NOALLOC);
			break;
		}
		/* FALLTHROUGH */
	case EFI_MD_TYPE_CODE:
	case EFI_MD_TYPE_DATA:
	case EFI_MD_TYPE_BS_CODE:
	case EFI_MD_TYPE_BS_DATA:
	case EFI_MD_TYPE_FREE:
		/*
		 * We're allowed to use any entry with these types.
		 */
		if (!exclude)
			physmem_hardware_region(p->md_phys,
			    p->md_pages * EFI_PAGE_SIZE);
		break;
	default:
		/* Other types shall not be handled by physmem. */
		break;
	}
}

static void
add_efi_map_entries(struct efi_map_header *efihdr)
{
	bool exclude = false;
	foreach_efi_map_entry(efihdr, handle_efi_map_entry, &exclude);
}

static void
exclude_efi_map_entries(struct efi_map_header *efihdr)
{
	bool exclude = true;
	foreach_efi_map_entry(efihdr, handle_efi_map_entry, &exclude);
}

static void
print_efi_map_entry(struct efi_md *p, void *argp __unused)
{
	const char *type;
	static const char *types[] = {
		"Reserved",
		"LoaderCode",
		"LoaderData",
		"BootServicesCode",
		"BootServicesData",
		"RuntimeServicesCode",
		"RuntimeServicesData",
		"ConventionalMemory",
		"UnusableMemory",
		"ACPIReclaimMemory",
		"ACPIMemoryNVS",
		"MemoryMappedIO",
		"MemoryMappedIOPortSpace",
		"PalCode",
		"PersistentMemory"
	};

	if (p->md_type < nitems(types))
		type = types[p->md_type];
	else
		type = "<INVALID>";
	printf("%23s %012lx %012lx %08lx ", type, p->md_phys,
	    p->md_virt, p->md_pages);
	if (p->md_attr & EFI_MD_ATTR_UC)
		printf("UC ");
	if (p->md_attr & EFI_MD_ATTR_WC)
		printf("WC ");
	if (p->md_attr & EFI_MD_ATTR_WT)
		printf("WT ");
	if (p->md_attr & EFI_MD_ATTR_WB)
		printf("WB ");
	if (p->md_attr & EFI_MD_ATTR_UCE)
		printf("UCE ");
	if (p->md_attr & EFI_MD_ATTR_WP)
		printf("WP ");
	if (p->md_attr & EFI_MD_ATTR_RP)
		printf("RP ");
	if (p->md_attr & EFI_MD_ATTR_XP)
		printf("XP ");
	if (p->md_attr & EFI_MD_ATTR_NV)
		printf("NV ");
	if (p->md_attr & EFI_MD_ATTR_MORE_RELIABLE)
		printf("MORE_RELIABLE ");
	if (p->md_attr & EFI_MD_ATTR_RO)
		printf("RO ");
	if (p->md_attr & EFI_MD_ATTR_RT)
		printf("RUNTIME");
	printf("\n");
}

static void
print_efi_map_entries(struct efi_map_header *efihdr)
{

	printf("%23s %12s %12s %8s %4s\n",
	    "Type", "Physical", "Virtual", "#Pages", "Attr");
	foreach_efi_map_entry(efihdr, print_efi_map_entry, NULL);
}



/*
 * Map the passed in VA in EFI space to a void * using the efi memory table to
 * find the PA and return it in the DMAP, if it exists. We're used between the
 * calls to pmap_bootstrap() and physmem_init_kernel_globals() to parse CFG
 * tables We assume that either the entry you are mapping fits within its page,
 * or if it spills to the next page, that's contiguous in PA and in the DMAP.
 * All observed tables obey the first part of this precondition.
 */
struct early_map_data
{
	vm_offset_t va;
	vm_offset_t pa;
};

static void
efi_early_map_entry(struct efi_md *p, void *argp)
{
	struct early_map_data *emdp = argp;
	vm_offset_t s, e;

	if (emdp->pa != 0)
		return;
	if ((p->md_attr & EFI_MD_ATTR_RT) == 0)
		return;
	s = p->md_virt;
	e = p->md_virt + p->md_pages * EFI_PAGE_SIZE;
	if (emdp->va < s  || emdp->va >= e)
		return;
	emdp->pa = p->md_phys + (emdp->va - p->md_virt);
}

static void *
efi_early_map(vm_offset_t va)
{
	struct early_map_data emd = { .va = va };

	foreach_efi_map_entry(efihdr, efi_early_map_entry, &emd);
	if (emd.pa == 0)
		return NULL;
	return (void *)PHYS_TO_DMAP(emd.pa);
}


/*
 * When booted via kboot, the prior kernel will pass in reserved memory areas in
 * a EFI config table. We need to find that table and walk through it excluding
 * the memory ranges in it. btw, this is called too early for the printf to do
 * anything since msgbufp isn't initialized, let alone a console...
 */
static void
exclude_efi_memreserve(vm_offset_t efi_systbl_phys)
{
	struct efi_systbl *systbl;
	struct uuid efi_memreserve = LINUX_EFI_MEMRESERVE_TABLE;

	systbl = (struct efi_systbl *)PHYS_TO_DMAP(efi_systbl_phys);
	if (systbl == NULL) {
		printf("can't map systbl\n");
		return;
	}
	if (systbl->st_hdr.th_sig != EFI_SYSTBL_SIG) {
		printf("Bad signature for systbl %#lx\n", systbl->st_hdr.th_sig);
		return;
	}

	/*
	 * We don't yet have the pmap system booted enough to create a pmap for
	 * the efi firmware's preferred address space from the GetMemoryMap()
	 * table. The st_cfgtbl is a VA in this space, so we need to do the
	 * mapping ourselves to a kernel VA with efi_early_map. We assume that
	 * the cfgtbl entries don't span a page. Other pointers are PAs, as
	 * noted below.
	 */
	if (systbl->st_cfgtbl == 0)	/* Failsafe st_entries should == 0 in this case */
		return;
	for (int i = 0; i < systbl->st_entries; i++) {
		struct efi_cfgtbl *cfgtbl;
		struct linux_efi_memreserve *mr;

		cfgtbl = efi_early_map(systbl->st_cfgtbl + i * sizeof(*cfgtbl));
		if (cfgtbl == NULL)
			panic("Can't map the config table entry %d\n", i);
		if (memcmp(&cfgtbl->ct_uuid, &efi_memreserve, sizeof(struct uuid)) != 0)
			continue;

		/*
		 * cfgtbl points are either VA or PA, depending on the GUID of
		 * the table. memreserve GUID pointers are PA and not converted
		 * after a SetVirtualAddressMap(). The list's mr_next pointer
		 * is also a PA.
		 */
		mr = (struct linux_efi_memreserve *)PHYS_TO_DMAP(
			(vm_offset_t)cfgtbl->ct_data);
		while (true) {
			for (int j = 0; j < mr->mr_count; j++) {
				struct linux_efi_memreserve_entry *mre;

				mre = &mr->mr_entry[j];
				physmem_exclude_region(mre->mre_base, mre->mre_size,
				    EXFLAG_NODUMP | EXFLAG_NOALLOC);
			}
			if (mr->mr_next == 0)
				break;
			mr = (struct linux_efi_memreserve *)PHYS_TO_DMAP(mr->mr_next);
		};
	}

}


#ifdef FDT
static void
try_load_dtb(caddr_t kmdp)
{
	vm_offset_t dtbp;

	dtbp = MD_FETCH(kmdp, MODINFOMD_DTBP, vm_offset_t);

#if defined(FDT_DTB_STATIC)
	/*
	 * In case the device tree blob was not retrieved (from metadata) try
	 * to use the statically embedded one.
	 */
	if (dtbp == (vm_offset_t)NULL)
		dtbp = (vm_offset_t)&fdt_static_dtb;
#endif

	if (dtbp == (vm_offset_t)NULL) {
		printf("ERROR loading DTB\n");
		return;
	}

	if (OF_install(OFW_FDT, 0) == FALSE)
		panic("Cannot install FDT");

	if (OF_init((void *)dtbp) != 0)
		panic("OF_init failed with the found device tree");
}
#endif

static void
cache_setup(void)
{

	/* TODO */

	dcache_line_size = 0;
	icache_line_size = 0;
	idcache_line_size = 0;
}

/*
 * Fake up a boot descriptor table.
 */
static void
fake_preload_metadata(struct loongarch_bootparams *bp)
{
	static uint32_t fake_preload[48];
	vm_offset_t lastaddr;
	size_t fake_size, dtb_size;

#define PRELOAD_PUSH_VALUE(type, value) do {			\
	*(type *)((char *)fake_preload + fake_size) = (value);	\
	fake_size += sizeof(type);				\
} while (0)

#define PRELOAD_PUSH_STRING(str) do {				\
	uint32_t ssize;						\
	ssize = strlen(str) + 1;				\
	PRELOAD_PUSH_VALUE(uint32_t, ssize);			\
	strcpy(((char *)fake_preload + fake_size), str);	\
	fake_size += ssize;					\
	fake_size = roundup(fake_size, sizeof(u_long));		\
} while (0)

	fake_size = 0;
	lastaddr = (vm_offset_t)&end;

	PRELOAD_PUSH_VALUE(uint32_t, MODINFO_NAME);
	PRELOAD_PUSH_STRING("kernel");
	PRELOAD_PUSH_VALUE(uint32_t, MODINFO_TYPE);
	PRELOAD_PUSH_STRING("elf kernel");

	PRELOAD_PUSH_VALUE(uint32_t, MODINFO_ADDR);
	PRELOAD_PUSH_VALUE(uint32_t, sizeof(vm_offset_t));
	PRELOAD_PUSH_VALUE(uint64_t, KERNBASE);

	PRELOAD_PUSH_VALUE(uint32_t, MODINFO_SIZE);
	PRELOAD_PUSH_VALUE(uint32_t, sizeof(size_t));
	PRELOAD_PUSH_VALUE(uint64_t, (size_t)((vm_offset_t)&end - KERNBASE));

	/* Copy the DTB to KVA space. */
	lastaddr = roundup(lastaddr, sizeof(int));
	PRELOAD_PUSH_VALUE(uint32_t, MODINFO_METADATA | MODINFOMD_DTBP);
	PRELOAD_PUSH_VALUE(uint32_t, sizeof(vm_offset_t));
	PRELOAD_PUSH_VALUE(vm_offset_t, lastaddr);
	dtb_size = fdt_totalsize(bp->dtbp_virt);
	memmove((void *)lastaddr, (const void *)bp->dtbp_virt, dtb_size);
	lastaddr = roundup(lastaddr + dtb_size, sizeof(int));

	PRELOAD_PUSH_VALUE(uint32_t, MODINFO_METADATA | MODINFOMD_KERNEND);
	PRELOAD_PUSH_VALUE(uint32_t, sizeof(vm_offset_t));
	PRELOAD_PUSH_VALUE(vm_offset_t, lastaddr);

	PRELOAD_PUSH_VALUE(uint32_t, MODINFO_METADATA | MODINFOMD_HOWTO);
	PRELOAD_PUSH_VALUE(uint32_t, sizeof(int));
	PRELOAD_PUSH_VALUE(int, RB_VERBOSE);

	/* End marker */
	PRELOAD_PUSH_VALUE(uint32_t, 0);
	PRELOAD_PUSH_VALUE(uint32_t, 0);
	preload_metadata = (caddr_t)fake_preload;

	/* Check if bootloader clobbered part of the kernel with the DTB. */
	KASSERT(bp->dtbp_phys + dtb_size <= bp->kern_phys ||
		bp->dtbp_phys >= bp->kern_phys + (lastaddr - KERNBASE),
	    ("FDT (%lx-%lx) and kernel (%lx-%lx) overlap", bp->dtbp_phys,
		bp->dtbp_phys + dtb_size, bp->kern_phys,
		bp->kern_phys + (lastaddr - KERNBASE)));
	KASSERT(fake_size < sizeof(fake_preload),
	    ("Too many fake_preload items"));

	if (boothowto & RB_VERBOSE)
		printf("FDT phys (%lx-%lx), kernel phys (%lx-%lx)\n",
		    bp->dtbp_phys, bp->dtbp_phys + dtb_size,
		    bp->kern_phys, bp->kern_phys + (lastaddr - KERNBASE));
}

/* Support for FDT configurations only. */
CTASSERT(FDT);

#ifdef FDT
static void
parse_fdt_bootargs(void)
{
	char bootargs[512];

	bootargs[sizeof(bootargs) - 1] = '\0';
	if (fdt_get_chosen_bootargs(bootargs, sizeof(bootargs) - 1) == 0) {
		boothowto |= boot_parse_cmdline(bootargs);
	}
}
#endif

static vm_offset_t
parse_metadata(void)
{
	caddr_t kmdp;
	vm_offset_t lastaddr;
#ifdef DDB
	vm_offset_t ksym_start, ksym_end;
#endif
	char *kern_envp;

	/* Find the kernel address */
	kmdp = preload_search_by_type("elf kernel");
	if (kmdp == NULL)
		kmdp = preload_search_by_type("elf64 kernel");
	KASSERT(kmdp != NULL, ("No preload metadata found!"));

	/* Read the boot metadata */
	boothowto = MD_FETCH(kmdp, MODINFOMD_HOWTO, int);
	lastaddr = MD_FETCH(kmdp, MODINFOMD_KERNEND, vm_offset_t);
	kern_envp = MD_FETCH(kmdp, MODINFOMD_ENVP, char *);
	if (kern_envp != NULL)
		init_static_kenv(kern_envp, 0);
	else
		init_static_kenv(static_kenv, sizeof(static_kenv));
#ifdef DDB
	ksym_start = MD_FETCH(kmdp, MODINFOMD_SSYM, uintptr_t);
	ksym_end = MD_FETCH(kmdp, MODINFOMD_ESYM, uintptr_t);
	db_fetch_ksymtab(ksym_start, ksym_end);
#endif
#ifdef FDT
	try_load_dtb(kmdp);
	if (kern_envp == NULL)
		parse_fdt_bootargs();
#endif
	return (lastaddr);
}

void
initloongarch(struct loongarch_bootparams *bp)
{
	struct mem_region mem_regions[FDT_MEM_REGIONS];
	int mem_regions_sz;
	struct pcpu *pcpup;
	vm_offset_t lastaddr;
	vm_size_t kernlen;
	char *env;
#if 0
	struct efi_fb *efifb;
#endif
	caddr_t kmdp;

	printf("in %s\n", __func__);

	TSRAW(&thread0, TS_ENTER, __func__, NULL);

	/* Set the pcpu data, this is needed by pmap_bootstrap */
	pcpup = &__pcpu[0];
	pcpu_init(pcpup, 0, sizeof(struct pcpu));

	/* Set the pcpu pointer */
	__asm __volatile("move $r21, %0" :: "r"(pcpup));

	PCPU_SET(curthread, &thread0);

	/* Parse the boot metadata. */
	if (bp->modulep != 0) {
		preload_metadata = (caddr_t)bp->modulep;
	} else {
		fake_preload_metadata(bp);
	}
	lastaddr = parse_metadata();

	/* set boot cpu id to 0 */
	boot_hart = 0;

	kmdp = preload_search_by_type("elf kernel");
	if (kmdp == NULL)
		kmdp = preload_search_by_type("elf64 kernel");

	efi_systbl_phys = MD_FETCH(kmdp, MODINFOMD_FW_HANDLE, vm_paddr_t);

	/* Load the physical memory ranges */
	efihdr = (struct efi_map_header *)preload_search_info(kmdp,
	    MODINFO_METADATA | MODINFOMD_EFI_MAP);
	if (efihdr != NULL)
		add_efi_map_entries(efihdr);
#ifdef FDT
	else {
		/* Grab physical memory regions information from device tree. */
		if (fdt_get_mem_regions(mem_regions, &mem_regions_sz, NULL) != 0) {
			panic("Cannot get physical memory regions");
		}
		physmem_hardware_regions(mem_regions, mem_regions_sz);
	}

	/*
	 * Exclude reserved memory specified by the device tree. Typically,
	 * this contains an entry for memory used by the runtime SBI firmware.
	 */
	if (fdt_get_reserved_mem(mem_regions, &mem_regions_sz) == 0) {
		physmem_exclude_regions(mem_regions, mem_regions_sz,
		    EXFLAG_NODUMP | EXFLAG_NOALLOC);
	}
#endif

// FIXME
#if 0
	/* Exclude the EFI framebuffer from our view of physical memory. */
	efifb = (struct efi_fb *)preload_search_info(kmdp,
	    MODINFO_METADATA | MODINFOMD_EFI_FB);
	if (efifb != NULL)
		physmem_exclude_region(efifb->fb_addr, efifb->fb_size,
		    EXFLAG_NOALLOC);
#endif
	/*
	 * Identify CPU/ISA features.
	 */
	identify_cpu(0);

	/* Do basic tuning, hz etc */
	init_param1();

	cache_setup();

	/* Bootstrap enough of pmap to enter the kernel proper */
	kernlen = (lastaddr - KERNBASE);
	pmap_bootstrap(bp->kern_l1pt, bp->kern_phys, kernlen);

	/* Exclude entries needed in the DMAP region, but not phys_avail */
	if (efihdr != NULL)
		exclude_efi_map_entries(efihdr);
	/*  Do the same for reserve entries in the EFI MEMRESERVE table */
	if (efi_systbl_phys != 0)
		exclude_efi_memreserve(efi_systbl_phys);

	physmem_init_kernel_globals();

	/* Establish static device mappings */
	devmap_bootstrap(0, NULL);

	cninit();

	/*
	 * Dump the boot metadata. We have to wait for cninit() since console
	 * output is required. If it's grossly incorrect the kernel will never
	 * make it this far.
	 */
	if (getenv_is_true("debug.dump_modinfo_at_boot"))
		preload_dump();

	init_proc0(bp->kern_stack);

	msgbufinit(msgbufp, msgbufsize);
	mutex_init();
	init_param2(physmem);
	kdb_init();
#ifdef KDB
	if ((boothowto & RB_KDB) != 0)
		kdb_enter(KDB_WHY_BOOTFLAGS, "Boot flags requested debugger");
#endif

	env = kern_getenv("kernelname");
	if (env != NULL)
		strlcpy(kernelname, env, sizeof(kernelname));

	if (boothowto & RB_VERBOSE)
		physmem_print_tables();

	early_boot = 0;

	TSEXIT();
}
