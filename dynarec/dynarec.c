/*
 * dynarec.c -- runtime-codegen foundation for piko's PocketSNES dynarec.
 *
 * Right now this is deliberately tiny: allocate an executable code buffer,
 * emit two hand-verified ARMv5 functions into it, flush the I-cache and call
 * them. If this passes on the real PXA255 we know the make-or-break primitive
 * of any dynarec -- generating and running native code at runtime -- works on
 * this kernel and CPU, and the block translator can be built on top with
 * confidence. If it fails, we find out now, at 40 lines, not 4000.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>

#include "dynarec.h"
#include "dynarec_arm.h"

#ifndef MAP_ANONYMOUS
#ifdef MAP_ANON
#define MAP_ANONYMOUS MAP_ANON
#endif
#endif

/* Allocate a read/write/execute code buffer. Returns NULL on failure.
 * (No W^X games here -- this is a single-purpose retro handheld, and the
 * dynarec writes then executes the same pages. If a future kernel refuses
 * RWX we'll split into RW+RX mirrors, but the PXA255 target does not.) */
static void *dyn_code_alloc(size_t bytes)
{
	void *p = mmap(NULL, bytes, PROT_READ | PROT_WRITE | PROT_EXEC,
	               MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	return (p == MAP_FAILED) ? NULL : p;
}

static void dyn_code_free(void *p, size_t bytes)
{
	if (p) munmap(p, bytes);
}

/* Make emitted code visible to the instruction stream. On ARM the I-cache and
 * D-cache are not coherent, so freshly-written code MUST be flushed before it
 * is executed or the CPU may run stale cache lines. __builtin___clear_cache
 * lowers to the cacheflush syscall on Linux/uClibc. */
static void dyn_code_commit(void *start, void *end)
{
	__builtin___clear_cache((char *)start, (char *)end);
}

typedef int (*fn_ii_i)(int, int);   /* int f(int, int) */
typedef int (*fn_v_i)(void);        /* int g(void)     */

int S9xDynSelfTest(void)
{
	const size_t BUFSZ = 4096;
	void *buf;
	ArmEmit e;
	fn_ii_i f;
	fn_v_i  g;
	int r1, r2, ok = 1;
	unsigned f_words, g_words;

	buf = dyn_code_alloc(BUFSZ);
	if (!buf) {
		fprintf(stderr, "PIKO-JIT selftest: FAIL (RWX mmap: %s)\n", strerror(errno));
		return 0;
	}

	arm_emit_init(&e, buf, BUFSZ / 4);

	/* f(a,b) = a*2 + b   (a in r0, b in r1, result in r0) */
	f = (fn_ii_i)(void *)e.cur;
	arm_add_reg_lsl(&e, ARM_R0, ARM_R1, ARM_R0, 1);   /* r0 = r1 + (r0<<1) */
	arm_bx_lr(&e);
	f_words = arm_emit_count(&e);

	/* g() = 42 */
	g = (fn_v_i)(void *)e.cur;
	arm_mov_imm8(&e, ARM_R0, 42);
	arm_bx_lr(&e);
	g_words = arm_emit_count(&e) - f_words;

	if (e.overflow) {
		fprintf(stderr, "PIKO-JIT selftest: FAIL (code buffer overflow)\n");
		dyn_code_free(buf, BUFSZ);
		return 0;
	}

	dyn_code_commit(buf, (char *)buf + arm_emit_count(&e) * 4);

	r1 = f(3, 4);      /* expect 3*2 + 4 = 10 */
	r2 = g();          /* expect 42          */

	if (r1 != 10) { fprintf(stderr, "PIKO-JIT selftest: f(3,4)=%d, want 10\n", r1); ok = 0; }
	if (r2 != 42) { fprintf(stderr, "PIKO-JIT selftest: g()=%d, want 42\n", r2); ok = 0; }

	fprintf(stderr,
	        "PIKO-JIT selftest: %s (emitted f=%u words @%p, g=%u words; "
	        "RWX mmap + I-cache flush + call all worked)\n",
	        ok ? "PASS" : "FAIL", f_words, (void *)f, g_words);

	dyn_code_free(buf, BUFSZ);
	return ok;
}
