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
#include "dynarec_block.h"
#include "dynarec_harness.h"

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

/* ---- Step 1 offline unit tests: decoder, block discovery, cache, harness.
 * Pure computation, no snes9x/emulator state -- safe to run over SSH. Prints
 * each failure; returns 1 if all pass. */
#define DYN_CHECK(cond, msg) \
	do { if (!(cond)) { fprintf(stderr, "PIKO-DYN test FAIL: %s\n", (msg)); ok = 0; } } while (0)

static int dyn_offline_tests(void)
{
	int ok = 1;

	/* --- decoder: mode-dependent immediate widths --- */
	DYN_CHECK(dyn_op_length(0xEA, 1, 1) == 1, "NOP length");             /* implied */
	DYN_CHECK(dyn_op_length(0xA9, 1, 1) == 2, "LDA# 8-bit length");      /* imm[M], M8 */
	DYN_CHECK(dyn_op_length(0xA9, 0, 1) == 3, "LDA# 16-bit length");     /* imm[M], M16 */
	DYN_CHECK(dyn_op_length(0xA2, 1, 1) == 2, "LDX# 8-bit length");      /* imm[X], X8 */
	DYN_CHECK(dyn_op_length(0xA2, 1, 0) == 3, "LDX# 16-bit length");     /* imm[X], X16 */
	DYN_CHECK(dyn_op_length(0x4C, 1, 1) == 3, "JMP abs length");
	DYN_CHECK(dyn_op_length(0xAF, 1, 1) == 4, "LDA long length");        /* absolute long */

	/* --- decoder: block-ender classification --- */
	DYN_CHECK(!dyn_op_is_block_end(0xEA), "NOP not ender");
	DYN_CHECK(!dyn_op_is_block_end(0xA9), "LDA# not ender");
	DYN_CHECK( dyn_op_is_block_end(0x4C), "JMP is ender");
	DYN_CHECK( dyn_op_is_block_end(0x60), "RTS is ender");
	DYN_CHECK( dyn_op_is_block_end(0xF0), "BEQ is ender");
	DYN_CHECK( dyn_op_is_block_end(0xC2), "REP is ender (mode change)");
	DYN_CHECK( dyn_op_is_block_end(0x20), "JSR is ender");

	/* --- block discovery: NOP; LDA #$34; NOP; JMP $8000  (M8) --- */
	{
		static const uint8_t code[] = { 0xEA, 0xA9, 0x34, 0xEA, 0x4C, 0x00, 0x80 };
		DynBlock b;
		int n = dyn_discover_block(code, 0x008000, /*m8*/1, /*x8*/1, &b);
		DYN_CHECK(n == 4,            "discover: 4 instructions");
		DYN_CHECK(b.length == 7,     "discover: 7 bytes");
		DYN_CHECK(b.end_op == 0x4C,  "discover: ends on JMP");
		DYN_CHECK(b.start_pc == 0x008000, "discover: start pc");
	}
	/* same code, 16-bit accumulator -> LDA# is 3 bytes, so JMP shifts out of a
	 * 7-byte window; discovery should read the extra immediate byte as part of
	 * LDA and re-frame the stream (proves width feeds the walker). */
	{
		static const uint8_t code[] = { 0xA9, 0x34, 0x12, 0x60 };  /* LDA #$1234 ; RTS */
		DynBlock b;
		int n = dyn_discover_block(code, 0x018000, /*m8*/0, /*x8*/1, &b);
		DYN_CHECK(n == 2,           "discover16: 2 instructions");
		DYN_CHECK(b.length == 4,    "discover16: 4 bytes (LDA#16 + RTS)");
		DYN_CHECK(b.end_op == 0x60, "discover16: ends on RTS");
	}

	/* --- block cache: insert / find / mode-keying --- */
	{
		DynBlock b, *p;
		memset(&b, 0, sizeof(b));
		b.start_pc = 0x00ABCD; b.m8 = 1; b.x8 = 0; b.n_insns = 3;
		dyn_cache_init();
		DYN_CHECK(dyn_cache_count() == 0, "cache empty after init");
		p = dyn_cache_insert(&b);
		DYN_CHECK(p != 0,                        "cache insert");
		DYN_CHECK(dyn_cache_count() == 1,        "cache count 1");
		DYN_CHECK(dyn_cache_find(0x00ABCD, 1, 0) == p, "cache find hit");
		DYN_CHECK(dyn_cache_find(0x00ABCD, 0, 0) == 0, "cache find miss (mode differs)");
		DYN_CHECK(dyn_cache_insert(&b) == p,     "cache re-insert returns same");
		DYN_CHECK(dyn_cache_count() == 1,        "cache count still 1");
	}

	/* --- harness: diff + hash --- */
	{
		DynCpuSnap a, c;
		const char *what = 0;
		memset(&a, 0, sizeof(a));
		a.A = 0x1234; a.PC = 0x8000; a.cycles = 42;
		c = a;
		DYN_CHECK(dyn_cpu_diff(&a, &c, &what) == 1, "harness: identical match");
		c.A = 0x1235;
		DYN_CHECK(dyn_cpu_diff(&a, &c, &what) == 0, "harness: detects A diff");
		DYN_CHECK(what && what[0] == 'A',           "harness: names A");
		c = a; c.cycles = 43;
		DYN_CHECK(dyn_cpu_diff(&a, &c, &what) == 0 && what[0] == 'c', "harness: detects cycles");

		DYN_CHECK(dyn_hash("hello", 5) == dyn_hash("hello", 5), "hash deterministic");
		DYN_CHECK(dyn_hash("hello", 5) != dyn_hash("hellp", 5), "hash distinguishes");
	}

	fprintf(stderr, "PIKO-DYN offline tests: %s\n", ok ? "PASS" : "FAIL");
	return ok;
}

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

	/* Step 1 offline unit tests (decoder / block discovery / cache / harness) */
	if (!dyn_offline_tests())
		ok = 0;

	return ok;
}
