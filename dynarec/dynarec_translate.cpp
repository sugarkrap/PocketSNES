/*
 * dynarec_translate.cpp -- 65816 -> ARMv5TE block translator (Step 3, start).
 *
 * Emits native ARM for a run of guest opcodes. This is a .cpp (not .c) so it
 * can `offsetof` the real snes9x structs and use ONE_CYCLE etc. -- the emitter
 * itself (dynarec_arm.h) stays plain C.
 *
 * BLOCK ABI (mirrors an interpreter opcode fn so fallbacks are trivial later):
 *     void block(SRegisters *reg, SICPU *icpu, SCPUState *cpu)   // r0, r1, r2
 *   pinned for the block's duration:
 *     r8 = reg    r9 = icpu    r10 = cpu
 *     r4 = A      r5 = X       r6 = Y        (loaded/spilled by pro/epilogue)
 *
 * FLAGS: snes9x does NOT keep N/Z/C/V packed in P during execution -- the live
 * flags are the separate SICPU fields _Carry/_Zero/_Negative/_Overflow (with
 * their own encodings, e.g. CLC => icpu->_Carry = 0). Native ops must write
 * those fields to stay consistent with interpreter fallbacks. (A later
 * optimisation can map them to ARM's own NZCV; for now we mirror the interp
 * exactly, in memory, for provable correctness.)
 *
 * This first step translates only CLC/SEC/NOP -- trivial, unambiguous, and CLC
 * is one of FF6's hottest opcodes (see README section 7). Everything else is
 * "unsupported" for now; the hybrid interpreter-fallback path + live wiring is
 * the next sub-step.
 */
#include <stddef.h>
#include <string.h>
#include <sys/mman.h>
#include <stdio.h>

#include "snes9x.h"
#include "65c816.h"
#include "cpuexec.h"

extern "C" {
#include "dynarec_arm.h"
#include "dynarec_block.h"   /* dyn_op_length */
}

/* pinned register assignment (see ABI above) */
#define TR_A      ARM_R4
#define TR_X      ARM_R5
#define TR_Y      ARM_R6
#define TR_PCBASE ARM_R7    /* host pointer to the block's first opcode byte */
#define TR_REG    ARM_R8
#define TR_ICPU   ARM_R9
#define TR_CPU    ARM_R10
#define TR_TMP    ARM_R3    /* caller-saved scratch */

/* pushed/popped callee-saved set (+lr / +pc). Eight registers -> the pushed
 * frame is 8-byte aligned, satisfying AAPCS at the interpreter-fallback call. */
#define TR_SAVES ((1u<<ARM_R4)|(1u<<ARM_R5)|(1u<<ARM_R6)|(1u<<ARM_R7)| \
                  (1u<<ARM_R8)|(1u<<ARM_R9)|(1u<<ARM_R10))

static void tr_prologue(ArmEmit *e)
{
	arm_push(e, TR_SAVES | (1u << ARM_LR));
	arm_mov_reg(e, TR_REG,  ARM_R0);
	arm_mov_reg(e, TR_ICPU, ARM_R1);
	arm_mov_reg(e, TR_CPU,  ARM_R2);
	arm_ldrh_imm(e, TR_A, TR_REG, offsetof(struct SRegisters, A));
	arm_ldrh_imm(e, TR_X, TR_REG, offsetof(struct SRegisters, X));
	arm_ldrh_imm(e, TR_Y, TR_REG, offsetof(struct SRegisters, Y));
	/* base host pointer for computing cpu->PC at fallback sites: the block is
	 * entered with cpu->PC pointing at its first opcode byte. */
	arm_ldr_imm(e, TR_PCBASE, TR_CPU, offsetof(struct SCPUState, PC));
}

static void tr_epilogue(ArmEmit *e)
{
	arm_strh_imm(e, TR_A, TR_REG, offsetof(struct SRegisters, A));
	arm_strh_imm(e, TR_X, TR_REG, offsetof(struct SRegisters, X));
	arm_strh_imm(e, TR_Y, TR_REG, offsetof(struct SRegisters, Y));
	arm_pop(e, TR_SAVES | (1u << ARM_PC));
}

/* icpu->_Carry = v  (v is 0 or 1) */
static void tr_set_carry(ArmEmit *e, int v)
{
	arm_mov_imm8(e, TR_TMP, (uint32_t)v);
	arm_str_imm(e, TR_TMP, TR_ICPU, offsetof(struct SICPU, _Carry));
}

/* cpu->Cycles += n */
static void tr_add_cycles(ArmEmit *e, int n)
{
	arm_ldr_imm(e, TR_TMP, TR_CPU, offsetof(struct SCPUState, Cycles));
	arm_add_imm8(e, TR_TMP, TR_TMP, (uint32_t)n);
	arm_str_imm(e, TR_TMP, TR_CPU, offsetof(struct SCPUState, Cycles));
}

/*
 * Hybrid fallback: call the interpreter's opcode fn for an opcode we don't
 * translate natively yet. `guest_off` is the opcode's byte offset from the
 * block start; `fn` is the interpreter routine (same (reg,icpu,cpu) signature).
 *
 *   spill pinned A/X/Y -> reg   (flags already live in SICPU memory)
 *   cpu->PC = pcbase + guest_off + 1   (past the opcode byte, where the fn
 *                                       expects to read its operands)
 *   r0/r1/r2 = reg/icpu/cpu ; BLX fn
 *   reload A/X/Y <- reg        (the fn may have changed them)
 *
 * reg/icpu/cpu (r8/r9/r10) and pcbase (r7) are callee-saved, so the C fn
 * preserves them across the call; only r0-r3/r12 are clobbered, all re-derived.
 */
static void tr_emit_fallback(ArmEmit *e, unsigned guest_off, void *fn)
{
	arm_strh_imm(e, TR_A, TR_REG, offsetof(struct SRegisters, A));
	arm_strh_imm(e, TR_X, TR_REG, offsetof(struct SRegisters, X));
	arm_strh_imm(e, TR_Y, TR_REG, offsetof(struct SRegisters, Y));

	arm_mov32(e, TR_TMP, guest_off + 1);
	arm_add_reg(e, TR_TMP, TR_PCBASE, TR_TMP);
	arm_str_imm(e, TR_TMP, TR_CPU, offsetof(struct SCPUState, PC));

	arm_mov_reg(e, ARM_R0, TR_REG);
	arm_mov_reg(e, ARM_R1, TR_ICPU);
	arm_mov_reg(e, ARM_R2, TR_CPU);
	arm_mov32(e, TR_TMP, (uint32_t)(uintptr_t)fn);
	arm_blx(e, TR_TMP);

	arm_ldrh_imm(e, TR_A, TR_REG, offsetof(struct SRegisters, A));
	arm_ldrh_imm(e, TR_X, TR_REG, offsetof(struct SRegisters, X));
	arm_ldrh_imm(e, TR_Y, TR_REG, offsetof(struct SRegisters, Y));
}

/* Emit one opcode. Returns 1 if translated natively, 0 if not yet supported. */
static int tr_emit_op(ArmEmit *e, uint8_t op)
{
	switch (op) {
	case 0x18:  /* CLC */
		tr_set_carry(e, 0);
		tr_add_cycles(e, ONE_CYCLE);
		return 1;
	case 0x38:  /* SEC */
		tr_set_carry(e, 1);
		tr_add_cycles(e, ONE_CYCLE);
		return 1;
	case 0xEA:  /* NOP */
		tr_add_cycles(e, ONE_CYCLE);
		return 1;
	default:
		return 0;
	}
}

/*
 * Translate a flat list of opcodes into `e`, returning the block entry pointer,
 * or NULL if any opcode isn't natively supported yet (caller falls back to the
 * interpreter for the whole block until the hybrid path exists). This early
 * form takes an explicit op list rather than walking guest memory so it is
 * unit-testable in isolation.
 */
/*
 * Translate a flat opcode list into `e`, returning the block entry pointer (or
 * NULL on overflow / unavailable fallback). Opcodes we can't emit natively are
 * routed to op_fn_table[op] (the interpreter routine); a NULL table entry means
 * "no fallback" and aborts. m8/x8 drive instruction lengths so the per-opcode
 * guest offset (used to set cpu->PC at fallbacks) is correct. This early form
 * takes an explicit op list rather than walking guest memory so it stays
 * unit-testable in isolation.
 */
extern "C" void *dyn_translate_ops(const uint8_t *ops, int n, ArmEmit *e,
                                   int m8, int x8, void *const op_fn_table[256])
{
	void *entry = (void *)e->cur;
	unsigned off = 0;
	int i;
	tr_prologue(e);
	for (i = 0; i < n; i++) {
		uint8_t op = ops[i];
		if (!tr_emit_op(e, op)) {
			void *fn = op_fn_table ? op_fn_table[op] : 0;
			if (!fn)
				return 0;   /* no native translation and no fallback */
			tr_emit_fallback(e, off, fn);
		}
		off += (unsigned)dyn_op_length(op, m8, x8);
	}
	tr_epilogue(e);
	return e->overflow ? 0 : entry;
}

/* ---- offline self-test ------------------------------------------------- */
typedef void (*tr_block)(struct SRegisters *, struct SICPU *, struct SCPUState *);

/* Stand-in "interpreter opcode" for the hybrid test: adds 0x34 to A and burns
 * a cycle (implied-style: no operands, doesn't touch PC). Same (reg,icpu,cpu)
 * signature as a real opcode fn -- it only ever sees A via reg->A because the
 * translator spills the pinned copy before calling. */
static void tr_stub_add_a(struct SRegisters *reg, struct SICPU *icpu, struct SCPUState *cpu)
{
	(void)icpu;
	reg->A.W += 0x34;
	cpu->Cycles += ONE_CYCLE;
}

extern "C" int dyn_translate_selftest(void)
{
	const size_t BUFSZ = 4096;
	void *buf = mmap(NULL, BUFSZ, PROT_READ | PROT_WRITE | PROT_EXEC,
	                 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ArmEmit e;
	tr_block blk;
	struct SRegisters reg;
	struct SICPU icpu;
	struct SCPUState cpu;
	int ok = 1;
	uint8_t codebuf[4];
	void *optab[256];
	/* CLC (native) ; 0x42 (fallback -> stub adds 0x34 to A) ; SEC (native) */
	static const uint8_t ops[3] = { 0x18, 0x42, 0x38 };

	if (buf == MAP_FAILED) {
		fprintf(stderr, "PIKO-DYN translate test: FAIL (RWX mmap)\n");
		return 0;
	}
	memset(optab, 0, sizeof(optab));
	optab[0x42] = (void *)tr_stub_add_a;

	arm_emit_init(&e, buf, BUFSZ / 4);
	blk = (tr_block)dyn_translate_ops(ops, 3, &e, /*m8*/1, /*x8*/0, optab);
	if (!blk) {
		fprintf(stderr, "PIKO-DYN translate test: FAIL (translate returned NULL)\n");
		munmap(buf, BUFSZ);
		return 0;
	}
	__builtin___clear_cache((char *)buf, (char *)e.cur);

	memset(&reg, 0, sizeof(reg));
	memset(&icpu, 0, sizeof(icpu));
	memset(&cpu, 0, sizeof(cpu));
	reg.A.W = 0x1000; reg.X.W = 0x5678; reg.Y.W = 0x9ABC;
	icpu._Carry = 0;                 /* SEC (last) must set it */
	cpu.Cycles = 0;
	cpu.PC = codebuf;                /* any valid base ptr; the stub ignores PC */

	blk(&reg, &icpu, &cpu);

	/* A: pinned 0x1000 spilled -> stub sees it, +0x34 -> reloaded -> spilled */
	if (reg.A.W != 0x1034)         { fprintf(stderr, "PIKO-DYN tr: A=%04X want 1034 (fallback spill/reload)\n", reg.A.W); ok = 0; }
	if (icpu._Carry != 1)          { fprintf(stderr, "PIKO-DYN tr: _Carry=%d want 1\n", (int)icpu._Carry); ok = 0; }
	if (cpu.Cycles != 3 * ONE_CYCLE) { fprintf(stderr, "PIKO-DYN tr: Cycles=%ld want %d\n", (long)cpu.Cycles, 3 * ONE_CYCLE); ok = 0; }
	if (reg.X.W != 0x5678)         { fprintf(stderr, "PIKO-DYN tr: X=%04X want 5678\n", reg.X.W); ok = 0; }
	if (reg.Y.W != 0x9ABC)         { fprintf(stderr, "PIKO-DYN tr: Y=%04X want 9ABC\n", reg.Y.W); ok = 0; }
	/* cpu->PC must have been set by the fallback to base + (off 1) + 1 = base+2 */
	if (cpu.PC != codebuf + 2)     { fprintf(stderr, "PIKO-DYN tr: PC off by %ld (want +2)\n", (long)(cpu.PC - codebuf)); ok = 0; }

	fprintf(stderr, "PIKO-DYN translate test: %s (hybrid CLC/[fallback]/SEC -> "
	        "A=%04X _Carry=%d cycles=%ld)\n", ok ? "PASS" : "FAIL",
	        reg.A.W, (int)icpu._Carry, (long)cpu.Cycles);
	munmap(buf, BUFSZ);
	return ok;
}
