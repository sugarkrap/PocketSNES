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
}

/* pinned register assignment (see ABI above) */
#define TR_A     ARM_R4
#define TR_X     ARM_R5
#define TR_Y     ARM_R6
#define TR_REG   ARM_R8
#define TR_ICPU  ARM_R9
#define TR_CPU   ARM_R10
#define TR_TMP   ARM_R3   /* caller-saved scratch */

/* pushed/popped callee-saved set (+lr / +pc) */
#define TR_SAVES ((1u<<ARM_R4)|(1u<<ARM_R5)|(1u<<ARM_R6)| \
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
extern "C" void *dyn_translate_ops(const uint8_t *ops, int n, ArmEmit *e)
{
	void *entry = (void *)e->cur;
	int i;
	tr_prologue(e);
	for (i = 0; i < n; i++)
		if (!tr_emit_op(e, ops[i]))
			return 0;
	tr_epilogue(e);
	return e->overflow ? 0 : entry;
}

/* ---- offline self-test: translate CLC;SEC;CLC and run it -------------- */
typedef void (*tr_block)(struct SRegisters *, struct SICPU *, struct SCPUState *);

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
	static const uint8_t ops[3] = { 0x18, 0x38, 0x18 };  /* CLC ; SEC ; CLC */

	if (buf == MAP_FAILED) {
		fprintf(stderr, "PIKO-DYN translate test: FAIL (RWX mmap)\n");
		return 0;
	}
	arm_emit_init(&e, buf, BUFSZ / 4);
	blk = (tr_block)dyn_translate_ops(ops, 3, &e);
	if (!blk) {
		fprintf(stderr, "PIKO-DYN translate test: FAIL (translate returned NULL)\n");
		munmap(buf, BUFSZ);
		return 0;
	}
	__builtin___clear_cache((char *)buf, (char *)e.cur);

	memset(&reg, 0, sizeof(reg));
	memset(&icpu, 0, sizeof(icpu));
	memset(&cpu, 0, sizeof(cpu));
	reg.A.W = 0x1234; reg.X.W = 0x5678; reg.Y.W = 0x9ABC;
	icpu._Carry = 1;                 /* start set, so CLC/SEC/CLC must end clear */
	cpu.Cycles = 0;

	blk(&reg, &icpu, &cpu);

	if (icpu._Carry != 0)          { fprintf(stderr, "PIKO-DYN tr: _Carry=%d want 0\n", (int)icpu._Carry); ok = 0; }
	if (cpu.Cycles != 3 * ONE_CYCLE) { fprintf(stderr, "PIKO-DYN tr: Cycles=%ld want %d\n", (long)cpu.Cycles, 3 * ONE_CYCLE); ok = 0; }
	if (reg.A.W != 0x1234)         { fprintf(stderr, "PIKO-DYN tr: A=%04X want 1234\n", reg.A.W); ok = 0; }
	if (reg.X.W != 0x5678)         { fprintf(stderr, "PIKO-DYN tr: X=%04X want 5678\n", reg.X.W); ok = 0; }
	if (reg.Y.W != 0x9ABC)         { fprintf(stderr, "PIKO-DYN tr: Y=%04X want 9ABC\n", reg.Y.W); ok = 0; }

	fprintf(stderr, "PIKO-DYN translate test: %s (CLC;SEC;CLC -> _Carry=%d, "
	        "cycles=%ld, A/X/Y intact)\n", ok ? "PASS" : "FAIL",
	        (int)icpu._Carry, (long)cpu.Cycles);
	munmap(buf, BUFSZ);
	return ok;
}
