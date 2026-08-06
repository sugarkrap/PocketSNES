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
#define TR_TMP2   ARM_R12   /* caller-saved scratch (ip) */

/* pushed/popped callee-saved set (+lr / +pc). Eight registers -> the pushed
 * frame is 8-byte aligned, satisfying AAPCS at the interpreter-fallback call. */
#define TR_SAVES ((1u<<ARM_R4)|(1u<<ARM_R5)|(1u<<ARM_R6)|(1u<<ARM_R7)| \
                  (1u<<ARM_R8)|(1u<<ARM_R9)|(1u<<ARM_R10))

/*
 * Which of the pinned A/X/Y a block actually uses natively.
 *
 * Blocks average 2.66 instructions, so caching all three guest registers in
 * ARM registers was pure loss: three loads in the prologue, three stores in
 * the epilogue, and -- the expensive part -- three stores plus three loads
 * bracketing EVERY interpreter fallback, of which there are ~1.5 per block.
 * A register no native op in this block touches does not need to be cached
 * at all; the interpreter's own copy in SRegisters stays authoritative.
 *
 * The mask MUST agree with what tr_emit_op/tr_emit_lda_abs actually emit.
 * Claiming fewer registers than are used means a fallback stops spilling one
 * the native code is caching, and the block and the interpreter silently
 * disagree about the machine state -- a corruption bug, not a slow path.
 */
#define TR_USE_A 1
#define TR_USE_X 2
#define TR_USE_Y 4

/*
 * -1 if `op` is not emitted natively, otherwise the pinned registers it uses.
 * ONE function answers both questions so the two cannot drift apart: an op
 * listed as native but omitted here would be emitted with its register
 * unpinned, and the fallback bracketing would stop spilling a register the
 * native code is caching. Pass 2 re-checks the dangerous direction.
 *
 * Keep in step with tr_emit_op's switch (and the 0xAD case in
 * dyn_translate_run, which is native only when a fallback table exists for
 * its run-time bail).
 */
static int tr_op_native_mask(uint8_t op)
{
	switch (op) {
	case 0x18: case 0x38: case 0xEA: return 0;          /* CLC/SEC/NOP */
	case 0xE8: case 0xCA:            return TR_USE_X;   /* INX/DEX     */
	case 0xC8: case 0x88:            return TR_USE_Y;   /* INY/DEY     */
	case 0xAD:                       return TR_USE_A;   /* LDA abs     */
	default:                         return -1;
	}
}

static void tr_prologue(ArmEmit *e, unsigned mask)
{
	arm_push(e, TR_SAVES | (1u << ARM_LR));
	arm_mov_reg(e, TR_REG,  ARM_R0);
	arm_mov_reg(e, TR_ICPU, ARM_R1);
	arm_mov_reg(e, TR_CPU,  ARM_R2);
	if (mask & TR_USE_A) arm_ldrh_imm(e, TR_A, TR_REG, offsetof(struct SRegisters, A));
	if (mask & TR_USE_X) arm_ldrh_imm(e, TR_X, TR_REG, offsetof(struct SRegisters, X));
	if (mask & TR_USE_Y) arm_ldrh_imm(e, TR_Y, TR_REG, offsetof(struct SRegisters, Y));
	/* base host pointer for computing cpu->PC at fallback sites: the block is
	 * entered with cpu->PC pointing at its first opcode byte. */
	arm_ldr_imm(e, TR_PCBASE, TR_CPU, offsetof(struct SCPUState, PC));
}

static void tr_epilogue(ArmEmit *e, unsigned mask)
{
	if (mask & TR_USE_A) arm_strh_imm(e, TR_A, TR_REG, offsetof(struct SRegisters, A));
	if (mask & TR_USE_X) arm_strh_imm(e, TR_X, TR_REG, offsetof(struct SRegisters, X));
	if (mask & TR_USE_Y) arm_strh_imm(e, TR_Y, TR_REG, offsetof(struct SRegisters, Y));
	arm_pop(e, TR_SAVES | (1u << ARM_PC));
}

/* icpu->_Carry = v  (v is 0 or 1). BYTE store: the flag fields are 1-byte and
 * adjacent (uint8_32 == unsigned char); a word store would clobber _Zero etc. */
static void tr_set_carry(ArmEmit *e, int v)
{
	arm_mov_imm8(e, TR_TMP, (uint32_t)v);
	arm_strb_imm(e, TR_TMP, TR_ICPU, offsetof(struct SICPU, _Carry));
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
static void tr_emit_fallback(ArmEmit *e, unsigned guest_off, void *fn, unsigned mask)
{
	/* Only the registers this block actually caches. An unpinned register is
	 * already authoritative in SRegisters, so spilling it would write a stale
	 * value over the interpreter's own. */
	if (mask & TR_USE_A) arm_strh_imm(e, TR_A, TR_REG, offsetof(struct SRegisters, A));
	if (mask & TR_USE_X) arm_strh_imm(e, TR_X, TR_REG, offsetof(struct SRegisters, X));
	if (mask & TR_USE_Y) arm_strh_imm(e, TR_Y, TR_REG, offsetof(struct SRegisters, Y));

	arm_mov32(e, TR_TMP, guest_off + 1);
	arm_add_reg(e, TR_TMP, TR_PCBASE, TR_TMP);
	arm_str_imm(e, TR_TMP, TR_CPU, offsetof(struct SCPUState, PC));

	arm_mov_reg(e, ARM_R0, TR_REG);
	arm_mov_reg(e, ARM_R1, TR_ICPU);
	arm_mov_reg(e, ARM_R2, TR_CPU);
	arm_mov32(e, TR_TMP, (uint32_t)(uintptr_t)fn);
	arm_blx(e, TR_TMP);

	if (mask & TR_USE_A) arm_ldrh_imm(e, TR_A, TR_REG, offsetof(struct SRegisters, A));
	if (mask & TR_USE_X) arm_ldrh_imm(e, TR_X, TR_REG, offsetof(struct SRegisters, X));
	if (mask & TR_USE_Y) arm_ldrh_imm(e, TR_Y, TR_REG, offsetof(struct SRegisters, Y));
}

/* cpu->WaitAddress = NULL  (CPU_SHUTDOWN idle-loop tracking, cleared by ops
 * that make progress -- INX/DEX/... set it so the interpreter and native paths
 * agree on shutdown state). */
static void tr_clear_wait(ArmEmit *e)
{
	arm_mov_imm8(e, TR_TMP, 0);
	arm_str_imm(e, TR_TMP, TR_CPU, offsetof(struct SCPUState, WaitAddress));
}

/* SETZN over an 8-bit value already in Rb's low byte (SETZN8: _Zero=_Negative=v) */
static void tr_setzn8(ArmEmit *e, int Rb)
{
	arm_strb_imm(e, Rb, TR_ICPU, offsetof(struct SICPU, _Zero));
	arm_strb_imm(e, Rb, TR_ICPU, offsetof(struct SICPU, _Negative));
}

/* SETZN over the 16-bit value in Rw (assumed already masked to 16 bits):
 * _Zero = (W != 0) ? 1 : 0 ; _Negative = high byte. */
static void tr_setzn16(ArmEmit *e, int Rw)
{
	arm_cmp_imm(e, Rw, 0);
	arm_mov_imm8_cc(e, ARM_EQ, TR_TMP, 0);
	arm_mov_imm8_cc(e, ARM_NE, TR_TMP, 1);
	arm_strb_imm(e, TR_TMP, TR_ICPU, offsetof(struct SICPU, _Zero));
	arm_mov_shift(e, TR_TMP, Rw, ARM_LSR, 8);
	arm_strb_imm(e, TR_TMP, TR_ICPU, offsetof(struct SICPU, _Negative));
}

/* INX/INY/DEX/DEY on pinned index register `rreg`. x8 selects width, matching
 * the interpreter's OpE8X1/X0 etc.: 8-bit touches only the low byte (SETZN8),
 * 16-bit the whole register (SETZN16), both clear WaitAddress and add a cycle. */
static void tr_index_incdec(ArmEmit *e, int rreg, int is_dec, int x8)
{
	tr_clear_wait(e);
	if (x8) {
		/* low byte only; high byte preserved */
		if (is_dec) arm_sub_imm8(e, TR_TMP, rreg, 1);
		else        arm_add_imm8(e, TR_TMP, rreg, 1);
		arm_and_imm(e, TR_TMP, TR_TMP, 0xFF, 0);   /* new low byte */
		arm_and_imm(e, rreg, rreg, 0xFF, 12);      /* keep 0xFF00 (high byte) */
		arm_orr_reg(e, rreg, rreg, TR_TMP);
		tr_setzn8(e, TR_TMP);
	} else {
		if (is_dec) arm_sub_imm8(e, rreg, rreg, 1);
		else        arm_add_imm8(e, rreg, rreg, 1);
		arm_mov_shift(e, rreg, rreg, ARM_LSL, 16);  /* truncate to 16 bits */
		arm_mov_shift(e, rreg, rreg, ARM_LSR, 16);
		tr_setzn16(e, rreg);
	}
	tr_add_cycles(e, ONE_CYCLE);
}


/*
 * Inlined S9xGetByte/S9xGetWord fast path, split into probe and commit.
 *
 * THE SPLIT MATTERS. The MAP_LAST test must happen before any Cycles update:
 * if the address turns out to be a handler we bail to the interpreter
 * fallback, and that fallback runs Absolute()/LDA8() itself and charges
 * MemSpeedx2 + MemorySpeed again. Anything charged before the bail is counted
 * twice, and VERIFY diffs cycles.
 *
 * probe:  block = (addr >> 12) & 0xFFF ; p = Memory.Map[block]
 *         bail if p < MAP_LAST                       [no side effects yet]
 * commit: Cycles += MemorySpeed[block] (<<1 for a word -- S9xGetWord does ONE
 *         lookup and charges double, it is not two byte accesses)
 *         if (BlockIsRAM[block]) WaitAddress = PCAtOpcodeStart
 *         load 1 or 2 bytes off p + (addr & 0xFFFF)
 *
 * Memory is a global, so Memory.Map / MemorySpeed / BlockIsRAM are link-time
 * constants and go in as immediates. Rp/Rblock are r1/r2; TR_TMP/TR_TMP2 are
 * clobbered.
 */
static unsigned tr_probe_map(ArmEmit *e, int Raddr, int Rblock, int Rp)
{
	arm_mov_shift(e, Rblock, Raddr, ARM_LSL, 8);      /* (addr >> 12) & 0xFFF */
	arm_mov_shift(e, Rblock, Rblock, ARM_LSR, 20);
	arm_mov32(e, TR_TMP, (uint32_t)(uintptr_t)Memory.Map);
	arm_ldr_reg_lsl(e, Rp, TR_TMP, Rblock, 2);
	arm_mov32(e, TR_TMP, (uint32_t)CMemory::MAP_LAST);
	arm_cmp_reg(e, Rp, TR_TMP);
	return arm_b_cc_fwd(e, ARM_CC);                   /* p < MAP_LAST -> handler */
}

static void tr_commit_load(ArmEmit *e, int Raddr, int Rblock, int Rp,
                           int Rlo, int Rhi, int is_word)
{
	unsigned skip;

	arm_mov32(e, TR_TMP, (uint32_t)(uintptr_t)Memory.MemorySpeed);
	arm_ldrb_reg(e, TR_TMP, TR_TMP, Rblock);
	if (is_word)
		arm_mov_shift(e, TR_TMP, TR_TMP, ARM_LSL, 1);   /* MemorySpeed << 1 */
	arm_ldr_imm(e, TR_TMP2, TR_CPU, offsetof(struct SCPUState, Cycles));
	arm_add_reg(e, TR_TMP2, TR_TMP2, TR_TMP);
	arm_str_imm(e, TR_TMP2, TR_CPU, offsetof(struct SCPUState, Cycles));

	arm_mov32(e, TR_TMP, (uint32_t)(uintptr_t)Memory.BlockIsRAM);
	arm_ldrb_reg(e, TR_TMP, TR_TMP, Rblock);
	arm_cmp_imm(e, TR_TMP, 0);
	skip = arm_b_cc_fwd(e, ARM_EQ);
	arm_ldr_imm(e, TR_TMP, TR_CPU, offsetof(struct SCPUState, PCAtOpcodeStart));
	arm_str_imm(e, TR_TMP, TR_CPU, offsetof(struct SCPUState, WaitAddress));
	arm_patch_fwd(e, skip);

	arm_mov_shift(e, TR_TMP, Raddr, ARM_LSL, 16);      /* addr & 0xFFFF */
	arm_mov_shift(e, TR_TMP, TR_TMP, ARM_LSR, 16);
	arm_add_reg(e, TR_TMP, Rp, TR_TMP);                /* p + off */
	arm_ldrb_imm(e, Rlo, TR_TMP, 0);
	if (is_word)
		arm_ldrb_imm(e, Rhi, TR_TMP, 1);               /* FAST_LSB_WORD_ACCESS
		                                                * is #undef'd: two
		                                                * LDRBs, never an
		                                                * unaligned LDRH */
}

/*
 * LDA absolute ($AD). The operand is a COMPILE-TIME CONSTANT -- we translate a
 * known PC -- so the 16-bit address goes in as an immediate and only the bank
 * (icpu->ShiftedDB) is read at run time. That removes the operand fetch, not
 * just its dispatch cost.
 *
 * Emits: native path, a branch over the fallback, then the bail landing pad.
 * Returns the word index of that branch for the caller to patch AFTER it has
 * emitted the interpreter fallback; the bail lands on the fallback.
 */
static unsigned tr_emit_lda_abs(ArmEmit *e, const uint8_t *code, unsigned off, int m8)
{
	uint32_t imm16 = (uint32_t)code[off + 1] | ((uint32_t)code[off + 2] << 8);
	unsigned bail, straddle = 0, done;

	/* PCAtOpcodeStart = pcbase + off -- the CPU_SHUTDOWN path reads it and
	 * cpuexec.cpp sets it per opcode. */
	arm_mov32(e, TR_TMP, off);
	arm_add_reg(e, TR_TMP, TR_PCBASE, TR_TMP);
	arm_str_imm(e, TR_TMP, TR_CPU, offsetof(struct SCPUState, PCAtOpcodeStart));

	/* addr = imm16 + icpu->ShiftedDB */
	arm_mov32(e, ARM_R0, imm16);
	arm_ldr_imm(e, TR_TMP, TR_ICPU, offsetof(struct SICPU, ShiftedDB));
	arm_add_reg(e, ARM_R0, ARM_R0, TR_TMP);

	if (!m8) {
		/* S9xGetWord punts on $00001FFF, where the word straddles a Map block
		 * edge at the top of the WRAM mirror; so do we. */
		arm_mov32(e, TR_TMP, 0x00001FFFu);
		arm_cmp_reg(e, ARM_R0, TR_TMP);
		straddle = arm_b_cc_fwd(e, ARM_EQ);
	}

	bail = tr_probe_map(e, ARM_R0, ARM_R2, ARM_R1);

	/* Absolute()'s own operand fetch, charged only once we know we are keeping
	 * the native path. */
	arm_ldr_imm(e, TR_TMP, TR_CPU, offsetof(struct SCPUState, MemSpeedx2));
	arm_ldr_imm(e, TR_TMP2, TR_CPU, offsetof(struct SCPUState, Cycles));
	arm_add_reg(e, TR_TMP2, TR_TMP2, TR_TMP);
	arm_str_imm(e, TR_TMP2, TR_CPU, offsetof(struct SCPUState, Cycles));

	tr_commit_load(e, ARM_R0, ARM_R2, ARM_R1, ARM_R1, ARM_R2, !m8);

	if (m8) {
		arm_and_imm(e, TR_A, TR_A, 0xFF, 12);      /* keep A's high byte */
		arm_orr_reg(e, TR_A, TR_A, ARM_R1);
		tr_setzn8(e, ARM_R1);
	} else {
		arm_mov_shift(e, ARM_R2, ARM_R2, ARM_LSL, 8);
		arm_orr_reg(e, TR_A, ARM_R1, ARM_R2);
		tr_setzn16(e, TR_A);
	}

	done = arm_b_cc_fwd(e, ARM_COND_AL);
	arm_patch_fwd(e, bail);
	if (!m8)
		arm_patch_fwd(e, straddle);
	return done;
}

/* Emit one opcode. Returns 1 if translated natively, 0 if not yet supported.
 * m8/x8 = current accumulator/index width (blocks are mode-specific). */
static int tr_emit_op(ArmEmit *e, uint8_t op, int m8, int x8)
{
	(void)m8;
	switch (op) {
	case 0x18:  /* CLC */ tr_set_carry(e, 0); tr_add_cycles(e, ONE_CYCLE); return 1;
	case 0x38:  /* SEC */ tr_set_carry(e, 1); tr_add_cycles(e, ONE_CYCLE); return 1;
	case 0xEA:  /* NOP */ tr_add_cycles(e, ONE_CYCLE); return 1;
	case 0xE8:  /* INX */ tr_index_incdec(e, TR_X, 0, x8); return 1;
	case 0xC8:  /* INY */ tr_index_incdec(e, TR_Y, 0, x8); return 1;
	case 0xCA:  /* DEX */ tr_index_incdec(e, TR_X, 1, x8); return 1;
	case 0x88:  /* DEY */ tr_index_incdec(e, TR_Y, 1, x8); return 1;
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
/* Per-instruction fetch cost: cpu->Cycles += cpu->MemSpeed, exactly as the
 * interpreter main loop does before each opcode. Emitted before every
 * instruction in a live block so cycle counts match the interpreter. */
static void tr_add_memspeed(ArmEmit *e)
{
	arm_ldr_imm(e, TR_TMP,  TR_CPU, offsetof(struct SCPUState, MemSpeed));
	arm_ldr_imm(e, TR_TMP2, TR_CPU, offsetof(struct SCPUState, Cycles));
	arm_add_reg(e, TR_TMP2, TR_TMP2, TR_TMP);
	arm_str_imm(e, TR_TMP2, TR_CPU, offsetof(struct SCPUState, Cycles));
}

/*
 * Translate a real straight-line guest block: walk `code` (a host pointer at
 * the block's first opcode, i.e. cpu->PC) under mode (m8,x8), emitting each
 * instruction until the first block-ender. Every instruction gets the
 * per-fetch MemSpeed cycle add, then either native code or an interpreter
 * fallback (op_fn_table[op], required -- the block-ender is always a fallback
 * since control-flow ops aren't translated yet, and it updates cpu->PC).
 * Returns the block entry pointer, or NULL if an opcode has no fallback, the
 * block runs past the safety cap without an ender, or the buffer overflows.
 */
/* Set by dyn_translate_run: instructions emitted natively vs left to the
 * interpreter in the block just translated. The block is a fixed straight-line
 * run, so these are constant per block and the exec cache can multiply them by
 * the execution count instead of counting per instruction at run time. */
unsigned dyn_tr_native, dyn_tr_fallback;

/* Blocks declined because translating them could only lose. Reported by
 * DYN-EXEC so the decision is visible rather than assumed. */
unsigned long dyn_tr_declined_nonative;

/* Set when the most recent dyn_translate_run declined for lack of any native
 * instruction -- a permanent property of those guest bytes, so the caller can
 * cache the refusal. The other ways to return 0 (emitter overflow, a missing
 * fallback) are not stable and must NOT be cached. */
int dyn_tr_last_declined;

extern "C" void *dyn_translate_run(const uint8_t *code, int m8, int x8,
                                   void *const op_fn_table[256], ArmEmit *e)
{
	void *entry = (void *)e->cur;
	unsigned off = 0;
	unsigned mask = 0, n_native = 0;

	dyn_tr_last_declined = 0;

	/*
	 * PASS 1: decide what this block needs before emitting its prologue.
	 *
	 * Two things come out of it. Which of A/X/Y to pin -- the prologue has to
	 * know, and it is emitted first. And whether translating is worth doing at
	 * all: a block with no native instructions is a prologue and an epilogue
	 * wrapped around a chain of interpreter calls, which is strictly more work
	 * than letting the interpreter dispatch them itself. Measured: the dynarec
	 * ran ~20% slower than the interpreter, and blocks average 2.66
	 * instructions, so this overhead is not a rounding error.
	 */
	for (;;) {
		uint8_t op = code[off];
		int ender = dyn_op_is_block_end(op);
		int nm    = tr_op_native_mask(op);
		int native = (op == 0xAD) ? (op_fn_table[op] != 0) : (nm >= 0);
		if (native) { n_native++; if (nm >= 0) mask |= (unsigned)nm; }
		off += (unsigned)dyn_op_length(op, m8, x8);
		if (ender) break;
		if (off >= DYN_BLOCK_MAX_BYTES)
			return 0;                  /* no ender in range: let the interpreter run it */
	}
	if (n_native == 0) {
		dyn_tr_declined_nonative++;
		dyn_tr_last_declined = 1;
		return 0;
	}

	/* PASS 2: emit. Walks identically -- same lengths, same ender test -- so
	 * the two passes cannot disagree about where the block stops. */
	off = 0;
	dyn_tr_native = dyn_tr_fallback = 0;
	tr_prologue(e, mask);
	for (;;) {
		uint8_t op = code[off];
		int ender = dyn_op_is_block_end(op);
		tr_add_memspeed(e);
		if (op == 0xAD && op_fn_table[op]) {
			/* Native fast path, then the interpreter fallback the run-time
			 * bail branches into. Verified on hardware: 0 divergences. */
			unsigned done = tr_emit_lda_abs(e, code, off, m8);
			tr_emit_fallback(e, off, op_fn_table[op], mask);
			arm_patch_fwd(e, done);
			dyn_tr_native++;           /* fast path; may bail at run time */
		} else if (!tr_emit_op(e, op, m8, x8)) {
			void *fn = op_fn_table[op];
			if (!fn)
				return 0;              /* no native + no fallback */
			tr_emit_fallback(e, off, fn, mask);
			dyn_tr_fallback++;
		} else {
			/* Emitted natively but pass 1 did not declare it: the tables have
			 * drifted, and `mask` is missing whatever register this op caches.
			 * Refuse the block rather than emit one that spills the wrong set. */
			if (tr_op_native_mask(op) < 0)
				return 0;
			dyn_tr_native++;
		}
		off += (unsigned)dyn_op_length(op, m8, x8);
		if (ender)
			break;
		if (off >= DYN_BLOCK_MAX_BYTES)
			return 0;
	}
	tr_epilogue(e, mask);
	return e->overflow ? 0 : entry;
}

extern "C" void *dyn_translate_ops(const uint8_t *ops, int n, ArmEmit *e,
                                   int m8, int x8, void *const op_fn_table[256])
{
	void *entry = (void *)e->cur;
	unsigned off = 0;
	unsigned mask = 0;
	int i;

	/*
	 * Same pinning decision as dyn_translate_run, for the same reason VERIFY
	 * exists at all: the stub has to be the code a real block would emit for
	 * these opcodes. Pinning all three here instead would verify a spill
	 * pattern the exec path never generates.
	 */
	for (i = 0; i < n; i++) {
		int nm = tr_op_native_mask(ops[i]);
		if (nm >= 0) mask |= (unsigned)nm;
	}
	tr_prologue(e, mask);
	for (i = 0; i < n; i++) {
		uint8_t op = ops[i];
		if (op == 0xAD) {
			/* The caller MUST supply an opcode table: the fast path can bail
			 * at run time (I/O addresses), and the bail has to land on the
			 * interpreter. Without one there is nowhere to go. */
			void *fn = op_fn_table ? op_fn_table[op] : 0;
			unsigned done;
			if (!fn)
				return 0;
			done = tr_emit_lda_abs(e, ops, off, m8);
			tr_emit_fallback(e, off, fn, mask);
			arm_patch_fwd(e, done);
		} else if (!tr_emit_op(e, op, m8, x8)) {
			void *fn = op_fn_table ? op_fn_table[op] : 0;
			if (!fn)
				return 0;   /* no native translation and no fallback */
			tr_emit_fallback(e, off, fn, mask);
		}
		off += (unsigned)dyn_op_length(op, m8, x8);
	}
	tr_epilogue(e, mask);
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

/* Translate a single index op for mode x8, run it with X=xin, and read back
 * the resulting X.W plus the _Zero/_Negative flag bytes. Returns 0 on failure. */
static int tr_run_index(uint8_t op, int x8, uint16_t xin,
                        uint16_t *xout, uint8_t *zout, uint8_t *nout)
{
	unsigned char code[64];
	ArmEmit e;
	tr_block blk;
	struct SRegisters reg;
	struct SICPU icpu;
	struct SCPUState cpu;
	uint8_t ops[1] = { op };
	void *rwx = mmap(NULL, 4096, PROT_READ | PROT_WRITE | PROT_EXEC,
	                 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (rwx == MAP_FAILED) return 0;
	arm_emit_init(&e, rwx, 1024);
	blk = (tr_block)dyn_translate_ops(ops, 1, &e, /*m8*/1, x8, 0);
	if (!blk) { munmap(rwx, 4096); return 0; }
	__builtin___clear_cache((char *)rwx, (char *)e.cur);

	memset(&reg, 0, sizeof(reg));
	memset(&icpu, 0, sizeof(icpu));
	memset(&cpu, 0, sizeof(cpu));
	reg.X.W = xin; reg.Y.W = xin;   /* Y=X so INY/DEY hit the same cases */
	cpu.PC = code;
	blk(&reg, &icpu, &cpu);

	/* INX/DEX act on X, INY/DEY on Y -- read whichever the op targets */
	*xout = (op == 0xC8 || op == 0x88) ? reg.Y.W : reg.X.W;
	*zout = icpu._Zero;
	*nout = icpu._Negative;
	munmap(rwx, 4096);
	return 1;
}

/* Offline INX/INY/DEX/DEY check against hand-derived expected values (covers
 * 16-bit wrap + Z/N, and 8-bit low-byte-only + high-byte preservation). */
static int dyn_index_selftest(void)
{
	int ok = 1;
	uint16_t xo; uint8_t zo, no;
	struct { uint8_t op; int x8; uint16_t xin, xexp; uint8_t zexp, nexp; } c[] = {
		/* 16-bit */
		{ 0xE8, 0, 0x1234, 0x1235, 1, 0x12 },  /* INX: nonzero, N=high byte */
		{ 0xE8, 0, 0xFFFF, 0x0000, 0, 0x00 },  /* INX wrap to 0: Z set */
		{ 0xE8, 0, 0x00FF, 0x0100, 1, 0x01 },
		{ 0xCA, 0, 0x0001, 0x0000, 0, 0x00 },  /* DEX to 0: Z set */
		{ 0xCA, 0, 0x0000, 0xFFFF, 1, 0xFF },  /* DEX wrap: N set (0xFF) */
		{ 0xC8, 0, 0x8000, 0x8001, 1, 0x80 },  /* INY: N from high byte */
		{ 0x88, 0, 0x0100, 0x00FF, 1, 0x00 },  /* DEY */
		/* 8-bit: only low byte changes, high preserved; SETZN8 on low byte */
		{ 0xE8, 1, 0x12FF, 0x1200, 0x00, 0x00 },  /* XL wraps 0xFF->0x00, Z set */
		{ 0xE8, 1, 0x127F, 0x1280, 0x80, 0x80 },  /* XL=0x80, N set */
		{ 0xCA, 1, 0x1200, 0x12FF, 0xFF, 0xFF },  /* XL 0x00->0xFF */
	};
	unsigned i;
	for (i = 0; i < sizeof(c) / sizeof(c[0]); i++) {
		if (!tr_run_index(c[i].op, c[i].x8, c[i].xin, &xo, &zo, &no)) {
			fprintf(stderr, "PIKO-DYN idx: translate/run failed op=%02X\n", c[i].op); ok = 0; continue;
		}
		if (xo != c[i].xexp || zo != c[i].zexp || no != c[i].nexp) {
			fprintf(stderr, "PIKO-DYN idx FAIL op=%02X x8=%d in=%04X: got X=%04X Z=%02X N=%02X want X=%04X Z=%02X N=%02X\n",
			        c[i].op, c[i].x8, c[i].xin, xo, zo, no, c[i].xexp, c[i].zexp, c[i].nexp);
			ok = 0;
		}
	}
	fprintf(stderr, "PIKO-DYN index test: %s\n", ok ? "PASS" : "FAIL");
	return ok;
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

	if (!dyn_index_selftest())
		ok = 0;
	return ok;
}
