/*
 * dynarec_arm.h -- minimal ARMv5TE instruction emitter for piko's PocketSNES
 * dynamic recompiler.
 *
 * Emits little-endian ARM (A32) words into a caller-provided buffer. This is
 * the assembler the recompiler will build 65816->ARM translations with; for
 * now it carries just enough encoders to (a) prove runtime codegen works on
 * the PXA255 (see S9xDynSelfTest) and (b) form the foundation for the register
 * file load/store + prologue/epilogue that block translation will need next.
 *
 * Target is plain ARMv5TE, soft-float: no VFP/NEON, integer flag math only.
 * All encoders use the AL (always) condition; conditional forms come later.
 *
 * Encodings are hand-verified against the ARM ARM (DDI 0100); each has the
 * base opcode word documented so the bit layout is checkable at a glance.
 */
#ifndef PIKO_DYNAREC_ARM_H
#define PIKO_DYNAREC_ARM_H

#include <stdint.h>

/* ARM register numbers */
enum {
	ARM_R0 = 0,  ARM_R1,  ARM_R2,  ARM_R3,  ARM_R4,  ARM_R5,  ARM_R6,  ARM_R7,
	ARM_R8,      ARM_R9,  ARM_R10, ARM_R11, ARM_R12, ARM_R13, ARM_R14, ARM_R15,
	ARM_SP = ARM_R13, ARM_LR = ARM_R14, ARM_PC = ARM_R15
};

/* condition AL (always) in bits [31:28] */
#define ARM_AL 0xE0000000u

typedef struct {
	uint32_t *base;   /* start of the code buffer            */
	uint32_t *cur;    /* next word to write                  */
	uint32_t *end;    /* one past the last writable word     */
	int       overflow;
} ArmEmit;

static inline void arm_emit_init(ArmEmit *e, void *buf, unsigned n_words)
{
	e->base = e->cur = (uint32_t *)buf;
	e->end  = e->base + n_words;
	e->overflow = 0;
}

static inline void arm_emit(ArmEmit *e, uint32_t w)
{
	if (e->cur < e->end) *e->cur++ = w;
	else                 e->overflow = 1;
}

/* words emitted so far */
static inline unsigned arm_emit_count(const ArmEmit *e)
{
	return (unsigned)(e->cur - e->base);
}

/* ---- data processing ------------------------------------------------- *
 * MOV Rd, #imm8   -- base 0xE3A00000 (I=1, opcode MOV=1101, S=0, rot=0).
 * Only the rotate-0 range (0..255) is encodable here; wider constants get a
 * MOV/ORR sequence (added when needed). */
static inline void arm_mov_imm8(ArmEmit *e, int Rd, uint32_t imm8)
{
	arm_emit(e, 0xE3A00000u | ((uint32_t)Rd << 12) | (imm8 & 0xFF));
}

/* ADD Rd, Rn, Rm, LSL #sh  -- base 0xE0800000 (register op2, LSL, shift imm). */
static inline void arm_add_reg_lsl(ArmEmit *e, int Rd, int Rn, int Rm, int sh)
{
	arm_emit(e, 0xE0800000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12)
	            | (((uint32_t)sh & 0x1F) << 7) | ((uint32_t)Rm & 0xF));
}

static inline void arm_add_reg(ArmEmit *e, int Rd, int Rn, int Rm)
{
	arm_add_reg_lsl(e, Rd, Rn, Rm, 0);
}

/* ---- load / store (immediate offset) --------------------------------- *
 * These are the primitives the recompiler will use to spill/reload the
 * 65816 register file (SRegisters) at block boundaries.
 * LDR Rd, [Rn, #off12]  -- base 0xE5900000 (P=1,U=1,B=0,W=0,L=1). off>=0.
 * STR Rd, [Rn, #off12]  -- base 0xE5800000 (L=0). */
static inline void arm_ldr_imm(ArmEmit *e, int Rd, int Rn, unsigned off12)
{
	arm_emit(e, 0xE5900000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12) | (off12 & 0xFFF));
}

static inline void arm_str_imm(ArmEmit *e, int Rd, int Rn, unsigned off12)
{
	arm_emit(e, 0xE5800000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12) | (off12 & 0xFFF));
}

/* ---- block transfer (prologue/epilogue) ------------------------------ *
 * PUSH {reglist}  == STMFD sp!, {reglist}  -- base 0xE92D0000.
 * POP  {reglist}  == LDMFD sp!, {reglist}  -- base 0xE8BD0000.
 * reglist is a 16-bit mask (bit n => Rn). */
static inline void arm_push(ArmEmit *e, uint32_t reglist)
{
	arm_emit(e, 0xE92D0000u | (reglist & 0xFFFF));
}

static inline void arm_pop(ArmEmit *e, uint32_t reglist)
{
	arm_emit(e, 0xE8BD0000u | (reglist & 0xFFFF));
}

/* ---- control flow ---------------------------------------------------- *
 * BX Rm  -- base 0xE12FFF10. (BX LR returns from a leaf block.) */
static inline void arm_bx(ArmEmit *e, int Rm)
{
	arm_emit(e, 0xE12FFF10u | ((uint32_t)Rm & 0xF));
}

static inline void arm_bx_lr(ArmEmit *e)
{
	arm_bx(e, ARM_LR);
}

#endif /* PIKO_DYNAREC_ARM_H */
