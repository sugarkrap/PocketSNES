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

/* ADD Rd, Rn, #imm8  -- base 0xE2800000 (I=1, opcode ADD, rot=0). */
static inline void arm_add_imm8(ArmEmit *e, int Rd, int Rn, uint32_t imm8)
{
	arm_emit(e, 0xE2800000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12) | (imm8 & 0xFF));
}

/* MOV Rd, Rm  (register)  -- base 0xE1A00000. */
static inline void arm_mov_reg(ArmEmit *e, int Rd, int Rm)
{
	arm_emit(e, 0xE1A00000u | ((uint32_t)Rd << 12) | ((uint32_t)Rm & 0xF));
}

/* ORR Rd, Rn, #imm8 ROR (2*rot)  -- base 0xE3800000 (I=1, opcode ORR=1100). */
static inline void arm_orr_imm(ArmEmit *e, int Rd, int Rn, uint32_t imm8, uint32_t rot)
{
	arm_emit(e, 0xE3800000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12)
	            | ((rot & 0xF) << 8) | (imm8 & 0xFF));
}

/* Materialise an arbitrary 32-bit constant into Rd (ARMv5 has no single-insn
 * form): MOV low byte, then ORR the other three byte-lanes in with rotations
 * 12/8/4 (i.e. <<8, <<16, <<24). Always 4 instructions. */
static inline void arm_mov32(ArmEmit *e, int Rd, uint32_t v)
{
	arm_mov_imm8(e, Rd, v & 0xFF);
	arm_orr_imm(e, Rd, Rd, (v >> 8)  & 0xFF, 12);
	arm_orr_imm(e, Rd, Rd, (v >> 16) & 0xFF, 8);
	arm_orr_imm(e, Rd, Rd, (v >> 24) & 0xFF, 4);
}

/* BLX Rm  -- call through a register, setting LR (ARMv5TE). base 0xE12FFF30. */
static inline void arm_blx(ArmEmit *e, int Rm)
{
	arm_emit(e, 0xE12FFF30u | ((uint32_t)Rm & 0xF));
}

/* AND Rd, Rn, #imm8 ROR (2*rot)  -- base 0xE2000000. */
static inline void arm_and_imm(ArmEmit *e, int Rd, int Rn, uint32_t imm8, uint32_t rot)
{
	arm_emit(e, 0xE2000000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12)
	            | ((rot & 0xF) << 8) | (imm8 & 0xFF));
}

/* SUB Rd, Rn, #imm8  -- base 0xE2400000. */
static inline void arm_sub_imm8(ArmEmit *e, int Rd, int Rn, uint32_t imm8)
{
	arm_emit(e, 0xE2400000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12) | (imm8 & 0xFF));
}

/* ORR Rd, Rn, Rm  (register)  -- base 0xE1800000. */
static inline void arm_orr_reg(ArmEmit *e, int Rd, int Rn, int Rm)
{
	arm_emit(e, 0xE1800000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12) | ((uint32_t)Rm & 0xF));
}

/* CMP Rn, #imm8  (sets flags)  -- base 0xE3500000. */
static inline void arm_cmp_imm(ArmEmit *e, int Rn, uint32_t imm8)
{
	arm_emit(e, 0xE3500000u | ((uint32_t)Rn << 16) | (imm8 & 0xFF));
}

/* ARM condition codes (for conditional forms) */
#define ARM_EQ 0x0
#define ARM_NE 0x1
#define ARM_CS 0x2   /* unsigned >=  (aka HS) */
#define ARM_CC 0x3   /* unsigned <   (aka LO) */
/* NOT "ARM_AL": that name is already taken above for the same condition in its
 * encoded position, bits [31:28]. Two macros with one name and different
 * values worked only because every user happened to sit on the right side of
 * the redefinition, and it made every build print a warning that could hide a
 * real one. */
#define ARM_COND_AL 0xE

/* CMP Rn, Rm  -- register compare, base 0xE1500000. */
static inline void arm_cmp_reg(ArmEmit *e, int Rn, int Rm)
{
	arm_emit(e, 0xE1500000u | ((uint32_t)Rn << 16) | ((uint32_t)Rm & 0xF));
}

/*
 * Forward branches.
 *
 * The emitter has no label machinery, so a forward branch is emitted with a
 * placeholder offset and patched once the target is known:
 *
 *     unsigned f = arm_b_cc_fwd(e, ARM_CC);   // branch if unsigned <
 *     ... emitted code the branch skips over ...
 *     arm_patch_fwd(e, f);                    // now points here
 *
 * arm_b_cc_fwd returns the WORD INDEX of the branch it emitted; patching
 * computes the offset from that index to the current position. The ARM branch
 * offset is (target - (branch + 2)) in words, because the PC reads two
 * instructions ahead.
 */
static inline unsigned arm_b_cc_fwd(ArmEmit *e, int cc)
{
	unsigned at = arm_emit_count(e);
	arm_emit(e, ((uint32_t)cc << 28) | 0x0A000000u);
	return at;
}

static inline void arm_patch_fwd(ArmEmit *e, unsigned at)
{
	uint32_t *ins;
	int32_t off;
	if (at >= arm_emit_count(e)) { e->overflow = 1; return; }
	ins = e->base + at;
	off = (int32_t)arm_emit_count(e) - (int32_t)at - 2;
	*ins = (*ins & 0xFF000000u) | ((uint32_t)off & 0x00FFFFFFu);
}

/* LDR Rd, [Rn, Rm, LSL #sh]  -- register-indexed word load, base 0xE7900000. */
static inline void arm_ldr_reg_lsl(ArmEmit *e, int Rd, int Rn, int Rm, int sh)
{
	arm_emit(e, 0xE7900000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12)
	            | (((uint32_t)sh & 0x1F) << 7) | ((uint32_t)Rm & 0xF));
}

/* LDRB Rd, [Rn, Rm]  -- register-indexed byte load, base 0xE7D00000. */
static inline void arm_ldrb_reg(ArmEmit *e, int Rd, int Rn, int Rm)
{
	arm_emit(e, 0xE7D00000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12)
	            | ((uint32_t)Rm & 0xF));
}

/* STRB Rd, [Rn, Rm]  -- register-indexed byte store, base 0xE7C00000. */
static inline void arm_strb_reg(ArmEmit *e, int Rd, int Rn, int Rm)
{
	arm_emit(e, 0xE7C00000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12)
	            | ((uint32_t)Rm & 0xF));
}

/* MOV<cc> Rd, #imm8  -- conditional immediate move (base minus cond = 0x03A00000). */
static inline void arm_mov_imm8_cc(ArmEmit *e, int cc, int Rd, uint32_t imm8)
{
	arm_emit(e, ((uint32_t)(cc & 0xF) << 28) | 0x03A00000u | ((uint32_t)Rd << 12) | (imm8 & 0xFF));
}

/* MOV Rd, Rm, <shift> #amount  -- base 0xE1A00000. type: 0=LSL, 1=LSR. */
#define ARM_LSL 0
#define ARM_LSR 1
static inline void arm_mov_shift(ArmEmit *e, int Rd, int Rm, int type, int amount)
{
	arm_emit(e, 0xE1A00000u | ((uint32_t)Rd << 12)
	            | (((uint32_t)amount & 0x1F) << 7) | (((uint32_t)type & 3) << 5) | ((uint32_t)Rm & 0xF));
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

/* Byte load/store (immediate offset). The snes9x flag fields _Carry/_Zero/
 * _Negative/_Overflow are 1-byte (uint8_32 == unsigned char) and adjacent, so
 * they MUST be touched with byte ops -- a word STR clobbers the neighbours.
 * LDRB Rd,[Rn,#off] -- base 0xE5D00000.  STRB Rd,[Rn,#off] -- base 0xE5C00000. */
static inline void arm_ldrb_imm(ArmEmit *e, int Rd, int Rn, unsigned off12)
{
	arm_emit(e, 0xE5D00000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12) | (off12 & 0xFFF));
}

static inline void arm_strb_imm(ArmEmit *e, int Rd, int Rn, unsigned off12)
{
	arm_emit(e, 0xE5C00000u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12) | (off12 & 0xFFF));
}

/* Halfword load/store (immediate offset, 0..255) -- the 65816 register file's
 * pairs are 16-bit. LDRH zero-extends into the 32-bit ARM reg.
 * LDRH Rd,[Rn,#off]  -- base 0xE1D000B0 (P=1,U=1,I=1,W=0,L=1, SH=01).
 * STRH Rd,[Rn,#off]  -- base 0xE1C000B0 (L=0). off split into two nibbles. */
static inline void arm_ldrh_imm(ArmEmit *e, int Rd, int Rn, unsigned off8)
{
	arm_emit(e, 0xE1D000B0u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12)
	            | ((off8 & 0xF0u) << 4) | (off8 & 0x0Fu));
}

static inline void arm_strh_imm(ArmEmit *e, int Rd, int Rn, unsigned off8)
{
	arm_emit(e, 0xE1C000B0u | ((uint32_t)Rn << 16) | ((uint32_t)Rd << 12)
	            | ((off8 & 0xF0u) << 4) | (off8 & 0x0Fu));
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

/* ---- block register ABI ---------------------------------------------- *
 * A translated block is an ARM function `void block(void *regfile)`:
 *   - r0 on entry = pointer to the 65816 register file (SRegisters).
 * The hot guest registers live in fixed callee-saved ARM regs for the block's
 * duration; the prologue loads them from the struct and the epilogue spills
 * them back. Callee-saved regs we clobber (r4-r8) are preserved via push/pop.
 *
 *   r8 = regfile base pointer
 *   r4 = A   r5 = X   r6 = Y   r7 = P
 *
 * Field byte-offsets are passed in (rather than hardcoded) so this stays
 * decoupled from the real SRegisters layout and unit-testable against a local
 * struct; the translator supplies offsetof(SRegisters, ...) at Step 3. */
#define ARM_GUEST_A    ARM_R4
#define ARM_GUEST_X    ARM_R5
#define ARM_GUEST_Y    ARM_R6
#define ARM_GUEST_P    ARM_R7
#define ARM_REGFILE    ARM_R8

typedef struct {
	unsigned a, x, y, p;   /* byte offsets of the .W fields in the reg file */
} ArmGuestOffsets;

/* push {r4-r8, lr} ; r8 = r0 ; load A/X/Y/P from [r8, #off] */
static inline void arm_emit_block_prologue(ArmEmit *e, const ArmGuestOffsets *o)
{
	arm_push(e, (1u<<ARM_R4)|(1u<<ARM_R5)|(1u<<ARM_R6)|(1u<<ARM_R7)|(1u<<ARM_R8)|(1u<<ARM_LR));
	arm_mov_reg(e, ARM_REGFILE, ARM_R0);
	arm_ldrh_imm(e, ARM_GUEST_A, ARM_REGFILE, o->a);
	arm_ldrh_imm(e, ARM_GUEST_X, ARM_REGFILE, o->x);
	arm_ldrh_imm(e, ARM_GUEST_Y, ARM_REGFILE, o->y);
	arm_ldrh_imm(e, ARM_GUEST_P, ARM_REGFILE, o->p);
}

/* store A/X/Y/P back to [r8, #off] ; pop {r4-r8, pc} (returns) */
static inline void arm_emit_block_epilogue(ArmEmit *e, const ArmGuestOffsets *o)
{
	arm_strh_imm(e, ARM_GUEST_A, ARM_REGFILE, o->a);
	arm_strh_imm(e, ARM_GUEST_X, ARM_REGFILE, o->x);
	arm_strh_imm(e, ARM_GUEST_Y, ARM_REGFILE, o->y);
	arm_strh_imm(e, ARM_GUEST_P, ARM_REGFILE, o->p);
	arm_pop(e, (1u<<ARM_R4)|(1u<<ARM_R5)|(1u<<ARM_R6)|(1u<<ARM_R7)|(1u<<ARM_R8)|(1u<<ARM_PC));
}

#endif /* PIKO_DYNAREC_ARM_H */
