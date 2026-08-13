/*
 * dynarec_verify.cpp -- live run-both-and-diff validation. See dynarec_verify.h.
 */
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#include "snes9x.h"
#include "65c816.h"
#include "cpuexec.h"

extern "C" {
#include "dynarec_arm.h"
#include "dynarec_verify.h"
/* from dynarec_translate.cpp */
void *dyn_translate_ops(const uint8_t *ops, int n, ArmEmit *e,
                        int m8, int x8, void *const op_fn_table[256]);
}

int dyn_verify_on = 0;

typedef void (*stub_fn)(struct SRegisters *, struct SICPU *, struct SCPUState *);

/* one native stub per opcode per (M,X) mode: index = (m8?2:0)|(x8?1:0). Index
 * ops etc. are mode-dependent, so the stub must match the runtime width. */
#define VMODE(m8, x8) (((m8) ? 2 : 0) | ((x8) ? 1 : 0))

/*
 * Opcodes we claim to translate natively -- the verify set.
 *
 * This used to be derived by pre-building one stub per opcode from a 1-byte
 * array. That cannot work for anything with operands: translating LDA abs
 * against a 1-byte array reads off the end of it, and since the fast path
 * bakes the absolute address in as an immediate, one stub per opcode cannot
 * stand for an instruction whose operand differs at every site.
 *
 * So the set is declared, and the stub is translated from the LIVE
 * instruction at cpu->PC instead (see dyn_verify_after). Keep this in step
 * with tr_emit_op's switch as opcodes land.
 */
static const uint8_t v_want[] = {
	0x18 /*CLC*/, 0x38 /*SEC*/, 0xEA /*NOP*/,
	0xE8 /*INX*/, 0xC8 /*INY*/, 0xCA /*DEX*/, 0x88 /*DEY*/,
	0xAD /*LDA abs*/,
	0x10 /*BPL*/, 0xD0 /*BNE*/, 0xF0 /*BEQ*/
};
static uint8_t        v_is_native[256];
static void          *v_optab[256];
static void          *v_buf;
static size_t         v_bufsz;
static struct SRegisters s_reg;
static struct SICPU      s_icpu;
static struct SCPUState  s_cpu;
static unsigned long      v_total, v_div;

void dyn_verify_init(void)
{
	unsigned i;

	v_bufsz = 16384;
	v_buf = mmap(NULL, v_bufsz, PROT_READ | PROT_WRITE | PROT_EXEC,
	             MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	memset(v_is_native, 0, sizeof(v_is_native));
	v_total = v_div = 0;
	if (v_buf == MAP_FAILED) {
		fprintf(stderr, "DYN-VERIFY: mmap failed, verify disabled\n");
		dyn_verify_on = 0;
		v_buf = 0;
		return;
	}
	for (i = 0; i < sizeof(v_want); i++)
		v_is_native[v_want[i]] = 1;
	fprintf(stderr, "DYN-VERIFY: armed (%u opcodes, translated at the live PC)\n",
	        (unsigned)sizeof(v_want));
}

int dyn_verify_translatable(unsigned char op)
{
	return v_buf && v_is_native[op];
}

void dyn_verify_report(void)
{
	fprintf(stderr, "DYN-VERIFY: FINAL %lu checked, %lu diverged\n", v_total, v_div);
	fflush(stderr);
}

void dyn_verify_before(struct SRegisters *reg, struct SICPU *icpu, struct SCPUState *cpu)
{
	s_reg  = *reg;    /* struct copies of the pre-opcode state */
	s_icpu = *icpu;
	s_cpu  = *cpu;
}

void dyn_verify_after(unsigned char op, struct SRegisters *reg,
                      struct SICPU *icpu, struct SCPUState *cpu)
{
	/* run the generated block on a private copy of the snapshot */
	struct SRegisters cr = s_reg;
	struct SICPU      ci = s_icpu;
	struct SCPUState  cc = s_cpu;
	const char *w = 0;
	/* pick the stub for the mode this instruction actually ran in (M/X live in
	 * the packed P, not the unpacked flag fields). */
	int m8 = (s_reg.P.W & MemoryFlag) != 0;
	int x8 = (s_reg.P.W & IndexFlag)  != 0;
	ArmEmit e;
	void *stub;

	/*
	 * Skip instructions whose operand lands on the I/O handler tier.
	 *
	 * run-both-and-diff executes the instruction TWICE -- interpreter first,
	 * then the generated stub on a copy -- and that is only sound for reads
	 * without side effects. FF6's `LDA $4210` reads RDNMI, which CLEARS the
	 * NMI flag: the interpreter got 0x80, the stub re-read it and got 0x00,
	 * and the harness reported a divergence in A for a fast path that was
	 * behaving perfectly (cycles matched exactly, because the bail to the
	 * interpreter fallback worked).
	 *
	 * Verifying the handler tier is not the point anyway: on a bail the block
	 * runs the interpreter, so there is nothing of ours left to check. What
	 * matters is the RAM/ROM fast path, which is side-effect free and
	 * re-runnable. So probe the same Map the emitted code probes, in C, and
	 * only verify when the address is direct memory.
	 */
	/*
	 * Branches: verify the NOT-TAKEN case only, and that is not a weakening --
	 * it is the only case with generated code in it. The taken path of a
	 * translated branch calls the interpreter's own Op10/OpD0/OpF0, so
	 * verifying it would compare the interpreter against itself; worse, it
	 * would RUN it a second time, and the taken path reaches CPUShutdown,
	 * which on an idle loop runs APU_EXECUTE1() until the next event. That is
	 * a global side effect, not something confined to our scratch copy -- the
	 * same trap as LDA $4210 clearing RDNMI.
	 *
	 * Not-taken is also where the dangerous failure lives. If the emitted flag
	 * test says "not taken" when the interpreter branched, the block runs on
	 * into instructions the guest jumped away from; the interpreter charged
	 * MemSpeed + ONE_CYCLE and we charged MemSpeed, so the cycle diff catches
	 * it. (The opposite error -- taking the slow path when we could have
	 * continued -- is safe, merely slower, and correctly invisible here.)
	 */
	if (op == 0x10 || op == 0xD0 || op == 0xF0) {
		if ((const unsigned char *)cpu->PC != (const unsigned char *)s_cpu.PC + 2)
			return;                       /* interpreter took it: not ours to check */
	}

	if (op == 0xAD) {
		uint32_t a = (uint32_t)(((const unsigned char *)s_cpu.PC)[1]
		           | (((const unsigned char *)s_cpu.PC)[2] << 8))
		           + s_icpu.ShiftedDB;
		int blk = (a >> MEMMAP_SHIFT) & MEMMAP_MASK;
		if (Memory.Map[blk] < (uint8 *)CMemory::MAP_LAST)
			return;                       /* handler tier: not re-runnable */
		if (!m8 && a == 0x00001FFF)
			return;                       /* straddle case bails too */
	}

	/*
	 * Translate THIS instruction, from the bytes it actually has. s_cpu.PC
	 * points at the opcode byte (cpuexec.cpp snapshots before the `*CPU.PC++`
	 * dispatch), so operands follow it in guest memory exactly as the block
	 * translator would see them. Re-translating per check is slow -- an
	 * I-cache flush each time -- but this is a debug gate, and it is the only
	 * way to verify an opcode whose operand is baked into the code as an
	 * immediate.
	 */
	/*
	 * Give the translator the interpreter's opcode table for THIS mode. An
	 * opcode with a memory fast path can bail at run time -- I/O addresses go
	 * to the handler tier -- and the bail must land on the interpreter, just
	 * as it does inside a real block. Verifying a stub without it would report
	 * every LDA that touched a PPU register as a divergence.
	 */
	{
		int i;
		for (i = 0; i < 256; i++)
			v_optab[i] = (void *)s_icpu.S9xOpcodes[i].S9xOpcode;
	}
	arm_emit_init(&e, v_buf, (unsigned)(v_bufsz / 4));
	stub = dyn_translate_ops((const uint8_t *)s_cpu.PC, 1, &e, m8, x8, v_optab);
	if (!stub || e.overflow) return;
	__builtin___clear_cache((char *)v_buf, (char *)e.cur);
	((stub_fn)stub)(&cr, &ci, &cc);

	/* diff native (cr/ci/cc) against interpreter (reg/icpu/cpu). PC is excluded:
	 * opcode-byte advancement is the dispatcher's job, not the block's. */
	if      (cr.A.W       != reg->A.W)        w = "A";
	else if (cr.X.W       != reg->X.W)        w = "X";
	else if (cr.Y.W       != reg->Y.W)        w = "Y";
	else if (ci._Carry    != icpu->_Carry)    w = "C";
	else if (ci._Zero     != icpu->_Zero)     w = "Z";
	else if (ci._Negative != icpu->_Negative) w = "N";
	else if (ci._Overflow != icpu->_Overflow) w = "V";
	else if (cc.Cycles    != cpu->Cycles)     w = "cycles";

	v_total++;
	if (w) {
		v_div++;
		if (v_div <= 12)
			fprintf(stderr,
			        "DYN-VERIFY DIVERGE op=%02X field=%s | native A=%04X "
			        "interp A=%04X | pre A=%04X DB=%02X ShiftedDB=%06X "
			        "operand=%02X%02X m8=%d cyc n=%ld i=%ld\n",
			        op, w, cr.A.W, reg->A.W, s_reg.A.W, s_reg.DB,
			        (unsigned)s_icpu.ShiftedDB,
			        ((const unsigned char *)s_cpu.PC)[2],
			        ((const unsigned char *)s_cpu.PC)[1],
			        m8, (long)cc.Cycles, (long)cpu->Cycles);
	}
	if ((v_total % 250000UL) == 0)
		fprintf(stderr, "DYN-VERIFY: %lu checked, %lu diverged\n", v_total, v_div);
}
