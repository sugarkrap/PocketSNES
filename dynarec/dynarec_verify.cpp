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
static void          *native_stub[256][4];
static struct SRegisters s_reg;
static struct SICPU      s_icpu;
static struct SCPUState  s_cpu;
static unsigned long      v_total, v_div;

void dyn_verify_init(void)
{
	static const size_t BUFSZ = 16384;
	void *buf = mmap(NULL, BUFSZ, PROT_READ | PROT_WRITE | PROT_EXEC,
	                 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ArmEmit e;
	static const uint8_t want[] = {
		0x18 /*CLC*/, 0x38 /*SEC*/, 0xEA /*NOP*/,
		0xE8 /*INX*/, 0xC8 /*INY*/, 0xCA /*DEX*/, 0x88 /*DEY*/
	};
	unsigned i, m8, x8;

	memset(native_stub, 0, sizeof(native_stub));
	v_total = v_div = 0;
	if (buf == MAP_FAILED) {
		fprintf(stderr, "DYN-VERIFY: mmap failed, verify disabled\n");
		dyn_verify_on = 0;
		return;
	}
	arm_emit_init(&e, buf, BUFSZ / 4);
	for (i = 0; i < sizeof(want); i++)
		for (m8 = 0; m8 < 2; m8++)
			for (x8 = 0; x8 < 2; x8++) {
				uint8_t op[1] = { want[i] };
				native_stub[want[i]][VMODE(m8, x8)] =
					dyn_translate_ops(op, 1, &e, (int)m8, (int)x8, 0);
			}
	__builtin___clear_cache((char *)buf, (char *)e.cur);
	fprintf(stderr, "DYN-VERIFY: armed (CLC/SEC/NOP/INX/INY/DEX/DEY x4 modes)\n");
}

int dyn_verify_translatable(unsigned char op)
{
	return native_stub[op][0] != 0;
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
	void *stub = native_stub[op][VMODE(m8, x8)];

	if (!stub) return;   /* not translated for this mode */
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
		if (v_div <= 20)
			fprintf(stderr, "DYN-VERIFY DIVERGE op=%02X field=%s\n", op, w);
	}
	if ((v_total % 250000UL) == 0)
		fprintf(stderr, "DYN-VERIFY: %lu checked, %lu diverged\n", v_total, v_div);
}
