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

static void          *native_stub[256];
static struct SRegisters s_reg;
static struct SICPU      s_icpu;
static struct SCPUState  s_cpu;
static unsigned long      v_total, v_div;

void dyn_verify_init(void)
{
	static const size_t BUFSZ = 4096;
	void *buf = mmap(NULL, BUFSZ, PROT_READ | PROT_WRITE | PROT_EXEC,
	                 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ArmEmit e;
	/* opcodes we currently translate natively (no fallback needed) */
	static const uint8_t want[] = { 0x18 /*CLC*/, 0x38 /*SEC*/, 0xEA /*NOP*/ };
	unsigned i;

	memset(native_stub, 0, sizeof(native_stub));
	v_total = v_div = 0;
	if (buf == MAP_FAILED) {
		fprintf(stderr, "DYN-VERIFY: mmap failed, verify disabled\n");
		dyn_verify_on = 0;
		return;
	}
	arm_emit_init(&e, buf, BUFSZ / 4);
	for (i = 0; i < sizeof(want); i++) {
		uint8_t op[1] = { want[i] };
		void *entry = dyn_translate_ops(op, 1, &e, /*m8*/1, /*x8*/0, /*fallback*/0);
		native_stub[want[i]] = entry;   /* NULL if it didn't fit -- treated as untranslatable */
	}
	__builtin___clear_cache((char *)buf, (char *)e.cur);
	fprintf(stderr, "DYN-VERIFY: armed (CLC/SEC/NOP), checking against interpreter\n");
}

int dyn_verify_translatable(unsigned char op)
{
	return native_stub[op] != 0;
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

	((stub_fn)native_stub[op])(&cr, &ci, &cc);

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
