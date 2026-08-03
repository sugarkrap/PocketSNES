/*
 * dynarec_verify.h -- live run-both-and-diff validation for the dynarec.
 *
 * Compiled only in `make VERIFY=1` builds (-DPIKO_DYNAREC_VERIFY), runtime-gated
 * by PIKO_DYN_VERIFY. For each translatable opcode encountered in the real CPU
 * loop, the generated native version is run on a COPY of the CPU state and
 * diffed against the interpreter's effect on the real state. The interpreter
 * remains authoritative -- nothing about emulator behaviour or timing changes,
 * and the generated code only ever writes to the scratch copy, so a bug in it
 * cannot corrupt the running game. This is the safety gate that must be green
 * before generated blocks are trusted to actually drive execution.
 */
#ifndef PIKO_DYNAREC_VERIFY_H
#define PIKO_DYNAREC_VERIFY_H

#ifdef __cplusplus
extern "C" {
#endif

struct SRegisters;
struct SICPU;
struct SCPUState;

extern int dyn_verify_on;

/* Pre-translate the supported opcodes into single-op native stubs. */
void dyn_verify_init(void);

/* Does a native stub exist for this opcode? */
int  dyn_verify_translatable(unsigned char op);

/* Snapshot the live state just before the interpreter runs the opcode. */
void dyn_verify_before(struct SRegisters *reg, struct SICPU *icpu, struct SCPUState *cpu);

/* Run the native stub on the snapshot and diff against the (now interpreter-
 * updated) live state; count matches/divergences, log the first field to differ. */
void dyn_verify_after(unsigned char op, struct SRegisters *reg,
                      struct SICPU *icpu, struct SCPUState *cpu);

#ifdef __cplusplus
}
#endif

#endif /* PIKO_DYNAREC_VERIFY_H */
