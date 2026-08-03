/*
 * dynarec_exec.h -- block-driven execution: translated blocks actually drive
 * the CPU. Compiled only in `make EXEC=1` (-DPIKO_DYNAREC_EXEC), runtime-gated
 * by PIKO_DYN_EXEC. EXPERIMENTAL and default-off: this runs generated code as
 * the live CPU, so a bug can hang or reboot the device.
 *
 * Scope of this first cut:
 *  - ROM blocks only (PB != $7E/$7F). RAM-resident code can be self-modifying,
 *    which needs cache invalidation we don't have yet, so RAM PCs fall through
 *    to the interpreter.
 *  - Cycle parity with the interpreter (per-instruction MemSpeed add inside the
 *    block); event servicing (APU/HBLANK/IRQ) stays at the main loop, now at
 *    block granularity rather than per instruction.
 *  - No speed win expected yet -- most opcodes are still interpreter fallbacks,
 *    so blocks pay spill/reload overhead. This step proves the plumbing; the
 *    win comes as native opcode coverage grows.
 */
#ifndef PIKO_DYNAREC_EXEC_H
#define PIKO_DYNAREC_EXEC_H

#ifdef __cplusplus
extern "C" {
#endif

struct SRegisters;
struct SICPU;
struct SCPUState;

extern int dyn_exec_on;

void dyn_exec_init(void);

/* Try to run one translated block starting at the current PC. Returns 1 if a
 * block ran (cpu->PC now at the block's exit target), 0 if the caller should
 * fall through to the normal single-instruction interpreter path. */
int  dyn_exec_step(struct SRegisters *reg, struct SICPU *icpu, struct SCPUState *cpu);

#ifdef __cplusplus
}
#endif

#endif /* PIKO_DYNAREC_EXEC_H */
