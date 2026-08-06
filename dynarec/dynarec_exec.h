/*
 * dynarec_exec.h -- block-driven execution: translated blocks actually drive
 * the CPU. Compiled only in `make EXEC=1` (-DPIKO_DYNAREC_EXEC), runtime-gated
 * by PIKO_DYN_EXEC. EXPERIMENTAL and default-off: this runs generated code as
 * the live CPU, so a bug can hang or reboot the device.
 *
 * Scope:
 *  - ROM and WRAM blocks. WRAM code can be rewritten under a cached block, so
 *    those blocks are indexed by the WRAM page they came from and discarded
 *    when the game writes there (dynarec_wram.h). WRAM was 72% of all dispatch
 *    while it was refused, so this is where the remaining win is.
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

/*
 * Hot-path counters, compiled out by `make NOCOUNT=1` (-DPIKO_DYN_NOCOUNT).
 *
 * These sit on paths taken by nearly every dispatched instruction, and each is
 * a load/add/store to a global in its own cache line. Bisection put ~17% of
 * the cost of arming EXEC inside dyn_exec_step's body, after the blocks, the
 * lookup count, the table footprint and the WRAM call were each ruled out on
 * hardware -- so the counters are the next suspect, and this is how they get
 * ruled in or out rather than argued about.
 *
 * A NOCOUNT build reports zeroes. It says so when it does: a silent zero reads
 * exactly like "the probe never fired".
 */
#ifdef PIKO_DYN_NOCOUNT
#define DYN_COUNT(stmt) ((void)0)
#else
#define DYN_COUNT(stmt) do { stmt; } while (0)
#endif

extern int dyn_exec_on;

/* Instructions the interpreter dispatched individually (i.e. NOT inside a
 * translated block). Together with the block counters this says how much of
 * the workload the dynarec actually sees. */
extern unsigned long dyn_interp_dispatches;

/* Gate on the per-dispatch block lookup; see cpuexec.cpp. */
extern int dyn_try_block;

/* 1 = also translate blocks running out of WRAM. Default 0: the mechanism is
 * correct but currently costs 24% (see dynarec_exec.cpp). Set by
 * PIKO_DYN_WRAM_BLOCKS=1, which is how the two get compared on ONE boot --
 * comparing across a device reboot has already produced numbers that turned
 * out not to be comparable. */
extern int dyn_exec_wram_blocks;

/* Minimum %% of a block's instructions that must be native before it is worth
 * translating (PIKO_DYN_MIN_NATIVE). See dynarec_translate.cpp. */
extern int dyn_exec_min_native;

/* Block-cache slots actually used (PIKO_DYN_SLOTS, rounded down to a power of
 * two). Shrinking this shrinks the working set the hash probes touch. */
extern int dyn_exec_slots;

/* PIKO_DYN_STUB=1: dyn_exec_step returns immediately. Ablation only. */
extern int dyn_exec_stub;

void dyn_exec_init(void);

/* Try to run one translated block starting at the current PC. Returns 1 if a
 * block ran (cpu->PC now at the block's exit target), 0 if the caller should
 * fall through to the normal single-instruction interpreter path. */
int  dyn_exec_step(struct SRegisters *reg, struct SICPU *icpu, struct SCPUState *cpu);

/* Print the block/translation/flush counters immediately. The periodic report
 * only fires every 4M blocks, so a bounded run can end having printed nothing
 * at all -- which reads exactly like "the dynarec never ran" and cost a
 * hardware run to tell apart. Call this before exiting. */
void dyn_exec_report(void);

#ifdef __cplusplus
}
#endif

#endif /* PIKO_DYNAREC_EXEC_H */
