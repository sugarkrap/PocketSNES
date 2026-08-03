/*
 * dynarec_harness.h -- correctness harness for piko's PocketSNES dynarec.
 *
 * The interpreter is the oracle: the recompiler must reproduce CPU state
 * byte-for-byte. This harness is the safety net that enforces it -- capture
 * the guest CPU state, and diff two captures, reporting the FIRST divergence.
 * The plan is to run each block both ways (interpreter vs recompiled) and diff;
 * every new opcode is added behind this. See dynarec/README.md section 5.
 *
 * The snapshot struct + diff are pure (no snes9x deps) so they unit-test
 * offline. Capturing from the live CPU (dyn_cpu_capture) is added when the
 * recompiler is wired into S9xMainLoop.
 */
#ifndef PIKO_DYNAREC_HARNESS_H
#define PIKO_DYNAREC_HARNESS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Full 65816 architectural state that must match after a block. Mirrors
 * SRegisters (PC/PB/DB/P/A/D/S/X/Y) plus the cycle count -- cycles are state:
 * a right value computed in the wrong number of cycles is a bug (timing drives
 * IRQ/HBLANK/HDMA). ram_hash folds WRAM so memory divergence is caught too. */
typedef struct {
	uint16_t PC;
	uint8_t  PB, DB;
	uint16_t P, A, D, S, X, Y;
	uint32_t cycles;
	uint32_t ram_hash;
} DynCpuSnap;

/* FNV-1a 32-bit over a byte range (used for the WRAM hash). */
uint32_t dyn_hash(const void *data, unsigned len);

/* Compare two snapshots. Returns 1 if identical. On difference returns 0 and,
 * if `whatp` is non-NULL, points it at a static string naming the first field
 * that differs ("A", "P", "cycles", "ram", ...). */
int dyn_cpu_diff(const DynCpuSnap *a, const DynCpuSnap *b, const char **whatp);

#ifdef __cplusplus
}
#endif

#endif /* PIKO_DYNAREC_HARNESS_H */
