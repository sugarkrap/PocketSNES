/*
 * dynarec.h -- public interface to piko's PocketSNES dynamic recompiler.
 *
 * This is the very first foundation stone: only the runtime-codegen self-test
 * lives here for now. The block cache, translator and correctness harness land
 * on top of it once the self-test proves the PXA255 can emit-and-run native
 * code at all.
 */
#ifndef PIKO_DYNAREC_H
#define PIKO_DYNAREC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Emit a couple of tiny functions into a runtime code buffer, execute them and
 * check the results. Proves the whole codegen pipeline on this kernel/CPU:
 * an RWX mmap, correct ARMv5 encodings, I-cache flush (__clear_cache), and the
 * host calling convention. Needs no framebuffer/input, so it is safe to run
 * over a plain SSH shell without disturbing the desktop.
 *
 * Returns 1 on success, 0 on failure. Prints a one-line PASS/FAIL to stderr.
 */
int S9xDynSelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* PIKO_DYNAREC_H */
