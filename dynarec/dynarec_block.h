/*
 * dynarec_block.h -- 65816 instruction decoder + block discovery + block cache
 * for piko's PocketSNES dynarec.
 *
 * "Block cache skeleton" (design README, Step 1): the data structures the
 * translator hangs on, plus a decoder that walks a straight-line run of 65816
 * instructions to a block-ender. Recompiles nothing yet -- this is the chassis
 * and it is intentionally decoupled from the live CPU so it can be unit-tested
 * offline (no emulator init, safe over SSH).
 *
 * The decoder reuses snes9x's own authoritative AddrModes[256] table and its
 * mode->length rules (see debug.cpp's S9xOPrint disassembler) rather than a
 * hand-rolled length table -- same source of truth the interpreter documents.
 */
#ifndef PIKO_DYNAREC_BLOCK_H
#define PIKO_DYNAREC_BLOCK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Length in bytes of the instruction whose opcode is `op`, given the current
 * accumulator/memory width (m8=1 => 8-bit A/M) and index width (x8=1 => 8-bit
 * X/Y). Only the two immediate addressing modes depend on m8/x8. */
int dyn_op_length(uint8_t op, int m8, int x8);

/* True if `op` ends a translation block: any control-flow transfer (branch,
 * JMP/JML, JSR/JSL, RTS/RTL/RTI, BRK/COP), any op that can change the M/X/E
 * mode (REP/SEP/XCE/PLP) -- because that swaps the live opcode table and thus
 * the block's validity -- plus STP/WAI (halt) and MVP/MVN (self-repeating). */
int dyn_op_is_block_end(uint8_t op);

/* The same answer as a table, for callers on the hot path (cpuexec.cpp asks
 * once per dispatched instruction; a call plus a compare chain showed up in
 * the frame rate). */
extern const uint8_t dyn_block_end_tab[256];

/* True if `op` is a conditional branch the translator emits natively (its
 * not-taken path continues the block). */
int dyn_op_is_cond_branch(uint8_t op);

/* True if `op` terminates a TRANSLATED block. Differs from
 * dyn_op_is_block_end only for the branches above. Everything that walks a
 * block the way the translator does -- notably the WRAM page-span computation
 * in dynarec_exec.cpp -- must use THIS, or it will stop early and leave the
 * block's tail untracked. */
int dyn_op_ends_translation(uint8_t op);

/* True if the 24-bit guest address `pc` lands in WRAM, INCLUDING its mirrors.
 * A translated block is a snapshot of the bytes it was built from, so code in
 * writable memory must never be cached: the game can rewrite it underneath us
 * and the cache would happily keep running the stale version forever. */
int dyn_pc_in_wram(uint32_t pc);

/* A discovered block: a straight-line instruction run valid for one (M,X) mode. */
typedef struct {
	uint32_t start_pc;   /* 24-bit guest address of the first instruction */
	uint16_t length;     /* total bytes covered                          */
	uint16_t n_insns;    /* instruction count                            */
	uint8_t  m8, x8;     /* mode this block was decoded under            */
	uint8_t  end_op;     /* the block-ending opcode (0 if length-capped) */
	uint32_t hits;       /* execution count (hotness; filled by the cache) */
} DynBlock;

/* Safety caps so discovery always terminates even on garbage/data. */
#define DYN_BLOCK_MAX_INSNS 256
#define DYN_BLOCK_MAX_BYTES 1024

/* Decode forward from `code` (a host pointer at the block's first byte, e.g.
 * CPU.PCBase + offset) under mode (m8,x8), filling `b`. Returns instruction
 * count. `code` must be readable for up to DYN_BLOCK_MAX_BYTES. */
int dyn_discover_block(const uint8_t *code, uint32_t start_pc,
                       int m8, int x8, DynBlock *b);

/* ---- block cache (open-addressing hash on (start_pc, mode)) ------------ */
void      dyn_cache_init(void);
/* Look up an existing block for (pc,m8,x8), or NULL. */
DynBlock *dyn_cache_find(uint32_t pc, int m8, int x8);
/* Insert (copying `b`) and return the stored entry, or NULL if the cache is
 * full. If an entry already exists it is returned as-is (hits preserved). */
DynBlock *dyn_cache_insert(const DynBlock *b);
/* Number of live entries (diagnostics/tests). */
unsigned  dyn_cache_count(void);

/* ---- hotness profiler (observation only; see PIKO_DYNAREC_PROFILE) ------ *
 * A gated, behaviour-neutral probe for answering "where does the CPU actually
 * spend its time in real games" before we translate a single opcode:
 *   - dyn_opcount[op]  : how often each opcode executes (what to translate 1st)
 *   - block-start hits : hot (PC,mode) blocks (via the cache's hit counter)
 * Enable at runtime with dyn_profile_on (set from PIKO_DYN_PROFILE in
 * mainEntry). Only compiled/called in PROFILE builds. */
extern int dyn_profile_on;

/* Called once per dispatched instruction (PROFILE builds only). `op` is the
 * current opcode byte, `pc` its 24-bit guest address, m8/x8 the live mode.
 * Counts the opcode and, at block starts, bumps the block's hit count; dumps
 * a top-N summary to stderr every few hundred thousand instructions. */
void dyn_profile_op(uint8_t op, uint32_t pc, int m8, int x8);

/* Print the hottest opcodes (with mnemonics) and hottest blocks to stderr. */
void dyn_profile_dump(unsigned top_ops, unsigned top_blocks);

#ifdef __cplusplus
}
#endif

#endif /* PIKO_DYNAREC_BLOCK_H */
