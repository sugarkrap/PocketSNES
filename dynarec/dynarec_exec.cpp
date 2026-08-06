/*
 * dynarec_exec.cpp -- block-driven execution. See dynarec_exec.h.
 */
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#include "snes9x.h"
#include "65c816.h"
#include "cpuexec.h"

extern unsigned dyn_tr_native, dyn_tr_fallback;
extern unsigned long dyn_tr_declined_nonative;
extern unsigned long dyn_tr_declined_toolong;
extern int dyn_tr_last_declined;

extern "C" {
#include "dynarec_arm.h"
#include "dynarec_block.h"
#include "dynarec_exec.h"
/* Unconditionally: this file is compiled in every build (Makefile wildcard),
 * and the cache's WRAM page index is part of it regardless of gate. Gating
 * this include left EXEC=1 building cleanly while the default build did not,
 * which is precisely the failure a single-gate check cannot see. */
#include "dynarec_wram.h"
/* from dynarec_translate.cpp */
void *dyn_translate_run(const uint8_t *code, int m8, int x8,
                        void *const op_fn_table[256], ArmEmit *e);
}

int dyn_exec_on = 0;

/* Bumped by cpuexec.cpp for every single-instruction interpreter dispatch. */
unsigned long dyn_interp_dispatches;

/* 1 = the current pc is a plausible block entry, so a cache lookup is worth
 * doing. Starts set; cpuexec.cpp maintains it. */
int dyn_try_block = 1;

typedef void (*block_fn)(struct SRegisters *, struct SICPU *, struct SCPUState *);

/* generated-code arena (bump-allocated; flushed wholesale when full) */
static uint8_t *code_buf;
static size_t   code_cap, code_used;

/* block cache: (guest_pc<<2 | mode) -> native entry. Open addressing.
 *
 * exec_valid is tri-state, and it has to be. Linear probing stops at the first
 * EMPTY slot, so clearing a slot on invalidation would cut the probe chain and
 * hide every key that had collided past it -- they would simply stop being
 * found, the blocks would be re-translated forever, and nothing would look
 * wrong except the throughput. Invalidated slots become TOMBSTONEs instead:
 * probing walks through them, insertion reuses them. */
#define EXEC_SLOTS 16384
#define EXEC_MASK  (EXEC_SLOTS - 1)
/*
 * A cached REFUSAL. Declining an all-fallback block is only a win if the
 * decision is remembered: without this, pass 1 re-walked and re-decoded the
 * block on every single dispatch, 4.75M times in 90s, and throughput fell from
 * 41.15 to 28.38 frames/s -- worse than translating the block had been.
 */
#define EXEC_DECLINED ((void *)1)
#define EXEC_EMPTY 0
#define EXEC_LIVE  1
#define EXEC_TOMB  2
static uint32_t  exec_key[EXEC_SLOTS];
static void     *exec_ptr[EXEC_SLOTS];
static int       exec_valid[EXEC_SLOTS];
static uint16_t  exec_nat[EXEC_SLOTS];    /* native insns in this block   */
static uint16_t  exec_fb[EXEC_SLOTS];     /* fallback insns in this block */
static unsigned long e_insn_nat, e_insn_fb, e_wram_skip;

static unsigned long e_blocks, e_translated, e_flushes;
static unsigned long e_wram_blocks, e_inval_calls, e_inval_blocks, e_wram_full;
static unsigned long e_declined_hits;

/*
 * WRAM blocks: page -> slot reverse index.
 *
 * Chained on the page the block STARTS in, with the span kept per slot, so a
 * write to page P must also check the few pages before it -- a block can be up
 * to DYN_BLOCK_MAX_BYTES long and reach forward into P. Scanning all 16,384
 * slots per write was never an option; walking five list heads is nothing, and
 * the measurement (README 8c) says the lists are one entry long in practice.
 */
#define WRAM_SPAN_PAGES ((DYN_BLOCK_MAX_BYTES >> DYN_WRAM_PAGE_SHIFT) + 1)
#define WRAM_UNCHAINED  0xFFFFu
uint8_t dyn_code_page[DYN_WRAM_PAGES];
int     dyn_wram_tracking;      /* set by dyn_exec_init when WRAM blocks are on */
static int16_t  wram_head[DYN_WRAM_PAGES];    /* first slot on this page, -1 */
static int16_t  wram_next[EXEC_SLOTS];        /* next slot in that list      */
static uint16_t wram_pg_first[EXEC_SLOTS];    /* WRAM_UNCHAINED = not a WRAM block */
static uint16_t wram_pg_last[EXEC_SLOTS];

/*
 * OFF by default, on the measurement rather than on principle.
 *
 * The invalidation below works: FF6 ran 90s with its $001500 trampoline being
 * rewritten under a cached block, ~21 invalidations a second, no corruption.
 * But an A/B on one boot, same binary, one env var apart, says translating
 * WRAM blocks is a LOSS at the current level of native opcode coverage:
 *
 *     ROM only    5,785,909 blocks  43% native  41.81 frames/s
 *     ROM + WRAM 17,522,850 blocks  20% native  31.63 frames/s
 *
 * Three times the blocks for three quarters of the speed. WRAM code hits
 * almost nothing the translator handles natively, so each of those blocks is
 * a prologue and an epilogue wrapped around a chain of interpreter calls --
 * strictly more work than letting the interpreter dispatch it directly.
 *
 * This is worth turning on once native coverage reaches the opcodes WRAM code
 * actually uses; the machinery is ready and costs nothing while it is off.
 * PIKO_DYN_WRAM_BLOCKS=1 enables it, for exactly that comparison.
 */
int dyn_exec_wram_blocks = 0;


/*
 * Slots actually in use. The arrays stay statically sized; this only limits
 * which entries get TOUCHED, which is the point -- cache pressure is about the
 * working set, not the allocation. exec_key/exec_ptr/exec_valid are 64 KB each,
 * 192 KB probed by a hash, against 32 KB of L1 data cache on the PXA270. If
 * that is where the ~20% of arming EXEC goes, shrinking this must recover it;
 * if it does not, the hypothesis is dead. PIKO_DYN_SLOTS sets it.
 */
int dyn_exec_slots = EXEC_SLOTS;
int dyn_exec_stub = 0;
static uint32_t exec_mask = EXEC_MASK;

static inline uint32_t exec_hash(uint32_t k) { return (k * 2654435761u) & exec_mask; }

static void exec_reset_cache(void)
{
	unsigned i;
	memset(exec_valid, 0, sizeof(exec_valid));
	memset(dyn_code_page, 0, sizeof(dyn_code_page));
	for (i = 0; i < DYN_WRAM_PAGES; i++) wram_head[i] = -1;
	for (i = 0; i < EXEC_SLOTS; i++) wram_pg_first[i] = WRAM_UNCHAINED;
}

/* Remove `slot` from its page list. Every path that frees a slot must call
 * this, or the list ends up pointing at a slot since reused for a different
 * block -- which would invalidate the wrong entry and leave the right one
 * stale, the exact failure this whole mechanism exists to prevent. */
static void wram_unlink(unsigned slot)
{
	int16_t *pp;

	if (wram_pg_first[slot] == WRAM_UNCHAINED) return;
	pp = &wram_head[wram_pg_first[slot]];
	while (*pp >= 0) {
		if ((unsigned)*pp == slot) { *pp = wram_next[slot]; break; }
		pp = &wram_next[*pp];
	}
	wram_pg_first[slot] = WRAM_UNCHAINED;
}

static void wram_chain(unsigned slot, uint32_t off, unsigned len)
{
	uint32_t first = off >> DYN_WRAM_PAGE_SHIFT;
	uint32_t last  = (off + (len ? len : 1) - 1) >> DYN_WRAM_PAGE_SHIFT;
	uint32_t p;

	if (last >= DYN_WRAM_PAGES) last = DYN_WRAM_PAGES - 1;
	wram_unlink(slot);                     /* may be a reused slot */
	for (p = first; p <= last; p++) dyn_code_page[p] = 1;
	wram_pg_first[slot] = (uint16_t)first;
	wram_pg_last[slot]  = (uint16_t)last;
	wram_next[slot]     = wram_head[first];
	wram_head[first]    = (int16_t)slot;
}

extern "C" void dyn_wram_invalidate_page(unsigned page)
{
	unsigned q, qlo;
	int hit = 0;

	qlo = (page >= WRAM_SPAN_PAGES - 1) ? page - (WRAM_SPAN_PAGES - 1) : 0;
	for (q = qlo; q <= page; q++) {
		int16_t s = wram_head[q];
		while (s >= 0) {
			int16_t nx = wram_next[s];
			if (wram_pg_first[s] <= page && wram_pg_last[s] >= page) {
				exec_valid[s] = EXEC_TOMB;
				wram_unlink((unsigned)s);
				e_inval_blocks++;
				hit = 1;
			}
			s = nx;
		}
	}
	if (hit) e_inval_calls++;
	/* dyn_code_page[page] is deliberately left set: a stale 1 only costs the
	 * walk above finding nothing, and the block is normally re-translated into
	 * the same page within a frame anyway. */
}

extern "C" void dyn_wram_flush_all(void)
{
	if (!code_buf) return;
	code_used = 0;
	exec_reset_cache();
	e_flushes++;
}

void dyn_exec_init(void)
{
	code_cap = 1u << 20;   /* 1 MiB of generated code */
	code_buf = (uint8_t *)mmap(NULL, code_cap, PROT_READ | PROT_WRITE | PROT_EXEC,
	                           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (code_buf == MAP_FAILED) {
		code_buf = 0;
		dyn_exec_on = 0;
		fprintf(stderr, "DYN-EXEC: code arena mmap failed, disabled\n");
		return;
	}
	code_used = 0;
	if (dyn_exec_slots < 16) dyn_exec_slots = 16;
	if (dyn_exec_slots > EXEC_SLOTS) dyn_exec_slots = EXEC_SLOTS;
	/* round down to a power of two: the hash masks, it does not modulo */
	while (dyn_exec_slots & (dyn_exec_slots - 1)) dyn_exec_slots &= dyn_exec_slots - 1;
	exec_mask = (uint32_t)dyn_exec_slots - 1;
	exec_reset_cache();
	/* Only make the write path pay for tracking if something will use it. */
	dyn_wram_tracking = dyn_exec_wram_blocks;
	e_blocks = e_translated = e_flushes = 0;
	e_insn_nat = e_insn_fb = e_wram_skip = 0;
	e_wram_blocks = e_inval_calls = e_inval_blocks = e_wram_full = 0;
	e_declined_hits = 0;
	dyn_interp_dispatches = 0;
	fprintf(stderr, "DYN-EXEC: armed (%s, min-native %d%%; EXPERIMENTAL)\n",
	        dyn_exec_wram_blocks ? "ROM + WRAM blocks, write-invalidated"
	                             : "ROM blocks only", dyn_exec_min_native);
	fprintf(stderr, "DYN-EXEC: %d cache slots (%d KB touched)\n",
	        dyn_exec_slots, (dyn_exec_slots * 12) / 1024);
}

/* Returns the block and, via *slot, where it lives -- the caller needs the slot
 * to charge the block's native/fallback instruction counts. */
static void *exec_find(uint32_t key, unsigned *slot)
{
	uint32_t h = exec_hash(key);
	unsigned p;
	for (p = 0; p < (unsigned)dyn_exec_slots; p++) {
		unsigned i = (h + p) & exec_mask;
		if (exec_valid[i] == EXEC_EMPTY) return 0;   /* tombstones do NOT stop us */
		if (exec_valid[i] == EXEC_LIVE && exec_key[i] == key) {
			*slot = i;
			return exec_ptr[i];
		}
	}
	return 0;
}

static unsigned exec_slot_of(uint32_t key)
{
	uint32_t h = exec_hash(key);
	unsigned p;
	for (p = 0; p < (unsigned)dyn_exec_slots; p++) {
		unsigned i = (h + p) & exec_mask;
		if (exec_valid[i] == EXEC_EMPTY) break;
		if (exec_valid[i] == EXEC_LIVE && exec_key[i] == key) return i;
	}
	return EXEC_SLOTS;   /* not found */
}

/* Returns the slot used, or EXEC_SLOTS if the table is full. */
static unsigned exec_insert(uint32_t key, void *entry)
{
	uint32_t h = exec_hash(key);
	unsigned p, tomb = EXEC_SLOTS;

	for (p = 0; p < (unsigned)dyn_exec_slots; p++) {
		unsigned i = (h + p) & exec_mask;
		if (exec_valid[i] == EXEC_TOMB) {
			/* remember the first reusable slot, but keep probing: the key may
			 * already be live further along the chain */
			if (tomb == EXEC_SLOTS) tomb = i;
			continue;
		}
		if (exec_valid[i] == EXEC_EMPTY) {
			if (tomb != EXEC_SLOTS) i = tomb;
			exec_key[i] = key; exec_ptr[i] = entry; exec_valid[i] = EXEC_LIVE;
			exec_nat[i] = (uint16_t)dyn_tr_native;
			exec_fb[i]  = (uint16_t)dyn_tr_fallback;
			return i;
		}
		if (exec_key[i] == key) {
			exec_ptr[i] = entry;
			exec_nat[i] = (uint16_t)dyn_tr_native;
			exec_fb[i]  = (uint16_t)dyn_tr_fallback;
			return i;
		}
	}
	return EXEC_SLOTS;
}

static void *exec_translate(struct SICPU *icpu, struct SCPUState *cpu,
                            int m8, int x8, uint32_t key, uint32_t pc)
{
	void *optab[256];
	ArmEmit e;
	void *entry;
	unsigned slot;
	uint32_t woff;
	int i;

	/* fallback targets = the interpreter's opcode table for THIS mode (which is
	 * the live icpu->S9xOpcodes, since we translate in the mode we're about to
	 * run). */
	for (i = 0; i < 256; i++)
		optab[i] = (void *)icpu->S9xOpcodes[i].S9xOpcode;

	arm_emit_init(&e, code_buf + code_used, (unsigned)((code_cap - code_used) / 4));
	entry = dyn_translate_run(cpu->PC, m8, x8, optab, &e);

	if (!entry || e.overflow) {
		if (e.overflow) {
			/* arena full: flush everything and start over (cheap, rare) */
			code_used = 0;
			exec_reset_cache();
			e_flushes++;
		} else if (dyn_tr_last_declined) {
			/* Remember the refusal. Nothing was emitted, so no arena is spent;
			 * the slot exists purely so the next dispatch costs a hash lookup
			 * instead of a full re-walk of the block. */
			dyn_tr_native = dyn_tr_fallback = 0;
			exec_insert(key, EXEC_DECLINED);
		}
		return 0;   /* run this block via the interpreter this time */
	}
	__builtin___clear_cache((char *)(code_buf + code_used), (char *)e.cur);
	code_used = (size_t)((uint8_t *)e.cur - code_buf);
	slot = exec_insert(key, entry);
	if (slot == EXEC_SLOTS) return 0;    /* table full; run it interpreted */
	e_translated++;

	/*
	 * A block built out of WRAM is a snapshot of bytes the game can rewrite,
	 * so it has to be findable from the page it came from. Chained by WRAM
	 * OFFSET, not by guest PC, which is what makes the mirrors work: $00:1500
	 * and $7E:1500 are two cache keys over the same bytes, and a write to
	 * either must kill both.
	 */
	if (dyn_wram_offset_of(pc, &woff)) {
		/*
		 * The span is recomputed with dyn_translate_run's own advance/terminate
		 * rule, NOT with dyn_discover_block. Discovery also stops at
		 * DYN_BLOCK_MAX_INSNS, so a block of more than 256 short instructions
		 * would report a length shorter than the code actually translated --
		 * and every page past that point would go unwatched. A block whose
		 * tail nobody watches is exactly the stale-cache bug all of this
		 * exists to prevent, and it would show up as a rare, unreproducible
		 * corruption rather than as anything a counter would catch.
		 */
		const uint8_t *c = (const uint8_t *)cpu->PC;
		unsigned o = 0;
		for (;;) {
			uint8_t op = c[o];
			int ender = dyn_op_ends_translation(op);
			o += (unsigned)dyn_op_length(op, m8, x8);
			if (ender || o >= DYN_BLOCK_MAX_BYTES) break;
		}
		wram_chain(slot, woff, o);
		e_wram_blocks++;
	}
	return entry;
}

void dyn_exec_report(void)
{
	unsigned long tot = e_insn_nat + e_insn_fb;
	fprintf(stderr, "DYN-EXEC: FINAL %lu blocks run, %lu translated, %lu flushes%s\n",
	        e_blocks, e_translated, e_flushes,
#ifdef PIKO_DYN_NOCOUNT
	        "  [NOCOUNT build: other counters below read 0 because they are"
	        " compiled out, not because nothing happened]"
#else
	        ""
#endif
	        );
	/*
	 * STATIC instruction counts: each dispatch charges the block's whole
	 * translated length. That was exact until branches stopped ending blocks;
	 * now a taken branch exits early, so these are UPPER BOUNDS and the split
	 * is "what the block contains", not "what it ran". Do not read them as
	 * executed-instruction counts -- they can exceed the instructions the
	 * emulated CPU actually issued, and did.
	 */
#ifdef PIKO_DYN_BLOCKSTATS
	fprintf(stderr, "DYN-EXEC: block contents (static) %lu native + %lu fallback = %lu (%lu%% native)\n",
	        e_insn_nat, e_insn_fb, tot, tot ? (e_insn_nat * 100UL) / tot : 0UL);
#else
	(void)tot;
	fprintf(stderr, "DYN-EXEC: block native/fallback split not built"
	                " (make BLOCKSTATS=1; costs 4.5%%)\n");
#endif
	/*
	 * The coverage question, and the one that decides whether any of this can
	 * pay: how many instructions does the interpreter dispatch one at a time,
	 * outside any block? If that dominates, the dynarec is not in the hot path
	 * and making blocks better cannot matter.
	 */
	fprintf(stderr, "DYN-EXEC: %lu instructions dispatched outside blocks\n",
	        dyn_interp_dispatches);
	/* Near zero is EXPECTED and not a broken probe: with WRAM blocks off,
	 * cpuexec.cpp's inline filter rejects those dispatches before this function
	 * is called, which is the entire point of it. */
	fprintf(stderr, "DYN-EXEC: %lu WRAM dispatches reached this function"
	                " (the inline filter takes the rest)\n", e_wram_skip);
	/*
	 * e_inval_blocks is the thing to watch. If it tracks e_wram_blocks closely,
	 * blocks are being thrown away about as fast as they are built and the
	 * cache is doing no work; the measurement (README 8c) predicts ~0.72
	 * invalidations a frame against a stable WRAM block population.
	 */
	fprintf(stderr, "DYN-EXEC: WRAM blocks %lu translated, %lu invalidated in %lu events\n",
	        e_wram_blocks, e_inval_blocks, e_inval_calls);
	/* Blocks refused as all-fallback. These are dispatched by the interpreter
	 * instead, which is strictly cheaper than wrapping them in a prologue. */
	fprintf(stderr, "DYN-EXEC: %lu declined all-fallback, %lu declined too-long, %lu cached-refusal hits\n",
	        dyn_tr_declined_nonative, dyn_tr_declined_toolong, e_declined_hits);
	fflush(stderr);
}

int dyn_exec_step(struct SRegisters *reg, struct SICPU *icpu, struct SCPUState *cpu)
{
	uint32_t pb, pc, key;
	int m8, x8;
	unsigned slot = EXEC_SLOTS;
	void *native;

	if (!code_buf) return 0;
	/*
	 * Ablation knob (PIKO_DYN_STUB=1): keep the call, do nothing. Splits the
	 * cost of arming EXEC into "the call and its effect on the dispatch loop"
	 * versus "the work inside this function". Three hypotheses have already
	 * died here -- the blocks themselves (zero blocks still cost it), the
	 * number of lookups (4x fewer was slower), and the table footprint (8x
	 * smaller changed nothing) -- so the next step is to bisect rather than
	 * guess at a fourth.
	 */
	if (dyn_exec_stub) return 0;

	pb = reg->PB;
	pc = ((uint32_t)pb << 16) | (uint16_t)(cpu->PC - cpu->PCBase);
	m8 = (reg->P.W & MemoryFlag) != 0;
	x8 = (reg->P.W & IndexFlag)  != 0;

#ifdef PIKO_DYNAREC_WRAMSTAT
	if (dyn_pc_in_wram(pc))
		dyn_wram_saw_block(pc, (const uint8_t *)cpu->PC, m8, x8);
#endif
	/*
	 * WRAM blocks used to be refused outright, because a cached block is a
	 * snapshot of bytes the game can rewrite underneath it. They are now
	 * translated and invalidated on write instead -- that was 72% of all
	 * dispatch going to the interpreter (README 8c).
	 */
	/*
	 * dyn_wram_offset_of() is the INLINE form of the same test (dynarec_wram.h,
	 * mirrors and all); dyn_pc_in_wram() is a cross-translation-unit call, and
	 * this runs on every dispatched instruction. Bisection put ~17% of the cost
	 * of arming EXEC in this function's body rather than in its hash table --
	 * which is also why shrinking that table 8x changed nothing: most
	 * dispatches are rejected right here and never reach it.
	 */
	if (!dyn_exec_wram_blocks) {
		uint32_t woff;
		if (dyn_wram_offset_of(pc, &woff)) {
			DYN_COUNT (e_wram_skip++);
			return 0;
		}
	}

	key = (pc << 2) | (uint32_t)((m8 ? 2 : 0) | (x8 ? 1 : 0));

	native = exec_find(key, &slot);
	if (native == EXEC_DECLINED) { DYN_COUNT (e_declined_hits++); return 0; }
	if (!native) {
		native = exec_translate(icpu, cpu, m8, x8, key, pc);
		if (!native) return 0;
		slot = exec_slot_of(key);
	}

	((block_fn)native)(reg, icpu, cpu);
	/*
	 * The native/fallback split costs two loads from 32 KB arrays plus two
	 * global read-modify-writes on EVERY block run, and measured 4.5% of the
	 * block path (43.23 -> 45.16 frames/s with it out). It is a diagnostic, so
	 * it is off unless asked for: `make BLOCKSTATS=1`. e_blocks stays, because
	 * the FINAL line is how a run is known to have done anything at all.
	 */
#ifdef PIKO_DYN_BLOCKSTATS
	if (slot < EXEC_SLOTS) {
		e_insn_nat += exec_nat[slot];
		e_insn_fb  += exec_fb[slot];
	}
#else
	(void)slot;
#endif
	if (((++e_blocks) % 4000000UL) == 0)
		fprintf(stderr, "DYN-EXEC: %lu blocks run, %lu translated, %lu flushes\n",
		        e_blocks, e_translated, e_flushes);
	return 1;
}
