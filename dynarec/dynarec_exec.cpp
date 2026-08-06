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

extern "C" {
#include "dynarec_arm.h"
#include "dynarec_block.h"
#include "dynarec_exec.h"
/* from dynarec_translate.cpp */
void *dyn_translate_run(const uint8_t *code, int m8, int x8,
                        void *const op_fn_table[256], ArmEmit *e);
}

int dyn_exec_on = 0;

typedef void (*block_fn)(struct SRegisters *, struct SICPU *, struct SCPUState *);

/* generated-code arena (bump-allocated; flushed wholesale when full) */
static uint8_t *code_buf;
static size_t   code_cap, code_used;

/* block cache: (guest_pc<<2 | mode) -> native entry. Open addressing. */
#define EXEC_SLOTS 16384
#define EXEC_MASK  (EXEC_SLOTS - 1)
static uint32_t  exec_key[EXEC_SLOTS];
static void     *exec_ptr[EXEC_SLOTS];
static int       exec_valid[EXEC_SLOTS];
static uint16_t  exec_nat[EXEC_SLOTS];    /* native insns in this block   */
static uint16_t  exec_fb[EXEC_SLOTS];     /* fallback insns in this block */
static unsigned long e_insn_nat, e_insn_fb, e_wram_skip;

static unsigned long e_blocks, e_translated, e_flushes;

static inline uint32_t exec_hash(uint32_t k) { return (k * 2654435761u) & EXEC_MASK; }

static void exec_reset_cache(void)
{
	memset(exec_valid, 0, sizeof(exec_valid));
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
	exec_reset_cache();
	e_blocks = e_translated = e_flushes = 0;
	e_insn_nat = e_insn_fb = e_wram_skip = 0;
	fprintf(stderr, "DYN-EXEC: armed (ROM blocks; EXPERIMENTAL)\n");
}

/* Returns the block and, via *slot, where it lives -- the caller needs the slot
 * to charge the block's native/fallback instruction counts. */
static void *exec_find(uint32_t key, unsigned *slot)
{
	uint32_t h = exec_hash(key);
	unsigned p;
	for (p = 0; p < EXEC_SLOTS; p++) {
		unsigned i = (h + p) & EXEC_MASK;
		if (!exec_valid[i]) return 0;
		if (exec_key[i] == key) { *slot = i; return exec_ptr[i]; }
	}
	return 0;
}

static unsigned exec_slot_of(uint32_t key)
{
	uint32_t h = exec_hash(key);
	unsigned p;
	for (p = 0; p < EXEC_SLOTS; p++) {
		unsigned i = (h + p) & EXEC_MASK;
		if (exec_valid[i] && exec_key[i] == key) return i;
	}
	return EXEC_SLOTS;   /* not found */
}

static void exec_insert(uint32_t key, void *entry)
{
	uint32_t h = exec_hash(key);
	unsigned p;
	for (p = 0; p < EXEC_SLOTS; p++) {
		unsigned i = (h + p) & EXEC_MASK;
		if (!exec_valid[i]) {
			exec_key[i] = key; exec_ptr[i] = entry; exec_valid[i] = 1;
			exec_nat[i] = (uint16_t)dyn_tr_native;
			exec_fb[i]  = (uint16_t)dyn_tr_fallback;
			return;
		}
		if (exec_key[i] == key) {
			exec_ptr[i] = entry;
			exec_nat[i] = (uint16_t)dyn_tr_native;
			exec_fb[i]  = (uint16_t)dyn_tr_fallback;
			return;
		}
	}
}

static void *exec_translate(struct SICPU *icpu, struct SCPUState *cpu,
                            int m8, int x8, uint32_t key)
{
	void *optab[256];
	ArmEmit e;
	void *entry;
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
		}
		return 0;   /* run this block via the interpreter this time */
	}
	__builtin___clear_cache((char *)(code_buf + code_used), (char *)e.cur);
	code_used = (size_t)((uint8_t *)e.cur - code_buf);
	exec_insert(key, entry);
	e_translated++;
	return entry;
}

void dyn_exec_report(void)
{
	unsigned long tot = e_insn_nat + e_insn_fb;
	fprintf(stderr, "DYN-EXEC: FINAL %lu blocks run, %lu translated, %lu flushes\n",
	        e_blocks, e_translated, e_flushes);
	/*
	 * The number that says whether translating more opcodes is worth it. The
	 * PROFILE gate cannot answer this: a translated block jumps past its hook,
	 * so it only ever sees what the dynarec did NOT run.
	 */
	fprintf(stderr, "DYN-EXEC: insns %lu native + %lu fallback = %lu (%lu%% native)\n",
	        e_insn_nat, e_insn_fb, tot, tot ? (e_insn_nat * 100UL) / tot : 0UL);
	fprintf(stderr, "DYN-EXEC: %lu dispatches skipped as WRAM (self-modifying)\n",
	        e_wram_skip);
	fflush(stderr);
}

int dyn_exec_step(struct SRegisters *reg, struct SICPU *icpu, struct SCPUState *cpu)
{
	uint32_t pb, pc, key;
	int m8, x8;
	unsigned slot = EXEC_SLOTS;
	void *native;

	if (!code_buf) return 0;

	pb = reg->PB;
	pc = ((uint32_t)pb << 16) | (uint16_t)(cpu->PC - cpu->PCBase);
	if (dyn_pc_in_wram(pc)) {  /* self-modifying: never cache a block from RAM */
		e_wram_skip++;
		return 0;
	}

	m8 = (reg->P.W & MemoryFlag) != 0;
	x8 = (reg->P.W & IndexFlag)  != 0;
	key = (pc << 2) | (uint32_t)((m8 ? 2 : 0) | (x8 ? 1 : 0));

	native = exec_find(key, &slot);
	if (!native) {
		native = exec_translate(icpu, cpu, m8, x8, key);
		if (!native) return 0;
		slot = exec_slot_of(key);
	}

	((block_fn)native)(reg, icpu, cpu);
	if (slot < EXEC_SLOTS) {
		e_insn_nat += exec_nat[slot];
		e_insn_fb  += exec_fb[slot];
	}
	if (((++e_blocks) % 4000000UL) == 0)
		fprintf(stderr, "DYN-EXEC: %lu blocks run, %lu translated, %lu flushes\n",
		        e_blocks, e_translated, e_flushes);
	return 1;
}
