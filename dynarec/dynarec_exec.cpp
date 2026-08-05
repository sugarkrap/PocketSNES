/*
 * dynarec_exec.cpp -- block-driven execution. See dynarec_exec.h.
 */
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#include "snes9x.h"
#include "65c816.h"
#include "cpuexec.h"

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
	fprintf(stderr, "DYN-EXEC: armed (ROM blocks; EXPERIMENTAL)\n");
}

static void *exec_find(uint32_t key)
{
	uint32_t h = exec_hash(key);
	unsigned p;
	for (p = 0; p < EXEC_SLOTS; p++) {
		unsigned i = (h + p) & EXEC_MASK;
		if (!exec_valid[i]) return 0;
		if (exec_key[i] == key) return exec_ptr[i];
	}
	return 0;
}

static void exec_insert(uint32_t key, void *entry)
{
	uint32_t h = exec_hash(key);
	unsigned p;
	for (p = 0; p < EXEC_SLOTS; p++) {
		unsigned i = (h + p) & EXEC_MASK;
		if (!exec_valid[i]) {
			exec_key[i] = key; exec_ptr[i] = entry; exec_valid[i] = 1;
			return;
		}
		if (exec_key[i] == key) { exec_ptr[i] = entry; return; }
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

int dyn_exec_step(struct SRegisters *reg, struct SICPU *icpu, struct SCPUState *cpu)
{
	uint32_t pb, pc, key;
	int m8, x8;
	void *native;

	if (!code_buf) return 0;

	pb = reg->PB;
	pc = ((uint32_t)pb << 16) | (uint16_t)(cpu->PC - cpu->PCBase);
	if (dyn_pc_in_wram(pc))    /* self-modifying: never cache a block from RAM */
		return 0;

	m8 = (reg->P.W & MemoryFlag) != 0;
	x8 = (reg->P.W & IndexFlag)  != 0;
	key = (pc << 2) | (uint32_t)((m8 ? 2 : 0) | (x8 ? 1 : 0));

	native = exec_find(key);
	if (!native) {
		native = exec_translate(icpu, cpu, m8, x8, key);
		if (!native) return 0;
	}

	((block_fn)native)(reg, icpu, cpu);
	if (((++e_blocks) % 4000000UL) == 0)
		fprintf(stderr, "DYN-EXEC: %lu blocks run, %lu translated, %lu flushes\n",
		        e_blocks, e_translated, e_flushes);
	return 1;
}
