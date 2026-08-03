/*
 * dynarec_block.c -- 65816 decoder + block discovery + block cache.
 * See dynarec_block.h. Decoupled from live CPU state on purpose (unit-testable).
 */
#include <string.h>
#include "dynarec_block.h"

/*
 * Opcode -> addressing-mode id, copied verbatim from snes9x's disassembler
 * (debug.cpp: AddrModes[256]). The authoritative table the interpreter itself
 * documents; do not hand-edit -- if snes9x's changes, re-copy.
 */
static const uint8_t AddrModes[256] = {
  /*0   1  2   3  4  5  6   7  8   9  A  B   C   D   E   F */
    3, 10, 3, 19, 6, 6, 6, 12, 0,  1,24, 0, 14, 14, 14, 17, /*0*/
    4, 11, 9, 20, 6, 7, 7, 13, 0, 16,24, 0, 14, 15, 15, 18, /*1*/
   14, 10,17, 19, 6, 6, 6, 12, 0,  1,24, 0, 14, 14, 14, 17, /*2*/
    4, 11, 9, 20, 7, 7, 7, 13, 0, 16,24, 0, 15, 15, 15, 18, /*3*/
    0, 10, 3, 19,25, 6, 6, 12, 0,  1,24, 0, 14, 14, 14, 17, /*4*/
    4, 11, 9, 20,25, 7, 7, 13, 0, 16, 0, 0, 17, 15, 15, 18, /*5*/
    0, 10, 5, 19, 6, 6, 6, 12, 0,  1,24, 0, 21, 14, 14, 17, /*6*/
    4, 11, 9, 20, 7, 7, 7, 13, 0, 16, 0, 0, 23, 15, 15, 18, /*7*/
    4, 10, 5, 19, 6, 6, 6, 12, 0,  1, 0, 0, 14, 14, 14, 17, /*8*/
    4, 11, 9, 20, 7, 7, 8, 13, 0, 16, 0, 0, 14, 15, 15, 18, /*9*/
    2, 10, 2, 19, 6, 6, 6, 12, 0,  1, 0, 0, 14, 14, 14, 17, /*A*/
    4, 11, 9, 20, 7, 7, 8, 13, 0, 16, 0, 0, 15, 15, 16, 18, /*B*/
    2, 10, 3, 19, 6, 6, 6, 12, 0,  1, 0, 0, 14, 14, 14, 17, /*C*/
    4, 11, 9,  9, 0, 7, 7, 13, 0, 16, 0, 0, 22, 15, 15, 18, /*D*/
    2, 10, 3, 19, 6, 6, 6, 12, 0,  1, 0, 0, 14, 14, 14, 17, /*E*/
    4, 11, 9, 20,14, 7, 7, 13, 0, 16, 0, 0, 23, 15, 15, 18  /*F*/
};

/*
 * Base instruction length per addressing-mode id, from debug.cpp's S9xOPrint
 * switch (the Size it assigns each mode). Modes 1 and 2 are placeholders here
 * (2) because they are width-dependent and handled specially in dyn_op_length.
 */
static const uint8_t ModeSize[26] = {
	/* 0 implied            */ 1,
	/* 1 imm[M]  (special)  */ 2,
	/* 2 imm[X]  (special)  */ 2,
	/* 3 imm always-8       */ 2,
	/* 4 relative           */ 2,
	/* 5 relative long      */ 3,
	/* 6 direct             */ 2,
	/* 7 direct,X           */ 2,
	/* 8 direct,Y           */ 2,
	/* 9 direct indirect    */ 2,
	/*10 direct idx indirect*/ 2,
	/*11 direct indirect idx*/ 2,
	/*12 direct indirect lng*/ 2,
	/*13 dir ind idx long   */ 2,
	/*14 absolute           */ 3,
	/*15 absolute,X         */ 3,
	/*16 absolute,Y         */ 3,
	/*17 absolute long      */ 4,
	/*18 absolute long,X    */ 4,
	/*19 stack relative     */ 2,
	/*20 stack rel ind idx  */ 2,
	/*21 absolute indirect  */ 3,
	/*22 absolute ind long  */ 3,
	/*23 absolute idx ind   */ 3,
	/*24 implied accumulator*/ 1,
	/*25 MVN/MVP src dst     */ 3
};

int dyn_op_length(uint8_t op, int m8, int x8)
{
	uint8_t mode = AddrModes[op];
	if (mode == 1) return m8 ? 2 : 3;   /* immediate, accumulator width */
	if (mode == 2) return x8 ? 2 : 3;   /* immediate, index width       */
	return ModeSize[mode];
}

int dyn_op_is_block_end(uint8_t op)
{
	switch (op) {
	/* conditional/unconditional branches */
	case 0x10: case 0x30: case 0x50: case 0x70:
	case 0x80: case 0x90: case 0xB0: case 0xD0: case 0xF0:
	case 0x82:                              /* BRL */
	/* jumps */
	case 0x4C: case 0x5C: case 0x6C: case 0x7C: case 0xDC:
	/* subroutine calls */
	case 0x20: case 0x22: case 0xFC:
	/* returns */
	case 0x40: case 0x60: case 0x6B:        /* RTI RTS RTL */
	/* software interrupts */
	case 0x00: case 0x02:                   /* BRK COP */
	/* mode / flag changes (swap the live opcode table) */
	case 0x28: case 0xC2: case 0xE2: case 0xFB: /* PLP REP SEP XCE */
	/* halts */
	case 0xCB: case 0xDB:                   /* WAI STP */
	/* block moves (self-repeating; decrement-and-reenter PC) */
	case 0x44: case 0x54:                   /* MVP MVN */
		return 1;
	default:
		return 0;
	}
}

int dyn_discover_block(const uint8_t *code, uint32_t start_pc,
                       int m8, int x8, DynBlock *b)
{
	uint32_t off = 0;
	int n = 0;
	uint8_t op = 0;

	/* M/X can't change mid-block: every mode-changing op is a block-ender,
	 * so (m8,x8) are constant across the whole scan. */
	for (;;) {
		op = code[off];
		off += (uint32_t)dyn_op_length(op, m8, x8);
		n++;
		if (dyn_op_is_block_end(op))
			break;
		if (n >= DYN_BLOCK_MAX_INSNS || off >= DYN_BLOCK_MAX_BYTES) {
			op = 0;   /* length-capped, not a real ender */
			break;
		}
	}

	b->start_pc = start_pc;
	b->length   = (uint16_t)off;
	b->n_insns  = (uint16_t)n;
	b->m8       = (uint8_t)(m8 ? 1 : 0);
	b->x8       = (uint8_t)(x8 ? 1 : 0);
	b->end_op   = op;
	b->hits     = 0;
	return n;
}

/* ---- block cache: fixed open-addressing hash --------------------------- */
#define DYN_CACHE_SLOTS 8192   /* power of two */
#define DYN_CACHE_MASK  (DYN_CACHE_SLOTS - 1)

static DynBlock  cache_slot[DYN_CACHE_SLOTS];
static uint8_t   cache_used[DYN_CACHE_SLOTS];
static unsigned  cache_count;

static uint32_t dyn_key(uint32_t pc, int m8, int x8)
{
	/* fold the 24-bit PC with the two mode bits */
	return (pc << 2) | ((uint32_t)(m8 ? 1 : 0) << 1) | (uint32_t)(x8 ? 1 : 0);
}

static uint32_t dyn_hash_key(uint32_t k)
{
	/* Knuth multiplicative */
	return (k * 2654435761u) & DYN_CACHE_MASK;
}

void dyn_cache_init(void)
{
	memset(cache_used, 0, sizeof(cache_used));
	cache_count = 0;
}

static int dyn_slot_matches(unsigned i, uint32_t pc, int m8, int x8)
{
	return cache_used[i] &&
	       cache_slot[i].start_pc == pc &&
	       cache_slot[i].m8 == (m8 ? 1 : 0) &&
	       cache_slot[i].x8 == (x8 ? 1 : 0);
}

DynBlock *dyn_cache_find(uint32_t pc, int m8, int x8)
{
	uint32_t h = dyn_hash_key(dyn_key(pc, m8, x8));
	unsigned probes;
	for (probes = 0; probes < DYN_CACHE_SLOTS; probes++) {
		unsigned i = (h + probes) & DYN_CACHE_MASK;
		if (!cache_used[i])
			return 0;
		if (dyn_slot_matches(i, pc, m8, x8))
			return &cache_slot[i];
	}
	return 0;
}

DynBlock *dyn_cache_insert(const DynBlock *b)
{
	uint32_t h = dyn_hash_key(dyn_key(b->start_pc, b->m8, b->x8));
	unsigned probes;
	for (probes = 0; probes < DYN_CACHE_SLOTS; probes++) {
		unsigned i = (h + probes) & DYN_CACHE_MASK;
		if (!cache_used[i]) {
			cache_slot[i] = *b;
			cache_used[i] = 1;
			cache_count++;
			return &cache_slot[i];
		}
		if (dyn_slot_matches(i, b->start_pc, b->m8, b->x8))
			return &cache_slot[i];   /* already present */
	}
	return 0;   /* full */
}

unsigned dyn_cache_count(void)
{
	return cache_count;
}
