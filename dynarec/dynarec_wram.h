/*
 * dynarec_wram.h -- WRAM code-page measurement (design README, step 8c).
 *
 * The dynarec currently refuses to translate anything running out of WRAM
 * (dyn_pc_in_wram), and the profiler says that is where 9.35M dispatches a
 * minute go. Lifting the refusal means invalidating cached blocks when the
 * game writes over the code they were built from -- and the shape of that
 * invalidation depends entirely on ONE number nobody has yet:
 *
 *     how often does a write land on a WRAM page that holds code?
 *
 *   rare      -> flush the whole block cache on such a write. Trivial.
 *   frequent  -> a flush costs re-translating everything (~1,100 blocks), so
 *                the invalidation has to be per-page, with a reverse index
 *                from page to the slots built from it.
 *
 * Guessing is expensive here because the check lands in the write path, which
 * is the hottest code in the emulator, so it gets measured first. This whole
 * file is compiled only under WRAMSTAT builds (-DPIKO_DYNAREC_WRAMSTAT) and
 * armed only by PIKO_DYN_WRAM=1; default builds do not contain it.
 *
 * WHAT COUNTS AS A CODE PAGE. A page (256 bytes) is marked the first time a
 * block start is dispatched from it -- i.e. exactly the pages dyn_exec_step
 * currently turns away. Marking uses the real block decoder, so a block that
 * spans a page boundary marks both pages; a start-page-only approximation
 * would under-report precisely the straddling case that makes invalidation
 * awkward.
 *
 * WHERE THE WRITE HOOK GOES. Both S9xSetByte/S9xSetWord (getset.h) and
 * REGISTER_2180 (ppu.h). DMA reaches WRAM through the $2180 port without ever
 * touching the byte path -- the same split that hid a watchpoint bug in
 * arm-snesrec, where a watch on the byte path reported a confident zero for a
 * region DMA was actively filling. Miss either and the answer is wrong in the
 * direction that looks encouraging.
 */
#ifndef PIKO_DYNAREC_WRAM_H
#define PIKO_DYNAREC_WRAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DYN_WRAM_SIZE       0x20000u                              /* 128K   */
#define DYN_WRAM_PAGE_SHIFT 8                                     /* 256 B  */
#define DYN_WRAM_PAGES      (DYN_WRAM_SIZE >> DYN_WRAM_PAGE_SHIFT)/* 512    */

extern int dyn_wram_on;

void dyn_wram_init(void);

/* A block start was dispatched at guest `pc` (which is in WRAM), with `code`
 * the host pointer to its first opcode byte. Decodes the block once per
 * distinct start address and marks every page it covers. */
void dyn_wram_saw_block(uint32_t pc, const uint8_t *code, int m8, int x8);

/* Record a write of `n` bytes at WRAM offset `off` (0..0x1FFFF). Out of line:
 * the inline part below is only the WRAM test, so non-WRAM writes -- the
 * overwhelming majority -- cost one predictable compare and nothing else. */
void dyn_wram_write_off(uint32_t off, unsigned n);

void dyn_wram_frame(void);
void dyn_wram_report(void);

/*
 * Guest address -> WRAM offset, mirrors included. This is dyn_pc_in_wram's
 * mapping (banks $7E/$7F in full, plus $0000-$1FFF of $00-$3F and $80-$BF,
 * which alias the FIRST 8K), and it has to stay that way: a block cached from
 * $00:1500 and a write to $7E:1500 are the same bytes.
 */
static __inline int dyn_wram_offset_of(uint32_t addr, uint32_t *off)
{
	uint8_t  bank = (uint8_t)((addr >> 16) & 0xFF);
	uint16_t o    = (uint16_t)(addr & 0xFFFF);

	if (bank == 0x7E) { *off = o; return 1; }
	if (bank == 0x7F) { *off = 0x10000u | o; return 1; }
	if (o < 0x2000 && (bank < 0x40 || (bank >= 0x80 && bank < 0xC0))) {
		*off = o;
		return 1;
	}
	return 0;
}

/* ---- write-invalidation (EXEC builds) --------------------------------- *
 *
 * The measurement above said per-page, not global flush: 0.72 dirty code
 * pages per frame against ~1,100 cached blocks, so a flush per event would
 * cost three orders of magnitude more than it saves. See README 8c.
 *
 * dyn_code_page[p] is nonzero if any cached block was translated from page p.
 * It is the whole fast path: almost every WRAM write finds a zero here and
 * costs one byte load. It is deliberately never cleared on invalidation --
 * a stale 1 only buys a walk of some empty lists, and the block is usually
 * re-translated into the same page moments later anyway.
 */
extern uint8_t dyn_code_page[DYN_WRAM_PAGES];

/*
 * Nonzero only while WRAM blocks are actually being cached, so a build with
 * the feature off pays one global load per write rather than the address
 * decode below.
 *
 * Do not read a number into this. The disarmed EXEC binary measured 50.25
 * frames/s before this early-out and 50.57 after, against 51.66 for a build
 * with no hook compiled at all -- but three runs of the SAME armed config came
 * out 41.81 / 40.87 / 41.15, so run-to-run spread here is ~2%. The early-out
 * is right on principle and its effect is inside the noise; the honest claim
 * is that the hook is not measurably expensive either way, not that it cost
 * 2.7% and was fixed.
 */
extern int dyn_wram_tracking;

/* Discard every cached block built from `page`. Out of line in
 * dynarec_exec.cpp, which owns the cache. */
void dyn_wram_invalidate_page(unsigned page);

/* Discard the entire cache. For the bulk rewrites that do not come through
 * the byte path at all: reset (memsets WRAM) and savestate load (reads 128K
 * straight over it). Neither is a write in the sense the hook understands,
 * and both change every byte of code in WRAM. */
void dyn_wram_flush_all(void);

/* `off` is already a WRAM offset: the $2180 port path has one directly. */
static __inline void dyn_wram_touch(uint32_t off, unsigned n)
{
	uint32_t first, last;

#ifdef PIKO_DYNAREC_WRAMSTAT
	if (dyn_wram_on) dyn_wram_write_off(off, n);
#endif
	first = off >> DYN_WRAM_PAGE_SHIFT;
	last  = (off + n - 1) >> DYN_WRAM_PAGE_SHIFT;   /* a word can straddle */
	if (dyn_code_page[first]) dyn_wram_invalidate_page(first);
	if (last != first && last < DYN_WRAM_PAGES && dyn_code_page[last])
		dyn_wram_invalidate_page(last);
}

static __inline void dyn_wram_write(uint32_t addr, unsigned n)
{
	uint32_t off;
	if (!dyn_wram_tracking) return;
	if (!dyn_wram_offset_of(addr, &off)) return;
	dyn_wram_touch(off, n);
}

#ifdef __cplusplus
}
#endif

#endif /* PIKO_DYNAREC_WRAM_H */
