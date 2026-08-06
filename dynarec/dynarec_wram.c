/*
 * dynarec_wram.c -- WRAM code-page measurement. See dynarec_wram.h for why
 * this exists and what question it is meant to answer.
 */
#ifdef PIKO_DYNAREC_WRAMSTAT
/*
 * The whole body is gated. Makefile.zaurus builds the dynarec directory's .c
 * files by wildcard, so
 * without this the measurement's ~20 KB of tables would sit in the BSS of
 * every build including the shipping one, on a device with 51 MB of RAM. That
 * is also what makes the gate checkable: `size` shows WRAMSTAT's bss larger
 * than the others', so a build where the flag failed to reach the compiler
 * cannot pass for a build where it did.
 */
#include <stdio.h>
#include <string.h>

#include "dynarec_block.h"
#include "dynarec_wram.h"

int dyn_wram_on = 0;

/* Pages holding code, and how many distinct block starts each holds. The block
 * count is the cost of invalidating that page: it is how many translations
 * would have to be redone. */
static uint8_t  page_is_code[DYN_WRAM_PAGES];
static uint16_t page_blocks[DYN_WRAM_PAGES];
static uint32_t page_writes[DYN_WRAM_PAGES];

/* One bit per WRAM byte: "a block start has already been decoded here". Block
 * discovery is far too expensive to redo on all ~9M WRAM dispatches a minute,
 * and it only ever gives the same answer for the same address. 16 KB. */
static uint8_t seen_start[DYN_WRAM_SIZE / 8];

/* Pages already dirtied in the CURRENT frame -- the second write to a page is
 * free, because the block built from it is already gone. Counting raw writes
 * instead would overstate the cost of invalidation by however chatty the game
 * happens to be within a page. */
static uint8_t page_dirty_frame[DYN_WRAM_PAGES];

static unsigned long w_wram, w_code, w_bytes;
static unsigned long frames;
static unsigned long dirty_pages_total, retrans_total;
static unsigned long dirty_pages_max, retrans_max;
static unsigned long dirty_pages_frame, retrans_frame;
static unsigned long blocks_seen;

void dyn_wram_init(void)
{
	memset(page_is_code, 0, sizeof(page_is_code));
	memset(page_blocks, 0, sizeof(page_blocks));
	memset(page_writes, 0, sizeof(page_writes));
	memset(seen_start, 0, sizeof(seen_start));
	memset(page_dirty_frame, 0, sizeof(page_dirty_frame));
	w_wram = w_code = w_bytes = 0;
	frames = 0;
	dirty_pages_total = retrans_total = 0;
	dirty_pages_max = retrans_max = 0;
	dirty_pages_frame = retrans_frame = 0;
	blocks_seen = 0;
	fprintf(stderr, "DYN-WRAM: measuring code pages (%u pages of %u bytes)\n",
	        (unsigned)DYN_WRAM_PAGES, 1u << DYN_WRAM_PAGE_SHIFT);
}

void dyn_wram_saw_block(uint32_t pc, const uint8_t *code, int m8, int x8)
{
	uint32_t off, first, last, p;
	DynBlock b;

	if (!dyn_wram_on) return;
	if (!dyn_wram_offset_of(pc, &off)) return;

	if (seen_start[off >> 3] & (1u << (off & 7))) return;
	seen_start[off >> 3] |= (uint8_t)(1u << (off & 7));
	blocks_seen++;

	memset(&b, 0, sizeof(b));
	dyn_discover_block(code, pc, m8, x8, &b);
	if (b.length == 0) b.length = 1;

	first = off >> DYN_WRAM_PAGE_SHIFT;
	last  = (off + b.length - 1) >> DYN_WRAM_PAGE_SHIFT;
	if (last >= DYN_WRAM_PAGES) last = DYN_WRAM_PAGES - 1;
	for (p = first; p <= last; p++)
		page_is_code[p] = 1;
	/* The block is charged to the page it STARTS in: invalidating any page it
	 * covers costs exactly one re-translation of it, and charging every page
	 * would count a straddling block twice. */
	if (page_blocks[first] < 0xFFFF) page_blocks[first]++;
}

void dyn_wram_write_off(uint32_t off, unsigned n)
{
	uint32_t first, last, p;
	int touched_code = 0;

	w_wram++;
	w_bytes += n;

	first = off >> DYN_WRAM_PAGE_SHIFT;
	last  = (off + n - 1) >> DYN_WRAM_PAGE_SHIFT;   /* a word can straddle */
	if (last >= DYN_WRAM_PAGES) last = DYN_WRAM_PAGES - 1;

	for (p = first; p <= last; p++) {
		if (!page_is_code[p]) continue;
		touched_code = 1;
		page_writes[p]++;
		if (!page_dirty_frame[p]) {
			page_dirty_frame[p] = 1;
			dirty_pages_frame++;
			retrans_frame += page_blocks[p];
		}
	}
	if (touched_code) w_code++;
}

void dyn_wram_frame(void)
{
	if (!dyn_wram_on) return;
	frames++;
	dirty_pages_total += dirty_pages_frame;
	retrans_total     += retrans_frame;
	if (dirty_pages_frame > dirty_pages_max) dirty_pages_max = dirty_pages_frame;
	if (retrans_frame     > retrans_max)     retrans_max     = retrans_frame;
	if (dirty_pages_frame) memset(page_dirty_frame, 0, sizeof(page_dirty_frame));
	dirty_pages_frame = retrans_frame = 0;
}

void dyn_wram_report(void)
{
	unsigned p, code_pages = 0;
	unsigned long f = frames ? frames : 1;
	unsigned i, top[8], n_top = 0;

	for (p = 0; p < DYN_WRAM_PAGES; p++)
		if (page_is_code[p]) code_pages++;

	fprintf(stderr, "DYN-WRAM: %u code pages, %lu distinct block starts\n",
	        code_pages, blocks_seen);
	fprintf(stderr, "DYN-WRAM: %lu WRAM writes (%lu bytes), %lu of them onto code pages\n",
	        w_wram, w_bytes, w_code);
	/*
	 * These two are the answer. dirty pages/frame is how many invalidations a
	 * per-page scheme would perform; re-translations/frame is what a WHOLE-
	 * CACHE flush would cost instead, per event. If the first is small, the
	 * per-page reverse index is worth building; if it is near zero, flushing
	 * outright is fine and far simpler.
	 */
	fprintf(stderr, "DYN-WRAM: %lu frames, dirty code pages/frame avg %lu.%02lu max %lu\n",
	        frames, dirty_pages_total / f,
	        ((dirty_pages_total % f) * 100) / f, dirty_pages_max);
	fprintf(stderr, "DYN-WRAM: re-translations/frame avg %lu.%02lu max %lu\n",
	        retrans_total / f, ((retrans_total % f) * 100) / f, retrans_max);

	/* Which pages, so "every frame" can be told apart from "one trampoline". */
	memset(top, 0, sizeof(top));
	for (p = 0; p < DYN_WRAM_PAGES; p++) {
		if (!page_writes[p]) continue;
		for (i = 0; i < n_top; i++)
			if (page_writes[p] > page_writes[top[i]]) break;
		if (i == 8) continue;
		{
			unsigned j, end = (n_top < 8) ? n_top : 7;
			for (j = end; j > i; j--) top[j] = top[j - 1];
		}
		top[i] = p;
		if (n_top < 8) n_top++;
	}
	for (i = 0; i < n_top; i++) {
		fprintf(stderr, "DYN-WRAM:   page %03X ($%05X-$%05X) %lu writes, %u blocks\n",
		        top[i], top[i] << DYN_WRAM_PAGE_SHIFT,
		        (top[i] << DYN_WRAM_PAGE_SHIFT) + (1u << DYN_WRAM_PAGE_SHIFT) - 1,
		        (unsigned long)page_writes[top[i]], page_blocks[top[i]]);
	}
	fflush(stderr);
}

#endif /* PIKO_DYNAREC_WRAMSTAT */
