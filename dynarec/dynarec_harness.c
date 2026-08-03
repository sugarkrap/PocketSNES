/*
 * dynarec_harness.c -- see dynarec_harness.h.
 */
#include "dynarec_harness.h"

uint32_t dyn_hash(const void *data, unsigned len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint32_t h = 2166136261u;   /* FNV-1a offset basis */
	unsigned i;
	for (i = 0; i < len; i++) {
		h ^= p[i];
		h *= 16777619u;         /* FNV prime */
	}
	return h;
}

int dyn_cpu_diff(const DynCpuSnap *a, const DynCpuSnap *b, const char **whatp)
{
	const char *what = 0;

	if      (a->PC     != b->PC)     what = "PC";
	else if (a->PB     != b->PB)     what = "PB";
	else if (a->DB     != b->DB)     what = "DB";
	else if (a->P      != b->P)      what = "P";
	else if (a->A      != b->A)      what = "A";
	else if (a->D      != b->D)      what = "D";
	else if (a->S      != b->S)      what = "S";
	else if (a->X      != b->X)      what = "X";
	else if (a->Y      != b->Y)      what = "Y";
	else if (a->cycles != b->cycles) what = "cycles";
	else if (a->ram_hash != b->ram_hash) what = "ram";

	if (what) {
		if (whatp) *whatp = what;
		return 0;
	}
	return 1;
}
