# Dynarec handoff — start here

Where this stands, what to do next, and the traps that have already cost time.
The full narrative is in `README.md` (sections 8c–16); this is the short form.

## Standing numbers

FF6, real Zaurus, 60s runs, emulated frames per wall second:

| | frames/s |
|---|---|
| interpreter (dynarec disarmed) | ~51.9 |
| **dynarec armed** | **~48.0** |

~7.5% overhead, down from ~15.4% at the start of the optimisation work.
5,698,515 blocks run, unchanged across every one of those steps — none of the
speedups changed what gets translated, only the machinery around it.

**The dynarec is still slower than the interpreter it exists to accelerate.**
That is the honest headline and it has not changed all session.

## The one thing that matters next

**Fallback density is the binding constraint.** Roughly half of the
instructions inside blocks call into the interpreter, and each call costs the
block's calling convention (spill, set `cpu->PC`, load fn address, `blx`,
reload) *on top of* the interpreter routine — where plain interpreter dispatch
is a table index and a call. So a block only pays for itself when it is mostly
native.

Six optimisations measured flat for this one reason: per-block register
pinning, declining all-fallback blocks, native branches, and caching
`cpu->Cycles` in a register all get amortised over one or two instructions and
vanish.

Two things change it, in this order:

1. **Native coverage of the hot opcodes.** From the profiler (share of all
   executed instructions): `CD` CMP abs **11.6%**, `A5` LDA dp **4.1%**, then
   the STA family (`9D`/`8F`/`8D`, ~2.3% combined). CMP abs is the obvious
   first move — it reuses `tr_probe_map`/`tr_commit_load` wholesale, so it is
   assembly of already-verified parts, and VERIFY covers it directly.
2. **Block linking**, so a block stops ending at every control transfer.

There is also a free two-memory-ops-per-block win left: a block always ends
with a fallback, which leaves `cpu->Cycles` correct in memory, and the emitted
code then reloads `TR_CYC` and the epilogue stores it straight back. Both dead.

## How to work on this

```sh
dynarec/run-on-zaurus.sh -m exec     -w 60 root@<ip>   # measure
dynarec/run-on-zaurus.sh -m verify   -w 90 root@<ip>   # correctness
dynarec/run-on-zaurus.sh -m profile  -w 90 root@<ip>   # opcode histogram
dynarec/run-on-zaurus.sh -m wramstat -w 90 root@<ip>   # WRAM code-page stats
SSH="sshpass -p zaurus ssh" SCP="sshpass -p zaurus scp"   # device is root/zaurus
```

Runtime knobs — all sweep on ONE boot with ONE binary, which is what made six
wrong hypotheses cheap instead of six rebuilds:

| env var | effect |
|---|---|
| `PIKO_DYN_EXEC=1` | arm block execution |
| `PIKO_DYN_STUB=1` | `dyn_exec_step` returns immediately (ablation) |
| `PIKO_DYN_SLOTS=N` | cache slots actually touched |
| `PIKO_DYN_MIN_NATIVE=N` | refuse blocks less than N% native |
| `PIKO_DYN_WRAM_BLOCKS=1` | translate WRAM blocks (off: costs 24%) |
| `PIKO_MAX_SECONDS=N` | self-imposed deadline — **the only way to stop it** |

Build gates: `EXEC=1`, `VERIFY=1`, `PROFILE=1`, `WRAMSTAT=1`, `BLOCKSTATS=1`,
`NOCOUNT=1`. **Build all of them after any change** — `EXEC=1` has compiled
cleanly while the default build did not, because `dynarec_exec.cpp` is
compiled by wildcard into every build.

## Traps that have already cost real time

- **Verify an optimisation was actually emitted.** `static __inline` at `-Os`
  is a hint GCC declined; a commit "inlining the WRAM test" was a silent no-op
  that measured flat and nearly counted as a disproved hypothesis. Use
  `objdump -d PocketSNES-debug --no-show-raw-insn` and
  `grep -c 'bl.*<fn>'`. Use `__attribute__((always_inline))` when it must
  happen.
- **Ablate before attributing.** Four plausible mechanisms for the ~20%
  overhead were wrong. Turn the suspect off and measure before optimising it.
  Check the arithmetic too: 23M lookups × ~20 instructions is ~1% of a 60s
  run, never 20%.
- **`blocks run` is not a speed metric.** It tripled once while throughput fell
  24%. Frames/s is the only comparable number, which is why it is counted in
  every build outside any dynarec gate.
- **Always `make clean`.** The Makefile does not track the dynarec `-D` flags,
  so switching gates silently reuses objects. Three hardware measurements were
  thrown away before this was found; `run-on-zaurus.sh` now cleans always.
- **Check the deploy landed.** The board drops off WiFi mid-`scp`; a stale
  binary has been diagnosed against more than once. Every run prints a
  `PIKO build <date> <time>` stamp — read it first.
- **The device has no `kill`, no `timeout`, no `killall`.** `PIKO_MAX_SECONDS`
  is the only way a run ends. `matchbox-fbrun` is mandatory when X is up.
- **VERIFY cannot see everything.** It diffs single opcodes, so it cannot see a
  stale cached block, and it excludes PC so it cannot check a branch target.
  For branches it deliberately checks the not-taken case only — the taken path
  calls the interpreter, and re-running it would fire `CPUShutdown` →
  `APU_EXECUTE1()`, a global side effect.

## Feature state

- **WRAM blocks + write-invalidation**: implemented and correct (1,896
  invalidations in 1,896 events, FF6 survives). **Off by default** — costs 24%
  at current native coverage. `PIKO_DYN_WRAM_BLOCKS=1`.
- **Native opcodes**: CLC, SEC, NOP, INX/INY/DEX/DEY, LDA abs, BPL/BNE/BEQ.
  Branches do not end a block; the not-taken path continues through.
- **Known limitation**: invalidation is block-granular, so a block that writes
  into its own page and keeps executing modified bytes within that same block
  runs the stale snapshot to the block's end. The interpreter re-fetches every
  opcode and would not. No game seen here needs it.
