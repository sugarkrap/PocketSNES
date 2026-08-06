# PocketSNES dynamic recompiler (dynarec) — design & plan

A 65816 → ARMv5TE dynamic recompiler for PocketSNES on the Sharp Zaurus
(piko). This document is the map: why it exists, why it's hard, how it's
built, the order we build it in, and — most importantly — the correctness
discipline that makes an effort this size survivable.

Read this before touching anything under `dynarec/`.

---

## 1. Why

The 65816 CPU core in this port is a **pure interpreter**: `cpuexec.cpp` runs
`for(;;) { … (*ICPU.S9xOpcodes[*CPU.PC++].S9xOpcode)(…); … }` — one indirect
call plus a big helper per emulated instruction, *every* time an instruction
runs. No native code is ever generated. There is no assembly core either (the
Makefile compiles only `.cpp`/`.c`).

On a ~200–400 MHz PXA255 (ARMv5TE, soft-float, no dynarec) that per-instruction
overhead is a real ceiling for CPU-heavy scenes.

**Important context before optimising the CPU:** the *first* big win on this
device was on the video side — per-page dirty-row skipping in the framebuffer
blit took the FF6 intro from ~4 to ~21 FPS (~5×), which proved the uncached
framebuffer writes, **not** the interpreter, were the initial bottleneck. So a
dynarec attacks the *second* bottleneck. By Amdahl, always confirm the CPU is
the current wall (per-frame CPU-vs-render-vs-blit profiling) before assuming a
faster core is the answer for a given workload.

A dynarec translates blocks of guest 65816 code into native ARM **once**,
caches them, and runs the native code directly thereafter — trading the
interpreter's per-instruction dispatch for near-native execution on hot paths.

---

## 2. Why a 65816 → ARMv5 dynarec is genuinely hard

- **Mode-dependent opcodes.** The 65816's `M` (accumulator/memory width) and
  `X` (index width) status bits, plus `E` (emulation) mode, change what an
  instruction *means and how long it is*: `LDA #` is 2 or 3 bytes depending on
  `M` at that moment. A translated block is only valid for one `(M,X,E)` mode,
  and any `REP`/`SEP`/`XCE`/`PLP`/mode-changing op ends a block.
  - *Silver lining:* snes9x already models this by **swapping the whole
    `ICPU.S9xOpcodes[]` table** when the mode changes. So we get the current
    mode for free as "which opcode table is live", and can key blocks on it.

- **Memory is not memory.** Every load/store goes through snes9x's address
  dispatch (`S9xGetByte`/`S9xSetByte`/…, `getset.h`, via `Memory.Map[]`). Large
  regions are MMIO with **side effects** — PPU/APU/DMA/HDMA registers. Generated
  code can only inline the RAM/ROM fast paths; MMIO must call back into the C
  handlers. That callback boundary is most of the complexity.

- **Cycle & event handshake.** `S9xMainLoop` counts cycles (`CPU.Cycles +=
  MemSpeed…`) and, at instruction boundaries, services NMI/IRQ, `DO_HBLANK_CHECK`,
  HDMA and timers, breaking out of the loop on `SCAN_KEYS_FLAG` (end of frame).
  Recompiled code must keep cycle counts exact and poll for these at safe
  points or games break in subtle, awful ways.

- **Coprocessors & self-modifying code.** SA-1, DSP-1, SuperFX, C4 have their
  own paths; RAM-executed code and bank remaps must invalidate cached blocks.

- **Little prior art.** Snes9x never shipped a dynarec; bsnes/higan never
  recompiled. There is no canonical open 65816→ARM JIT to copy. We are closer
  to pioneering than porting — exciting, but it means the **correctness harness
  is not optional**.

---

## 3. Architecture

```
        guest PC + live opcode-table (mode)
                     │
                     ▼
            ┌──────────────────┐   miss   ┌───────────────────────┐
            │  block cache     │ ───────► │  translator            │
            │  (PC,mode)→block │          │  walk ops to a block-  │
            └──────────────────┘          │  ender, emit ARM       │
                     │ hit                 │  (unknown op → call    │
                     ▼                     │   interpreter fn)      │
            ┌──────────────────┐          └───────────┬───────────┘
            │  execute native  │ ◄────────────────────┘
            │  block (ARM)     │
            └──────────────────┘
                     │ block end: update cycles, poll events (NMI/IRQ/HBLANK)
                     ▼
              back to dispatch
```

- **Block cache** — keyed on `(24-bit guest address, opcode-table id)`.
  A block is a straight-line run of guest instructions ending at the first
  control-flow or mode-changing op (branch, `JMP/JSR/RTS/RTI`, `REP/SEP/XCE`,
  interrupt), or a length cap.

- **Register ABI** — a translated block mirrors an interpreter opcode fn's
  signature: `void blk(SRegisters *reg, SICPU *icpu, SCPUState *cpu)`
  (`r0/r1/r2`), so interpreter fallbacks are a straight pass-through. Pinned for
  the block's duration: `r8/r9/r10` = reg/icpu/cpu base pointers, `r4/r5/r6` =
  `A/X/Y` (loaded by the prologue from `SRegisters`, spilled by the epilogue).
  `SRegisters` = `PC, PB, DB, P, A, D, S, X, Y` (`pair` = 16-bit).

- **Flags are NOT in `P` during execution** (crucial). snes9x keeps the live
  N/Z/C/V in four separate `SICPU` fields — `_Carry`, `_Zero`, `_Negative`,
  `_Overflow` — with their own encodings (`CLC ⇒ icpu->_Carry = 0`;
  `_Zero == 0` *means* Z set; `_Negative` holds a byte whose bit 7 is N). `P` is
  only packed/unpacked at interrupt/loop boundaries. Native ops therefore write
  those `SICPU` fields (in memory) to stay consistent with interpreter
  fallbacks. A later optimisation can map the four to ARM's own NZCV condition
  flags; correctness first means mirroring the interpreter exactly for now.

- **Emitter** (`dynarec_arm.h`) — writes ARMv5TE words into an RWX buffer;
  `__builtin___clear_cache` flushes them (ARM I-cache/D-cache are **not**
  coherent — skipping this runs stale code).

- **Hybrid fallback** — the linchpin that makes this incremental. Any opcode we
  haven't taught the translator to emit yet is compiled as **a call to the
  existing interpreter function** for that opcode. So the emulator is 100%
  correct from the very first translated block; we light opcodes up one at a
  time and watch FPS climb while correctness never regresses.

- **Memory** — inline the RAM/ROM fast path (bounds-check the mapped region,
  direct access); MMIO ranges call the C `S9xGetByte`/`S9xSetByte` handlers so
  their side effects still fire.

- **Translation (instruction) cache** — the RWX code buffer (Step 0) + the
  block cache (Step 1) together *are* the instruction cache: guest block →
  translated native code. It must be **invalidated** when the underlying guest
  bytes change — SNES games run code from WRAM (profiling confirms hot blocks in
  bank `$7E`, below) and remap banks — or the CPU would run stale translations.
  Track which guest pages a block was translated from and drop affected blocks
  on writes to those pages / on bank remaps.

- **AOT dry-run (pre-pass)** — before/while running, statically walk the ROM to
  pre-populate the cache and "feed the loop". This is a **control-flow
  traversal**, not a linear sweep: seed a worklist from the reset/NMI/IRQ
  vectors, decode each block to its ender, enqueue its statically-known
  successors (branch/call/jump targets), repeat. It cannot be a naive
  decode-every-byte pass — ROM data aliases as valid opcodes — and it cannot be
  complete (indirect jumps, computed targets, RAM code are undecidable
  statically). So it is a *seed*: the runtime JIT still fills whatever the
  static pass can't reach. Hybrid AOT + JIT. The Step 1 decoder is exactly the
  machinery this pre-pass runs on.

---

## 4. Build order (and current status)

Each step is a shippable, individually-verifiable commit. Nothing is enabled by
default until it's proven; the shipped binary stays the known-good interpreter.

- [x] **Step 0 — Codegen foundation.** Prove the PXA255 can emit-and-run native
  code at all (RWX mmap + correct ARMv5 encodings + I-cache flush + host call
  convention). `dynarec_arm.h` emitter + `S9xDynSelfTest()`.
  **Done — PASSes on real hardware** (`f(3,4)=10`, `g()=42`).
- **Step 1 — Block cache skeleton (recompile nothing) + correctness harness.**
  - [x] **Decoder** (`dynarec_block.*`): mode-aware 65816 instruction lengths
    (reusing snes9x's `AddrModes[256]`) + block-ender classification.
  - [x] **Block discovery**: walk a straight-line run to the first block-ender.
  - [x] **Block cache**: open-addressing hash keyed on `(PC, M, X)` with hit
    counting (hotness).
  - [x] **Harness core** (`dynarec_harness.*`): `DynCpuSnap` + `dyn_cpu_diff`
    (incl. cycles) + WRAM FNV hash.
  - [x] Offline unit tests for all of the above, run under `PIKO_JIT_SELFTEST`.
    **PASS on real hardware.**
  - [x] **Live hotness profiler** (`make PROFILE=1`, `PIKO_DYN_PROFILE=1`):
    observation-only opcode histogram + hot-block counter hooked into the CPU
    dispatch, default-compiled-out. **Run on FF6 — findings in section 7.**
  - [ ] **Run-both-and-diff** deferred to Step 3: it needs recompiled code to
    compare the interpreter against, so it lands as the per-opcode gate there
    (capture `DynCpuSnap` live, run block both ways, diff).
- [x] **Step 2 — Emitter register ABI + prologue/epilogue.** Guest `A/X/Y/P`
  pinned to `r4/r5/r6/r7`, regfile base in `r8`; block is `void blk(void*
  regfile)` (r0), prologue `push {r4-r8,lr}` + `LDRH` loads, epilogue `STRH`
  spills + `pop {r4-r8,pc}`. Added `LDRH/STRH/ADD#imm/MOV reg` encoders.
  **Done — round-trip PASSes on real hardware.**
- **Step 3 — Translate opcodes** (in progress).
  - [x] Translator skeleton (`dynarec_translate.cpp`) with the interpreter-
    matching block ABI + prologue/epilogue; native emission for **CLC/SEC/NOP**
    (CLC is a top-3 hottest opcode). Offline self-test translates `CLC;SEC;CLC`
    and runs it — **PASS on hardware** (`_Carry`, cycles, A/X/Y all correct).
  - [x] Hybrid interpreter-fallback emission: spill pinned A/X/Y → struct, set
    `cpu->PC` past the opcode, `BLX` the interpreter fn (r0/r1/r2 = reg/icpu/cpu),
    reload A/X/Y. reg/icpu/cpu (r8/r9/r10) + pcbase (r7) are callee-saved so the
    C fn preserves them. Self-test runs native CLC / fallback / native SEC —
    **PASS on hardware** (A round-trips through the fallback, flags/cycles/PC
    correct). New encoders: `MOV32` (4-insn const), `ORR #imm`, `BLX`.
  - [x] **Live run-both-and-diff** (`make VERIFY=1`, `PIKO_DYN_VERIFY=1`,
    `dynarec_verify.cpp`): for each translatable opcode in the *real* CPU loop,
    run the generated block on a scratch copy and diff against the interpreter's
    effect on the live state. Interpreter stays authoritative; generated code
    only touches the copy. **Immediately earned its keep** — caught native CLC
    diverging on the `Z` flag: the flag fields are 1-byte (`uint8_32 == unsigned
    char`) and adjacent, and a word `STR` to `_Carry` was clobbering
    `_Zero/_Negative/_Overflow`. Fixed with `STRB`; **0 divergences** after, on
    the same code path that failed immediately before.
  - [x] Index inc/dec **INX/INY/DEX/DEY**, both widths (8-bit low-byte-only +
    16-bit with wrap), exact `SETZN8/16` (incl. the 16-bit `_Zero`=0/1 via
    conditional `MOV`, `_Negative`=high byte) and `WaitAddress`/cycle handling.
    The validator is now **mode-aware** (a stub per opcode per M/X mode, picked
    from the runtime P). Verified offline (10 hand-derived cases) **and** live
    (0 divergences). New encoders: `AND/SUB #imm`, `ORR/CMP`, cond `MOV`,
    shifted `MOV`.
  - [ ] Translate the rest of the hot set (`LDA/STA/STZ`, branches, `CMP/CPY`,
    `INC/DEC A`, `ASL A`, `XBA`, `JSR/RTS`, `PHA/PLA`) — memory + control-flow
    ops are the larger remaining effort — each gated by the validator.
  - [~] Block-driven execution (`make EXEC=1`, `PIKO_DYN_EXEC=1`,
    `dynarec_exec.cpp`): the recompiler drives the CPU. Plumbing **written and
    builds** (default byte-identical); **not yet validated end-to-end** — needs
    a run (QEMU or device). A translated block runs a straight-line guest run to
    its ender and updates `cpu->PC`; the main loop skips the single-instruction
    dispatch when a block ran (event servicing SA1/HBLANK/IRQ now at block
    granularity). Cycle parity via a per-instruction `cpu->Cycles += MemSpeed`
    add inside the block. ROM-only for now (`!dyn_pc_in_wram(pc)`) — RAM code
    can self-modify and needs cache invalidation we don't have yet. NO speed win
    expected until native opcode coverage grows (blocks are still mostly
    fallbacks). 1 MiB code arena, flushed wholesale when full.
  - [ ] **LDA/STA with inlined memory fast paths** -- the hot set from
    section 7 that is still entirely interpreter fallback. Design, from
    reading getset.h and cpuops.cpp:

    `S9xGetByte` is already a two-tier lookup, and only the fast tier needs
    inlining:

        block = (addr >> MEMMAP_SHIFT) & MEMMAP_MASK   /* shift 12, mask 0xFFF */
        p     = Memory.Map[block]
        if (p >= (uint8 *)CMemory::MAP_LAST) {         /* direct memory */
            cpu->Cycles += Memory.MemorySpeed[block];
            if (Memory.BlockIsRAM[block])
                cpu->WaitAddress = cpu->PCAtOpcodeStart;
            return p[addr & 0xFFFF];
        }
        /* else: MAP_PPU/MAP_CPU/... -- do NOT inline, fall back */

    So the emitted sequence is: compute the effective address, index Map,
    one compare against MAP_LAST, and either the inline load or a branch to
    the existing tr_emit_fallback() for that opcode. The slow tier keeps its
    single implementation; we are not reimplementing PPU/CPU register access
    in ARM, which is where a rewrite of this would go wrong.

    The win that makes this worth more than shaving cycles: for `LDA abs`
    ($AD), `STA abs` ($8D) and friends the OPERAND IS A COMPILE-TIME
    CONSTANT -- we are translating a known PC, so the 16-bit absolute address
    is read out of the ROM at translation time and embedded as an immediate.
    Only the bank has to come from a register (`reg->DB` for absolute,
    `reg->PB` for long). That removes the operand fetch entirely, not just
    its dispatch overhead.

    Correctness constraints that must hold, since blocks run as the live CPU:
      - cycle parity: the interpreter adds MemorySpeed[block] per access via
        VAR_CYCLES, plus whatever Absolute()/LDA8() add for the operand
        fetch. VERIFY diffs cycles, so this is checkable rather than hoped.
      - CPU_SHUTDOWN: BlockIsRAM blocks set cpu->WaitAddress; skipping that
        changes idle-loop detection and therefore timing.
      - **probe before charging cycles.** The MAP_LAST test must come before
        any Cycles update. If the address turns out to be a handler and we
        bail to the interpreter fallback, that fallback runs Absolute() and
        LDA8() itself and charges MemSpeedx2 + MemorySpeed again -- so any
        cycles added before the bail are counted twice. Split the sequence
        into a side-effect-free probe (block, Map load, compare, bail) and a
        commit (Cycles += MemSpeedx2, Cycles += MemorySpeed[block],
        BlockIsRAM -> WaitAddress, the byte load).
      - the block must set `cpu->PCAtOpcodeStart = pcbase + off` before the
        access, because the CPU_SHUTDOWN path reads it and the interpreter
        sets it per opcode in cpuexec.cpp. Both VAR_CYCLES and CPU_SHUTDOWN
        are on in this build (pocketsnes/linux/port.h).
      - **the 16-bit path is NOT two byte accesses.** S9xGetWord does ONE Map
        lookup and charges `MemorySpeed[block] << 1`, does ONE BlockIsRAM
        check, and then loads two bytes off the same base pointer
        (FAST_LSB_WORD_ACCESS is `#undef`d in linux/port.h, so it is two LDRBs
        rather than an unaligned LDRH -- which is what we want on ARMv5
        anyway). Emitting two byte fast paths instead would charge
        `MemorySpeed[b1] + MemorySpeed[b2]`, do two lookups, and write
        WaitAddress twice: identical only when both bytes land in the same
        block, and wrong in cycles regardless. Emit one probe, one doubled
        cycle charge, one BlockIsRAM check, two LDRBs.
      - S9xGetWord special-cases `Address == 0x00001fff` and falls back to two
        S9xGetByte calls, because the word straddles a Map block edge at the
        top of the WRAM mirror. The fast path must bail to the interpreter for
        that address rather than assume one block covers both bytes.
      - **VERIFY must be extended FIRST -- it cannot cover these opcodes as
        written.** dyn_verify_init() builds one stub per opcode from a 1-byte
        array (`uint8_t op[1] = { want[i] }`). Translating LDA abs against
        that reads code[off+1..2], past the end of the array; and since the
        design bakes the operand in as an immediate, a single per-opcode stub
        cannot stand for an instruction whose address differs at every site.
        The existing "53,908 checks, 0 diverged" result therefore covers only
        the seven operand-less opcodes.

        The fix is to translate the ACTUAL instruction at the live PC into a
        scratch buffer in dyn_verify_before(), rather than looking up a
        prebuilt stub: the bytes are right there at cpu->PC, the mode is
        known, and the harness already runs the result against a scratch CPU
        copy. Slower per instruction, but VERIFY is a debug gate and 60s on
        the board is plenty. Do this before adding a single operand-bearing
        opcode -- every opcode landed so far was landed with the net under it,
        and that is why they are trusted. VERIFY on hardware is cheap
        (53,908 checks in 60s) and is what proved the existing opcode set
        correct on a real PXA270.

  - [ ] Branches as native conditionals + block linking, so a loop stops
    re-entering the hash lookup on every iteration.
  - [ ] Extend the profiler to report native-vs-fallback instruction counts
    and WRAM-skip counts, so the next move after that is measured. Note it
    cannot be armed together with EXEC (see section 9).
  - [ ] Idle-loop detection for the boot spin (`$FA:00F9` CLC/LDA/BPL).
    Deliberately LAST: it deletes work rather than speeding it up, so doing
    it first would mask how much the other items are actually worth.
- [ ] **Step 4 — Memory fast paths** (inline RAM/ROM; MMIO → C handlers) and
  ALU/addressing modes.
- [ ] **Step 5 — Block linking / branch chaining**, cycle-accurate event
  polling at boundaries, `CPU_SHUTDOWN`-style idle-loop handling.
- [ ] **Step 6 — Coprocessors & invalidation** (SA-1 etc., SMC/bank-remap block
  invalidation), then tuning.

---

## 5. Correctness discipline (non-negotiable)

A dynarec that is 99.9% correct is 100% useless — one wrong flag ten million
instructions in is an unshippable, undebuggable heisenbug. The rules:

1. **The interpreter is the oracle.** Its behaviour is ground truth. The
   recompiler must produce byte-identical CPU state.
2. **Run-both-and-diff harness before any codegen** (Step 1). Every new opcode
   is added *behind* it: translate, run both paths, diff `SRegisters` +
   `CPU.Cycles` + RAM, and only then move on. First divergence = stop and fix.
3. **Cycles are state.** An instruction that computes the right value in the
   wrong number of cycles is a bug — timing drives IRQ/HBLANK/HDMA.
4. **Default off.** Until a step is proven, the shipped build is the
   interpreter. Dynarec paths stay behind a flag/env until they're green.
5. **FF6 is the first corpus.** The annotated disassembly at
   <https://github.com/everything8215/ff6> is ground-truth 65816 for our
   primary test game — use it to see which opcodes/modes the game actually
   exercises (prioritise those) and to sanity-check semantics.

---

## 6. How to build & test

```sh
# cross toolchain in PATH (piko checkout as a sibling; override PIKO_DIR)
export PATH="../../../../../../toolchain/x-tools/arm-unknown-linux-uclibcgnueabi/bin:$PATH"
make -f Makefile.zaurus PIKO_DIR=/path/to/piko
```

**Runtime-codegen self-test** — no framebuffer or input needed, so it runs over
a plain SSH shell without disturbing the desktop:

```sh
PIKO_JIT_SELFTEST=1 ./PocketSNES      # prints "PIKO-JIT selftest: PASS", exits
```

It runs at the very top of `mainEntry()` (before `sal_Init()`), JITs two tiny
functions and checks their results. Default runs (env unset) are byte-for-byte
unaffected.

---

## 7. Profiling findings (FF6, `PIKO_DYN_PROFILE`)

First real run — FF6 boot into the opening, ~17M instructions observed on
hardware. Two regimes:

- **Boot idle-loop.** For the first ~9M instructions one block dominates
  everything: `$FA:00F9` (`CLC / LDA $xxxx / BPL` — a spin-wait on a hardware
  flag), with `CLC`/`LDA`/`BPL` each at exactly 33.3%. Classic emulator idle
  loop. The big win here isn't translating it fast, it's **detecting the idle
  loop and skipping to the next event** (cf. snes9x's `CPU_SHUTDOWN`); worth a
  dedicated pass.

- **Game logic.** Once running, the hot-opcode set is small and clear — the
  **translate-first list for Step 3** (covers the large majority of dynamic
  instructions):
  - loads: `LDA` (`AD/A5/B7/B9`)
  - stores: `STA` (`8D/9D/8F`), `STZ` (`9C`)
  - flags/branches: `CLC`, `BPL/BNE/BEQ/BMI`
  - compares: `CMP` (`CD`), `CPY` (`C4/C0`)
  - inc/dec & index: `INY/INX/DEX/DEY/DEC` (`C8/E8/CA/C6`), `ASL` (`0A`)
  - misc: `XBA` (`EB`), `JSR/RTS`, `PHA/PLA`

- **Mode.** Overwhelmingly `M=1,X=0` (8-bit accumulator, 16-bit index) at boot,
  mixing `M=0` later. Start with both accumulator widths; index-16 is common.

- **Code in RAM.** Hot blocks appear in bank `$7E` (WRAM) — FF6 executes code
  it copies into RAM. Confirms the **translation cache must invalidate on
  writes** to code pages (see architecture). Not an afterthought.

  Note that "RAM" is not just `$7E/$7F`: the first 8K of WRAM is mirrored into
  `$0000-$1FFF` of every bank in `$00-$3F` and `$80-$BF`, and that mirror is
  where the interesting cases live. FF6's NMI vector is a JML trampoline at
  `$001500` — bank `$00` — which the game reinstalls every frame. Use
  `dyn_pc_in_wram()`; a bank-only test silently lets those blocks into the
  cache.

## 8b. First native-vs-fallback numbers (FF6, SL-C860, EXEC, 60s)

    5,784,831 blocks run, 1,109 translated, 0 flushes
    insns 6,656,188 native + 8,734,209 fallback = 15,390,397  (43% native)
    9,349,118 dispatches skipped as WRAM (self-modifying)

Translated set at the time: CLC/SEC/NOP/INX/INY/DEX/DEY + LDA abs.

**The WRAM number is the headline, and it reorders the roadmap.** 9.35M
dispatches were refused because the PC was in RAM, against 5.78M blocks
actually run -- so roughly 62% of all dispatch attempts never reach the
dynarec at all. FF6 executes more from WRAM than from ROM.

More opcodes improves the 43% inside the ROM blocks we do run. Block linking
improves dispatch for those same blocks. Neither touches the 62% we decline
outright. Section 7 already said the translation cache must invalidate on
writes and that it was "not an afterthought" -- that was a design opinion;
this is a measurement, and it says WRAM support with write-invalidation is
worth more than either of the other two.

43% native from one memory opcode plus six trivial ones is also higher than
expected, which suggests the remaining LDA/STA modes are worth having but are
not where the order-of-magnitude sits.

UNEXPLAINED, and it invalidates casual throughput comparisons: the block count
comes out at ~5.78M for runs of 45s, 60s AND 90s. It should scale with wall
time and does not, so something other than duration is bounding it. Do not
trust blocks/sec figures (including the "128k vs 96k" noted when LDA landed)
until this is understood.

## 8c. WRAM blocks: the measurement, and what it decided

The dynarec refuses to translate anything running out of WRAM, and 8b said
that is where most of the work is. Lifting the refusal needs block
invalidation, and the shape of that depends on one number: **how often does a
write land on a WRAM page that holds code?** That number was measured before
any of it was built, because the check lands in the write path -- the hottest
code in the emulator -- and getting the shape wrong means rewriting it twice.

`make WRAMSTAT=1` + `PIKO_DYN_WRAM=1` (dynarec_wram.[ch]) marks a 256-byte
WRAM page as holding code the first time a block start is dispatched from it,
using the real block decoder so a block spanning a page boundary marks both,
then counts writes against that map.

**The answer** (FF6, 90s on the device, 3,336 frames):

```
DYN-EXEC: FINAL 5785482 blocks run, 1109 translated, 0 flushes
DYN-EXEC: 14838992 dispatches skipped as WRAM (self-modifying)
DYN-WRAM: 17 code pages, 1479 distinct block starts
DYN-WRAM: 1682482 WRAM writes (2577686 bytes), 346701 of them onto code pages
DYN-WRAM: 3336 frames, dirty code pages/frame avg 0.72 max 1
DYN-WRAM: re-translations/frame avg 0.72 max 1
DYN-WRAM:   page 015 ($01500-$015FF) 346701 writes, 1 blocks
```

Two things fall out, and they point opposite ways:

**The prize is large.** 14.8M WRAM dispatches against 5.8M ROM blocks run --
WRAM is ~72% of all block dispatch, and every one of it is interpreted today.
Only 1,479 distinct block starts serve those 14.8M dispatches: ~10,000
executions per block. This is the single biggest remaining win.

**The cost is nearly nil, but only per-page.** 0.72 dirty code pages per
frame, never more than one in any frame. Note what that does NOT say: a write
lands on a code page 346,701 times, ~104 times per frame. Those collapse to
0.72 invalidations because they all hit the SAME page, and re-dirtying a page
whose block is already gone is free.

So the two strategies differ by three orders of magnitude:

  - **global flush** on any code-page write: 0.72 flushes/frame x ~1,100+
    cached blocks = roughly 800 re-translations per frame. Unaffordable.
  - **per-page invalidation**: 0.72 re-translations per frame. Free.

Per-page it is, with a reverse index (page -> slot list). Scanning all 16,384
cache slots per write is not an option; but note that the index barely has to
scale -- one page, holding one block, is the entire working set.

**0.72 is an upper bound, twice over.** A 256-byte page is coarse: the block
at $001500 is small, so most of those 104 writes/frame are probably data
living in the same page, not code. And a page only becomes "code" once
something executes there, so the write that INSTALLS a routine is never
counted -- correct for measuring steady state, but it means a page written
once and then only executed shows zero. Both biases make the real cost lower,
and it is already negligible, so the granularity question is closed.

**Do not remove dyn_pc_in_wram().** It stops meaning "refuse to translate" and
starts meaning "this block needs invalidation tracking". The mirror handling
it already does ($0000-$1FFF of banks $00-$3F and $80-$BF, not just $7E/$7F)
is exactly what the page indexing must also get right: a block cached from
$00:1500 and a write to $7E:1500 are the same bytes.

**Where the write check goes.** S9xSetByte/S9xSetWord in getset.h, and
REGISTER_2180 in ppu.h. DMA reaches WRAM through the $2180 port and calls that
macro directly (dma.cpp) without ever touching the byte path -- the same split
that hid a watchpoint bug in arm-snesrec, where a watch on the byte path
reported a confident zero for a region DMA was actively filling. Miss either
hook and the cache goes stale silently, in the direction that looks like good
news.

**Verification is behavioural, not VERIFY.** VERIFY diffs single opcodes and
cannot see a stale block: the failure is not a wrong opcode, it is a block
that should have been discarded and was not.

## 8d. It was built, it works, and it is off by default

The invalidation is implemented and correct. FF6 ran 90 seconds with its
$001500 trampoline being rewritten underneath a cached block, ~21
invalidations a second, no corruption, X restored cleanly. The predicted cost
held: 1,896 invalidations in 1,896 events, i.e. every event discarded exactly
one block, exactly as "the lists are one entry long" predicted.

It is still **off by default**, because it makes the emulator slower.

### The measurement that says so, and the one that nearly hid it

`blocks run` is not a speed metric, and it was being read as one. Enabling
WRAM blocks TRIPLED it -- which looks like a triumph and is not:

| configuration | blocks run | native | **frames/s** |
|---|---|---|---|
| interpreter (no dynarec compiled) | -- | -- | **51.66** |
| EXEC binary, dynarec disarmed | -- | -- | **50.57** |
| EXEC, ROM blocks only | 5,785,909 | 43% | **41.15** |
| EXEC, ROM + WRAM blocks | 17,522,850 | 20% | **31.63** |

Same binary and same boot for the last two, one environment variable apart.
Three times the blocks for three quarters of the speed. WRAM code hits almost
nothing the translator does natively (43% -> 20%), so each of those blocks is
a prologue and an epilogue wrapped around a chain of interpreter calls --
strictly more work than letting the interpreter dispatch it directly.

That is why menu/main.cpp now counts emulated frames in EVERY build, outside
any dynarec gate: it is the only figure comparable between the interpreter and
a dynarec configuration.

### The larger finding

Read the first and third rows together. **The dynarec is currently ~20% slower
than the interpreter it exists to accelerate**, before WRAM blocks enter into
it. 43% of instructions going native is not enough to pay for the block
prologue/epilogue plus a call per fallback instruction.

So the bottleneck is not block plumbing, and adding more of it makes things
worse. What would change the picture is native coverage of the opcodes that
actually run, and block linking to stop paying prologue/epilogue per block.
WRAM blocks become worth switching on at the point where those land -- the
machinery is ready and inert until then (PIKO_DYN_WRAM_BLOCKS=1).

### Noise, and what not to conclude

Three runs of the *same* armed configuration gave 41.81 / 40.87 / 41.15 --
about 2% spread. The interpreter-vs-dynarec gap (51.66 vs ~41) and the WRAM
gap (~41 vs 31.63) are far outside that and are real. The ~1 frame/s between
"no hook compiled" and "hook compiled but inert" is NOT: it is inside the
spread, and no claim rests on it.

### One limitation, stated rather than papered over

Invalidation is at block granularity, checked when the write happens. A block
that writes into its own page and then keeps executing modified bytes *within
the same block* will run the stale snapshot to the end of that block; the
interpreter, which re-fetches every opcode, would not. Bailing out mid-block
is expensive and no game seen here needs it -- FF6's trampoline is rewritten
by other code, not by the block living there -- but it is a real difference
from interpreter semantics rather than an oversight.

## 9. Gates that cannot be combined, and other traps

- **EXEC and PROFILE are mutually blind.** When a translated block runs,
  cpuexec.cpp does `goto piko_after_dispatch`, which jumps past the profiler
  hook. Arming both gives a profile of only what the dynarec did NOT run, and
  no block counters worth reading. Run them separately.
- **Always `make clean` when changing gate flags.** Makefile.zaurus does not
  track the dynarec `-D` flags, so switching between EXEC=1 / VERIFY=1 /
  PROFILE=1 silently links objects compiled under the previous set. This
  produced a run that printed "DYN-EXEC: armed" and then "FINAL 0 blocks run";
  the identical flags built clean gave 5,789,931 blocks.
- **Both gates report only on an interval** (4M blocks, 250k checks), so a
  short run can end having printed nothing -- which looks exactly like the
  feature never running. dyn_exec_report()/dyn_verify_report() print at exit;
  use them rather than inferring from silence.
- **The device cannot stop a process.** No `kill` applet, no `timeout` applet,
  no ash kill builtin. Use PIKO_MAX_SECONDS; see dynarec/run-on-zaurus.sh.

## 8. Files

| File | Role |
|------|------|
| `dynarec_arm.h` | ARMv5TE instruction emitter (RWX-buffer writes). The assembler. |
| `dynarec.c` | RWX code buffer + I-cache flush + `S9xDynSelfTest()` (JIT + Step 1 offline tests). |
| `dynarec.h`  | Public interface. |
| `dynarec_block.{h,c}` | 65816 decoder, block discovery, block cache. |
| `dynarec_harness.{h,c}` | CPU-state snapshot/diff + WRAM hash (correctness net). |
| `dynarec_translate.cpp` | 65816 → ARM block translator (`.cpp` for `offsetof` on snes9x structs). |
| `dynarec_verify.cpp` | Live run-both-and-diff validator (`make VERIFY=1`). |
| `dynarec_exec.{h,cpp}` | Block-driven execution: blocks drive the CPU (`make EXEC=1`). |
| `README.md`  | This document. |

## 10. Branches, and the ~20% that is still unexplained

BPL/BNE/BEQ are translated natively and, more importantly, **no longer end a
block**: the not-taken path continues straight into the following
instructions. They were 36% of everything executed and every one of them cut a
block short, which is why blocks averaged 2.66 instructions.

The taken path deliberately calls the interpreter's own Op10/OpD0/OpF0 and
exits the block. Op10 is not "add a signed byte to PC": it runs a BranchCheck
macro whose behaviour depends on cpu->BranchSkip and Settings.SoundSkipMethod
(BranchCheck1 and BranchCheck2 differ), then CPUShutdown, the idle-loop
detector. Reimplementing that in generated code to save a call on the path
that leaves the block anyway would trade the correctness of the common
not-taken path for almost nothing.

It works. Blocks got 2.2x longer (2.66 -> 6.69 static instructions), VERIFY
passes 15,787 checks with 0 divergences, and FF6 runs.

**Throughput did not move.** 40.54 frames/s, against 40.47 before branches and
41.15 before that. Three substantial optimisations -- per-block register
pinning, declining all-fallback blocks, native branches with fall-through --
all correct, all verified, none measurable.

### What the min-native sweep showed, and what it did not

`PIKO_DYN_MIN_NATIVE=N` refuses blocks less than N% native. Swept on one boot:

| min-native | blocks run | frames/s |
|---|---|---|
| 1%   | 3,900,000 | 43.61 |
| 60%  | 3,237,165 | 42.06 |
| 90%  | **0**     | 40.80 |
| 100% | **0**     | 40.78 |

At 90% and above NO blocks run at all -- and it still measures 40.8, against
50.57 for the same binary with the dynarec simply disarmed. So the ~20% cost
of EXEC is **not the blocks**. It is paid before any block executes.

The obvious suspect was dyn_exec_step's cache lookup on every dispatch. That
was tested and it is wrong: gating the lookup on "the previous instruction
ended a block" cut lookups from ~23M to ~5.7M with the identical set of blocks
still running, and measured SLOWER (39.40 vs 40.91). The gate is reverted; the
arithmetic also disagrees with the hypothesis, since ~23M lookups of ~20
instructions is on the order of 1% of a 90-second run, not 20%.

The leading remaining candidate is D-cache pressure rather than instruction
count. exec_key/exec_ptr/exec_valid are 64 KB each, 192 KB of tables probed by
a hash, on a PXA270 with a 32 KB L1 data cache -- so every lookup is likely to
miss AND to evict the interpreter's own hot data (opcode table, CPU state).
That would explain why cutting the NUMBER of lookups by 4x did not help: the
working set is what matters, not the count. It also predicts the fix is to
shrink the cache (a small direct-mapped table, 16-bit slot indices, one array
instead of three) rather than to consult it less often.

That is a prediction, not a result. The way to settle it is to shrink the
tables and re-measure, and it is where the next session should start --
**before** any more opcodes are translated, because until this is understood
every opcode added is being measured through a 20% fog.

### A measurement that stopped being trustworthy

"insns native + fallback" is now a STATIC count: each dispatch charges the
block's whole translated length, and since a taken branch exits early, the
total can exceed the instructions the emulated CPU actually issued -- and did
(38.1M, which is more than the machine executes). It is relabelled "block
contents (static)" in the report. dyn_interp_dispatches counts real
single-instruction dispatches and is exact.

## 11. Bisecting the ~20%: four hypotheses down, one number that matters

Same binary, same boot, one environment variable apart. `PIKO_DYN_STUB=1` makes
dyn_exec_step return on its first line, so the call still happens and its body
does not:

| configuration | frames/s | dispatches/frame |
|---|---|---|
| disarmed (no call at all)        | **50.81** | 9,118 |
| armed, call returns immediately  | **48.46** | 9,153 |
| armed, full lookup, zero blocks  | **40.38** | 9,322 |
| armed, blocks running            | **43.70** | 3,410 |

Read down the first column: the CALL costs 4.6%, the BODY costs another 17%,
and blocks then hand back 8%. So block-driven execution is a real win over
armed-but-idle -- it is dragging a fixed 21% overhead behind it.

Read across the second column: 9,118 dispatches/frame disarmed against 9,322
armed. The dynarec is not making the machine do more work; it is making the
same work slower, per instruction. Those two want opposite fixes, which is why
this counter is no longer gated on dyn_exec_on.

**What the 17% is not.** Every one of these was tested on hardware, not
reasoned about:

  - *Not the blocks.* PIKO_DYN_MIN_NATIVE=100 runs ZERO blocks and still
    measures 40.38.
  - *Not the number of lookups.* Gating them on control-flow boundaries cut
    ~23M to ~5.7M with the identical blocks running, and measured SLOWER
    (39.40 vs 40.91). Reverted.
  - *Not the table footprint.* PIKO_DYN_SLOTS sweeping 16384/8192/4096/2048 --
    192 KB down to 24 KB of touched working set, against a 32 KB L1 -- gave
    43.75 / 43.50 / 44.31 / 44.15. Flat. The D-cache pressure hypothesis, which
    section 10 called the leading candidate, is **dead**.
  - *Not the dyn_pc_in_wram call.* Replacing it with the inline
    dyn_wram_offset_of gave 43.31 against 43.70. Flat.

The third and fourth results explain each other: with WRAM blocks off, most
dispatches are rejected by the WRAM test BEFORE the hash table is reached, so
the table was never hot enough for its size to matter.

**What is left in that body**, in order of remaining suspicion: the global
counters incremented on nearly every dispatch (e_wram_skip, and
dyn_interp_dispatches in cpuexec.cpp) -- each a load/add/store to a separate
cache line; the pc reconstruction (reg->PB, cpu->PC, cpu->PCBase, reg->P.W are
four loads from three structs); and I-cache displacement of the interpreter's
dispatch loop. The next experiment is the cheapest of those: compile the
counters out and re-measure. If they are it, that is a satisfying answer, and
if they are not, the remaining candidates are structural and want the profiler
pointed at the dispatch loop rather than another guess.

**Do not skip the ablation step.** Four plausible mechanisms in a row were
wrong, and each cost a build-deploy-measure cycle. The arithmetic was available
in advance for at least two of them: ~23M lookups of ~20 instructions is on the
order of 1% of a 90-second run, never 20%.

## 12. Hypothesis 1: the counters. Partly right, and worth 4.5%

Bisection (section 11) put ~17% inside dyn_exec_step's body. First suspect on
the list was the global counters incremented on nearly every dispatch. Tested
with `make NOCOUNT=1`, which compiles them all out, against the same configs:

| configuration | counted | NOCOUNT | delta |
|---|---|---|---|
| disarmed              | 51.11 | 51.98 | +1.7% |
| armed, zero blocks    | 40.16 | 40.66 | +1.2% |
| armed, blocks running | 43.23 | **45.16** | **+4.5%** |

Two different answers in one table, and the difference between them is the
finding:

  - On the **dispatch** path the counters cost ~1.2%. So they are NOT the 17%.
    Hypothesis 1 is mostly wrong, like the four before it.
  - On the **block** path they cost 4.5%, which is real and was being paid on
    every single block run for a diagnostic.

The expensive one is the native/fallback split: `e_insn_nat += exec_nat[slot]`
is two loads from 32 KB arrays plus two global read-modify-writes, per block.
It is now behind `make BLOCKSTATS=1` and off by default; e_blocks stays,
because the FINAL line is how a run is known to have done anything. A build
without it says so in the report rather than printing a silent zero.

Default EXEC build after the change: **44.63** against 43.23, with an identical
5,698,515 blocks run. Overhead against disarmed (51.56) is down from ~20% to
~13.4%.

Still outstanding, and still in this order:
  2. the pc reconstruction -- reg->PB, cpu->PC, cpu->PCBase, reg->P.W is four
     loads across three structs, per dispatch;
  3. I-cache displacement of the interpreter's dispatch loop.

Six hypotheses tested so far, one and a half right. The ablation knobs
(PIKO_DYN_STUB / _SLOTS / _MIN_NATIVE, make NOCOUNT / BLOCKSTATS) are what
makes being wrong cheap, and they are worth more than any single answer.
