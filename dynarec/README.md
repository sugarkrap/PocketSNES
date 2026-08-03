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
  - [ ] Run-both-and-diff gate, then translate the rest of the hot set
    (`LDA/STA/STZ`, branches, `CMP/CPY`, `INX/INY/DEX/DEC`, `ASL`, `XBA`,
    `JSR/RTS`, `PHA/PLA`), one opcode at a time.
  - [ ] Idle-loop detection for the boot spin (`$FA:00F9` CLC/LDA/BPL).
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

## 8. Files

| File | Role |
|------|------|
| `dynarec_arm.h` | ARMv5TE instruction emitter (RWX-buffer writes). The assembler. |
| `dynarec.c` | RWX code buffer + I-cache flush + `S9xDynSelfTest()` (JIT + Step 1 offline tests). |
| `dynarec.h`  | Public interface. |
| `dynarec_block.{h,c}` | 65816 decoder, block discovery, block cache. |
| `dynarec_harness.{h,c}` | CPU-state snapshot/diff + WRAM hash (correctness net). |
| `dynarec_translate.cpp` | 65816 → ARM block translator (`.cpp` for `offsetof` on snes9x structs). |
| `README.md`  | This document. |
