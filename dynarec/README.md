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

- **Register ABI** — the hot 65816 registers (`A`, `X`, `Y`, and the `P`
  flags) live in fixed ARM registers for the duration of a block; the block
  **prologue** loads them from `SRegisters`, the **epilogue** spills them back.
  `SRegisters` = `PC, PB, DB, P, A, D, S, X, Y` (`pair` = 16-bit with byte
  access). Everything else stays memory-backed and is touched via `LDR/STR`.

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
  - [ ] **Live wiring** (remaining): capture `DynCpuSnap` from the running CPU;
    hook `S9xMainLoop` to execute cached blocks via the interpreter fn-pointers
    with identical per-instruction event checks; run-both-and-diff mode; per-
    block hotness dump. This is the behaviourally-risky part and lands behind
    the harness above, default-off.
- [ ] **Step 2 — Emitter register ABI + prologue/epilogue.** Map `A/X/Y/P` to
  ARM regs; block entry/exit spill/reload against `SRegisters`.
- [ ] **Step 3 — Translate the trivial opcodes**, each gated by the harness:
  register transfers (`TAX/TAY/TXA…`), immediate loads (`LDA/LDX/LDY #`), flag
  ops (`SEC/CLC/SEI`; `REP/SEP` as block-enders). Everything else still calls
  the interpreter.
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

## 7. Files

| File | Role |
|------|------|
| `dynarec_arm.h` | ARMv5TE instruction emitter (RWX-buffer writes). The assembler. |
| `dynarec.c` | RWX code buffer + I-cache flush + `S9xDynSelfTest()` (JIT + Step 1 offline tests). |
| `dynarec.h`  | Public interface. |
| `dynarec_block.{h,c}` | 65816 decoder, block discovery, block cache. |
| `dynarec_harness.{h,c}` | CPU-state snapshot/diff + WRAM hash (correctness net). |
| `README.md`  | This document. |
