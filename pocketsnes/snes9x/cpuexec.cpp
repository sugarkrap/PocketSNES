/*
 * Snes9x - Portable Super Nintendo Entertainment System (TM) emulator.
 *
 * (c) Copyright 1996 - 2001 Gary Henderson (gary.henderson@ntlworld.com) and
 *                           Jerremy Koot (jkoot@snes9x.com)
 *
 * Super FX C emulator code 
 * (c) Copyright 1997 - 1999 Ivar (ivar@snes9x.com) and
 *                           Gary Henderson.
 * Super FX assembler emulator code (c) Copyright 1998 zsKnight and _Demo_.
 *
 * DSP1 emulator code (c) Copyright 1998 Ivar, _Demo_ and Gary Henderson.
 * C4 asm and some C emulation code (c) Copyright 2000 zsKnight and _Demo_.
 * C4 C code (c) Copyright 2001 Gary Henderson (gary.henderson@ntlworld.com).
 *
 * DOS port code contains the works of other authors. See headers in
 * individual files.
 *
 * Snes9x homepage: http://www.snes9x.com
 *
 * Permission to use, copy, modify and distribute Snes9x in both binary and
 * source form, for non-commercial purposes, is hereby granted without fee,
 * providing that this license information and copyright notice appear with
 * all copies and any derived work.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event shall the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Snes9x is freeware for PERSONAL USE only. Commercial users should
 * seek permission of the copyright holders first. Commercial use includes
 * charging money for Snes9x or software derived from Snes9x.
 *
 * The copyright holders request that bug fixes and improvements to the code
 * should be forwarded to them so everyone can benefit from the modifications
 * in future versions.
 *
 * Super NES and Super Nintendo Entertainment System are trademarks of
 * Nintendo Co., Limited and its subsidiary companies.
 */
#include "snes9x.h"
#include "memmap.h"
#include "cpuops.h"
#include "ppu.h"
#include "cpuexec.h"
#include "debug.h"
#include "snapshot.h"
#include "gfx.h"
#include "missing.h"
#include "apu.h"
#include "dma.h"
#include "fxemu.h"
#include "sa1.h"
#ifdef PIKO_DYNAREC_PROFILE
#include "dynarec_block.h"
#endif
#ifdef PIKO_DYNAREC_VERIFY
#include "dynarec_verify.h"
#endif
#ifdef PIKO_DYNAREC_EXEC
#include "dynarec_exec.h"
#include "dynarec_block.h"
#endif
#ifdef THREADCPU
void S9xMainLoop (struct SRegisters * reg, struct SICPU * icpu, struct SCPUState * cpu)
{
#else
#ifdef PIKO_DYNAREC_EXEC
/*
 * Cheap "could a block even start here?" filter, inlined at the dispatch site.
 *
 * dyn_exec_step pushes NINE callee-saved registers and a 28-byte frame before
 * it looks at anything (checked in the disassembly), and with WRAM blocks off
 * it then throws away ~72% of dispatches -- having paid a call, a prologue,
 * and a second out-of-line call to dyn_wram_offset_of, which -Os declined to
 * inline despite the keyword and which returns its result through memory.
 *
 * always_inline, not a hint: `static __inline` on exactly this test is what
 * failed to inline last time, and the resulting no-op change measured flat and
 * was nearly recorded as "hypothesis disproved".
 *
 * This must agree with dyn_wram_offset_of. It is only a FILTER -- dyn_exec_step
 * re-checks -- so a false "yes" costs a wasted call and nothing else; a false
 * "no" would silently stop translating a region, which is why it mirrors the
 * mirror handling exactly rather than approximating it.
 */
static __inline__ __attribute__((always_inline))
int piko_block_possible (void)
{
	uint32 bank, off;

	if (dyn_exec_wram_blocks)
		return 1;                       /* WRAM blocks on: everything is fair game */
	bank = (uint32) Registers.PB & 0xFF;
	if (bank == 0x7E || bank == 0x7F)
		return 0;
	off = (uint32) (uint16) (CPU.PC - CPU.PCBase);
	if (off < 0x2000 && (bank < 0x40 || (bank >= 0x80 && bank < 0xC0)))
		return 0;
	return 1;
}
#endif

void S9xMainLoop (void)
{
	struct SICPU		* icpu  = &ICPU;
	struct SCPUState	* cpu	= &CPU;
	struct SRegisters	* reg	= &Registers;
#endif
    for (;;)
    {
		APU_EXECUTE ();

		if (CPU.Flags)
		{
			if (CPU.Flags & NMI_FLAG)
			{
				if (--CPU.NMICycleCount == 0)
				{
					CPU.Flags &= ~NMI_FLAG;
					if (CPU.WaitingForInterrupt)
					{
						CPU.WaitingForInterrupt = FALSE;
						++CPU.PC;
					}
					S9xOpcode_NMI ();
				}
			}

#ifdef DEBUGGER
			if ((CPU.Flags & BREAK_FLAG) &&
			!(CPU.Flags & SINGLE_STEP_FLAG))
			{
				for (int Break = 0; Break != 6; Break++)
				{
					if (S9xBreakpoint[Break].Enabled &&
					S9xBreakpoint[Break].Bank == Registers.PB &&
					S9xBreakpoint[Break].Address == CPU.PC - CPU.PCBase)
					{
						if (S9xBreakpoint[Break].Enabled == 2)
							S9xBreakpoint[Break].Enabled = TRUE;
						else
							CPU.Flags |= DEBUG_MODE_FLAG;
					}
				}
			}
#endif
			CHECK_SOUND ();

			if (CPU.Flags & IRQ_PENDING_FLAG)
			{
				if (CPU.IRQCycleCount == 0)
				{
					if (CPU.WaitingForInterrupt)
					{
						CPU.WaitingForInterrupt = FALSE;
						CPU.PC++;
					}
					if (CPU.IRQActive && !Settings.DisableIRQ)
					{
						if (!CHECKFLAG (IRQ))
							S9xOpcode_IRQ ();
					}
					else
						CPU.Flags &= ~IRQ_PENDING_FLAG;
				}
				else
					CPU.IRQCycleCount--;
			}
#ifdef DEBUGGER
			if (CPU.Flags & DEBUG_MODE_FLAG)
				break;
#endif
			if (CPU.Flags & SCAN_KEYS_FLAG)
				break;
#ifdef DEBUGGER
			if (CPU.Flags & TRACE_FLAG)
				S9xTrace ();

			if (CPU.Flags & SINGLE_STEP_FLAG)
			{
				CPU.Flags &= ~SINGLE_STEP_FLAG;
				CPU.Flags |= DEBUG_MODE_FLAG;
			}
#endif
		} //if (CPU.Flags)
#ifdef PIKO_DYNAREC_EXEC
		/* block-driven execution: if a translated block runs, it advances the
		 * CPU through a whole straight-line run and updates cpu->PC; skip the
		 * single-instruction dispatch. Event servicing (SA1/HBLANK below, and
		 * the CPU.Flags/APU work above) then happens at block granularity. */
		/*
		 * Tried and reverted: gating this on "the previous instruction ended a
		 * block" (dyn_try_block). It cut lookups from ~23M to ~5.7M with the
		 * identical set of blocks still running, and measured SLOWER --
		 * 40.91 -> 39.40 frames/s. Whatever the ~20% cost of arming EXEC is,
		 * the per-dispatch cache lookup is not it. See README section 10.
		 */
		if (dyn_exec_on && piko_block_possible ()
		    && dyn_exec_step (&Registers, &ICPU, &CPU))
			goto piko_after_dispatch;
#endif
#ifdef CPU_SHUTDOWN
		CPU.PCAtOpcodeStart = CPU.PC;
#endif
#ifdef PIKO_DYNAREC_PROFILE
		/* dynarec hotness probe: observation only (counters), reads just the
		 * current opcode byte -- no look-ahead, no OOB. Runtime-gated. */
		if (dyn_profile_on)
			dyn_profile_op(*CPU.PC,
			               ((uint32)Registers.PB << 16)
			                   | (uint16)(CPU.PC - CPU.PCBase),
			               (Registers.P.W & MemoryFlag) != 0,
			               (Registers.P.W & IndexFlag) != 0);
#endif
#ifdef VAR_CYCLES
		CPU.Cycles += CPU.MemSpeed;
#else
		CPU.Cycles += ICPU.Speed [*CPU.PC];
#endif
#ifdef PIKO_DYNAREC_VERIFY
		/* run-both-and-diff: snapshot AFTER the cycle add (so the native and
		 * interpreter cycle baselines match) but BEFORE the opcode runs. */
		unsigned char _vop = 0; int _vdo = 0;
		if (dyn_verify_on) {
			_vop = *CPU.PC;
			if (dyn_verify_translatable(_vop)) {
				dyn_verify_before(&Registers, &ICPU, &CPU);
				_vdo = 1;
			}
		}
#endif
#ifdef PIKO_DYNAREC_EXEC
		/* Unconditional: gating this on dyn_exec_on made armed and disarmed
		 * runs incomparable, so "is the dynarec slower per instruction, or is
		 * it causing MORE instructions to run?" could not be answered. */
		DYN_COUNT (dyn_interp_dispatches++);
#endif
		(*ICPU.S9xOpcodes [*CPU.PC++].S9xOpcode) (&Registers, &ICPU, &CPU);
#ifdef PIKO_DYNAREC_VERIFY
		if (_vdo)
			dyn_verify_after(_vop, &Registers, &ICPU, &CPU);
#endif
#ifdef PIKO_DYNAREC_EXEC
	piko_after_dispatch:
#endif

		if (SA1.Executing)
			S9xSA1MainLoop ();
		DO_HBLANK_CHECK();
	} // for(;;)
    Registers.PC = CPU.PC - CPU.PCBase;
    S9xPackStatus ();
    APURegisters.PC = IAPU.PC - IAPU.RAM;
    S9xAPUPackStatus ();
    if (CPU.Flags & SCAN_KEYS_FLAG)
    {
#ifdef DEBUGGER
		if (!(CPU.Flags & FRAME_ADVANCE_FLAG))
#endif
			S9xSyncSpeed ();
		CPU.Flags &= ~SCAN_KEYS_FLAG;
    }
    if (CPU.BRKTriggered && Settings.SuperFX && !CPU.TriedInterleavedMode2)
    {
		CPU.TriedInterleavedMode2 = TRUE;
		CPU.BRKTriggered = FALSE;
		S9xDeinterleaveMode2 ();
    }
}


void S9xSetIRQ (uint32 source)
{
    CPU.IRQActive |= source;
    CPU.Flags |= IRQ_PENDING_FLAG;
    CPU.IRQCycleCount = 3;
    if (CPU.WaitingForInterrupt)
    {
		// Force IRQ to trigger immediately after WAI - 
		// Final Fantasy Mystic Quest crashes without this.
		CPU.IRQCycleCount = 0;
		CPU.WaitingForInterrupt = FALSE;
		CPU.PC++;
    }
}

void S9xClearIRQ (uint32 source)
{
    CLEAR_IRQ_SOURCE (source);
}

void S9xDoHBlankProcessing ()
{
#ifdef CPU_SHUTDOWN
    CPU.WaitCounter++;
#endif
    switch (CPU.WhichEvent)
    {
    case HBLANK_START_EVENT:
		if (IPPU.HDMA && CPU.V_Counter <= PPU.ScreenHeight)
			IPPU.HDMA = S9xDoHDMA (IPPU.HDMA);
	break;

    case HBLANK_END_EVENT:
	S9xSuperFXExec ();

#ifndef STORM
	if (Settings.SoundSync)
	    S9xGenerateSound ();
#endif

	CPU.Cycles -= Settings.H_Max;
	if (IAPU.APUExecuting)
	    APU.Cycles -= Settings.H_Max;
	else
	    APU.Cycles = 0;

	CPU.NextEvent = -1;
	ICPU.Scanline++;

	if (++CPU.V_Counter > (Settings.PAL ? SNES_MAX_PAL_VCOUNTER : SNES_MAX_NTSC_VCOUNTER))
	{
	    PPU.OAMAddr = PPU.SavedOAMAddr;
	    PPU.OAMFlip = 0;
	    CPU.V_Counter = 0;
	    CPU.NMIActive = FALSE;
	    ICPU.Frame++;
	    PPU.HVBeamCounterLatched = 0;
	    CPU.Flags |= SCAN_KEYS_FLAG;
	    S9xStartHDMA ();
	}

	if (PPU.VTimerEnabled && !PPU.HTimerEnabled &&
	    CPU.V_Counter == PPU.IRQVBeamPos)
	{
	    S9xSetIRQ (PPU_V_BEAM_IRQ_SOURCE);
	}

	if (CPU.V_Counter == PPU.ScreenHeight + FIRST_VISIBLE_LINE)
	{
	    // Start of V-blank
	    S9xEndScreenRefresh ();
	    PPU.FirstSprite = 0;
	    IPPU.HDMA = 0;
	    // Bits 7 and 6 of $4212 are computed when read in S9xGetPPU.
	    missing.dma_this_frame = 0;
	    IPPU.MaxBrightness = PPU.Brightness;
	    PPU.ForcedBlanking = (Memory.FillRAM [0x2100] >> 7) & 1;

	    Memory.FillRAM[0x4210] = 0x80;
	    if (Memory.FillRAM[0x4200] & 0x80)
	    {
			CPU.NMIActive = TRUE;
			CPU.Flags |= NMI_FLAG;
			CPU.NMICycleCount = CPU.NMITriggerPoint;
	    }

#ifdef OLD_SNAPSHOT_CODE
	    if (CPU.Flags & SAVE_SNAPSHOT_FLAG)
	    {
			CPU.Flags &= ~SAVE_SNAPSHOT_FLAG;
			Registers.PC = CPU.PC - CPU.PCBase;
			S9xPackStatus ();
			S9xAPUPackStatus ();
			Snapshot (NULL);
	    }
#endif
        }

	if (CPU.V_Counter == PPU.ScreenHeight + 3)
	    S9xUpdateJoypads ();

	if (CPU.V_Counter == FIRST_VISIBLE_LINE)
	{
	    Memory.FillRAM[0x4210] = 0;
	    CPU.Flags &= ~NMI_FLAG;
	    S9xStartScreenRefresh ();
	}
	if (CPU.V_Counter >= FIRST_VISIBLE_LINE &&
	    CPU.V_Counter < PPU.ScreenHeight + FIRST_VISIBLE_LINE)
	{
	    RenderLine (CPU.V_Counter - FIRST_VISIBLE_LINE);
	}
	// Use TimerErrorCounter to skip update of SPC700 timers once
	// every 128 updates. Needed because this section of code is called
	// once every emulated 63.5 microseconds, which coresponds to
	// 15.750KHz, but the SPC700 timers need to be updated at multiples
	// of 8KHz, hence the error correction.
//	IAPU.TimerErrorCounter++;
//	if (IAPU.TimerErrorCounter >= )
//	    IAPU.TimerErrorCounter = 0;
//	else
	{
	    if (APU.TimerEnabled [2])
	    {
			APU.Timer [2] += 4;
			while (APU.Timer [2] >= APU.TimerTarget [2])
			{
				IAPU.RAM [0xff] = (IAPU.RAM [0xff] + 1) & 0xf;
				APU.Timer [2] -= APU.TimerTarget [2];
#ifdef SPC700_SHUTDOWN		
				IAPU.WaitCounter++;
				IAPU.APUExecuting = TRUE;
#endif		
			}
	    }
	    if (CPU.V_Counter & 1)
	    {
			if (APU.TimerEnabled [0])
			{
				APU.Timer [0]++;
				if (APU.Timer [0] >= APU.TimerTarget [0])
				{
					IAPU.RAM [0xfd] = (IAPU.RAM [0xfd] + 1) & 0xf;
					APU.Timer [0] = 0;
#ifdef SPC700_SHUTDOWN		
					IAPU.WaitCounter++;
					IAPU.APUExecuting = TRUE;
#endif		    
			    }
			}
			if (APU.TimerEnabled [1])
			{
				APU.Timer [1]++;
				if (APU.Timer [1] >= APU.TimerTarget [1])
				{
					IAPU.RAM [0xfe] = (IAPU.RAM [0xfe] + 1) & 0xf;
					APU.Timer [1] = 0;
#ifdef SPC700_SHUTDOWN		
					IAPU.WaitCounter++;
					IAPU.APUExecuting = TRUE;
#endif		    
			    }
			}
	    }
	}
	break; //HBLANK_END_EVENT

    case HTIMER_BEFORE_EVENT:
    case HTIMER_AFTER_EVENT:
	if (PPU.HTimerEnabled &&
	    (!PPU.VTimerEnabled || CPU.V_Counter == PPU.IRQVBeamPos))
	{
	    S9xSetIRQ (PPU_H_BEAM_IRQ_SOURCE);
	}
	break;
    }
    S9xReschedule ();
}
