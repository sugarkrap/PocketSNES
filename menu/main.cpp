
#include <sal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "unzip.h"
#include "zip.h"
#include "menu.h"
#include "snes9x.h"
#include "memmap.h"
#include "apu.h"
#include "gfx.h"
#include "soundux.h"
#include "snapshot.h"
#include "savestateio.h"
#include "dynarec.h"
#ifdef PIKO_DYNAREC_PROFILE
#include "dynarec_block.h"
#endif
#ifdef PIKO_DYNAREC_VERIFY
#include "dynarec_verify.h"
#endif
#ifdef PIKO_DYNAREC_EXEC
#include "dynarec_exec.h"
#include "dynarec_wram.h"
#endif

#define SNES_SCREEN_WIDTH  256
#define SNES_SCREEN_HEIGHT 192

/* PIKO_MAX_SECONDS deadline; 0 = run until told otherwise. See mainEntry. */
static time_t piko_deadline = 0;

/*
 * Emulated frames and wall time, in EVERY build rather than under a dynarec
 * gate, because this is the only throughput number that can be compared
 * between the interpreter and any dynarec configuration.
 *
 * "Blocks run" is not a speed metric and was being read as one: turning on
 * WRAM block translation tripled the block count (5.8M -> 17.5M) while
 * throughput FELL by 24% (41.81 -> 31.63 frames/s). WRAM blocks are shorter,
 * so more of them is what a slowdown looks like too.
 */
static unsigned long piko_frames = 0;
static time_t        piko_start  = 0;

static void piko_report_frames(void)
{
	long secs = (long)(time(NULL) - piko_start);
	if (secs <= 0) secs = 1;
	fprintf(stderr, "PIKO: %lu emulated frames in %lds = %lu.%02lu frames/s\n",
	        piko_frames, secs, piko_frames / (unsigned long)secs,
	        ((piko_frames % (unsigned long)secs) * 100) / (unsigned long)secs);
}

static struct MENU_OPTIONS mMenuOptions;
static int mEmuScreenHeight;
static int mEmuScreenWidth;
static char mRomName[SAL_MAX_PATH]={""};
static u32 mLastRate=0;

static s8 mFpsDisplay[16]={""};
static s8 mVolumeDisplay[16]={""};
static s8 mQuickStateDisplay[16]={""};
static u32 mFps=0;
static u32 mLastTimer=0;
static u32 mEnterMenu=0;
static u32 mLoadRequested=0;
static u32 mSaveRequested=0;
static u32 mQuickStateTimer=0;
static u32 mVolumeTimer=0;
static u32 mVolumeDisplayTimer=0;
static u32 mInMenu=0;

static int S9xCompareSDD1IndexEntries (const void *p1, const void *p2)
{
    return (*(uint32 *) p1 - *(uint32 *) p2);
}

extern "C"
{

void S9xExit ()
{
}

void S9xGenerateSound (void)
{
	S9xMessage (0,0,"generate sound");
	return;
}

void S9xSetPalette ()
{

}

void S9xExtraUsage ()
{
}
	
void S9xParseArg (char **argv, int &index, int argc)
{	
}

bool8 S9xOpenSnapshotFile (const char *fname, bool8 read_only, STREAM *file)
{
	if (read_only)
	{
		if (*file = OPEN_STREAM(fname,"rb")) 
			return(TRUE);
	}
	else
	{
		if (*file = OPEN_STREAM(fname,"w+b")) 
			return(TRUE);
	}

	return (FALSE);	
}
	
void S9xCloseSnapshotFile (STREAM file)
{
	CLOSE_STREAM(file);
}

void S9xMessage (int /* type */, int /* number */, const char *message)
{
	//MenuMessageBox("PocketSnes has encountered an error",(s8*)message,"",MENU_MESSAGE_BOX_MODE_PAUSE);
}

void erk (void)
{
      S9xMessage (0,0, "Erk!");
}

const char *osd_GetPackDir(void)
{
      S9xMessage (0,0,"get pack dir");
      return ".";
}

void S9xLoadSDD1Data (void)
{

}

   

bool8_32 S9xInitUpdate ()
{
	if(mInMenu) return (TRUE);
	if(mMenuOptions.fullScreen)	GFX.Screen = (uint8 *) sal_VideoGetBuffer();
	else				GFX.Screen = (uint8 *) sal_VideoGetBuffer()+(320-SNES_WIDTH)+((240-SNES_HEIGHT)*320);

	return (TRUE);
}

bool8_32 S9xDeinitUpdate (int Width, int Height, bool8_32)
{
	if(mInMenu) return TRUE;

	u32 newTimer;
	if (mMenuOptions.showFps) 
	{
		mFps++;
		newTimer=sal_TimerRead();
		if(newTimer-mLastTimer>Memory.ROMFramesPerSecond)
		{
			mLastTimer=newTimer;
			sprintf(mFpsDisplay,"FPS: %d / %d", mFps, Memory.ROMFramesPerSecond);
			mFps=0;
		}
		
		sal_VideoDrawRect(0,0,13*8,8,SAL_RGB(0,0,0));
		sal_VideoPrint(0,0,mFpsDisplay,SAL_RGB(31,31,31));
	}

	if(mVolumeDisplayTimer>0)
	{
		sal_VideoDrawRect(100,0,8*8,8,SAL_RGB(0,0,0));
		sal_VideoPrint(100,0,mVolumeDisplay,SAL_RGB(31,31,31));
	}

	if(mQuickStateTimer>0)
	{
		sal_VideoDrawRect(200,0,8*8,8,SAL_RGB(0,0,0));
		sal_VideoPrint(200,0,mQuickStateDisplay,SAL_RGB(31,31,31));
	}

	sal_VideoFlip(0);
	return TRUE;   /* non-void: must return (missing return here is UB, same class as S9xOpenSoundDevice) */
}

const char *S9xGetFilename (const char *ex)
{
	static char dir [SAL_MAX_PATH];
	char fname [SAL_MAX_PATH];
	char ext [SAL_MAX_PATH];

	sal_DirectorySplitFilename(Memory.ROMFilename, dir, fname, ext);
	strcpy(dir, sal_DirectoryGetHome());
	sal_DirectoryCombine(dir,fname);
	strcat (dir, ex);

	return (dir);
}

static u8 *mTempState=NULL;
static
void LoadStateMem()
{
	SetSaveStateIoModeMemory(&mTempState);
	S9xUnfreezeGame("blah");
}

static 
void SaveStateMem()
{
	SetSaveStateIoModeMemory(&mTempState);

	S9xFreezeGame("blah");
}

void HandleQuickStateRequests()
{
	if(mVolumeTimer>0) mVolumeTimer--;
	if(mVolumeDisplayTimer>0) mVolumeDisplayTimer--;

	if(mQuickStateTimer>0)
	{
		mQuickStateTimer--;
		return;
	}

	if(mSaveRequested)
	{
		mSaveRequested=0;
		SaveStateMem();
		mQuickStateTimer = 60;
	}

	if(mLoadRequested)
	{
		mLoadRequested=0;
		LoadStateMem();
		mQuickStateTimer = 60;
	}

	
}
uint32 S9xReadJoypad (int which1)
{
	uint32 val=0x80000000;
	if (mInMenu) return val;
	if (which1 != 0) return val;

	u32 joy = sal_InputPoll();
	
	if ((joy & SAL_INPUT_SELECT)&&(joy & SAL_INPUT_START))	
	{
		mEnterMenu = 1;		
		return val;
	}

	
	if ((joy & SAL_INPUT_L)&&(joy & SAL_INPUT_R)&&(joy & SAL_INPUT_LEFT))
	{
		if (mQuickStateTimer==0)
		{
			mSaveRequested=1;
			strcpy(mQuickStateDisplay,"Saved!");
		}
		return val;
	}

	if ((joy & SAL_INPUT_L)&&(joy & SAL_INPUT_R)&&(joy & SAL_INPUT_RIGHT))
	{
		if (mQuickStateTimer==0)
		{
			mLoadRequested=1;
			strcpy(mQuickStateDisplay,"Loaded!");
		}
		return val;
	}

#if 0
	if ((joy & SAL_INPUT_L)&&(joy & SAL_INPUT_R)&&(joy & SAL_INPUT_UP))
	{
		if(mVolumeTimer==0)
		{
			mMenuOptions.volume++;
			if(mMenuOptions.volume>31) mMenuOptions.volume=31;
			sal_AudioSetVolume(mMenuOptions.volume,mMenuOptions.volume);
			mVolumeTimer=5;
			mVolumeDisplayTimer=60;
			sprintf(mVolumeDisplay,"Vol: %d",mMenuOptions.volume);
		}
		return val;
	}

	if ((joy & SAL_INPUT_L)&&(joy & SAL_INPUT_R)&&(joy & SAL_INPUT_DOWN))
	{
		if(mVolumeTimer==0)
		{
			mMenuOptions.volume--;
			if(mMenuOptions.volume>31) mMenuOptions.volume=0;
			sal_AudioSetVolume(mMenuOptions.volume,mMenuOptions.volume);
			mVolumeTimer=5;
			mVolumeDisplayTimer=60;
			sprintf(mVolumeDisplay,"Vol: %d",mMenuOptions.volume);
		}
		return val;
	}
#endif

	if (joy & SAL_INPUT_Y) val |= SNES_Y_MASK;
	if (joy & SAL_INPUT_A) val |= SNES_A_MASK;
	if (joy & SAL_INPUT_B) val |= SNES_B_MASK;
	if (joy & SAL_INPUT_X) val |= SNES_X_MASK;
		
	if (joy & SAL_INPUT_UP) 	val |= SNES_UP_MASK;
	if (joy & SAL_INPUT_DOWN) 	val |= SNES_DOWN_MASK;
	if (joy & SAL_INPUT_LEFT) 	val |= SNES_LEFT_MASK;
	if (joy & SAL_INPUT_RIGHT)	val |= SNES_RIGHT_MASK;
	if (joy & SAL_INPUT_START) 	val |= SNES_START_MASK;
	if (joy & SAL_INPUT_SELECT) 	val |= SNES_SELECT_MASK;
	if (joy & SAL_INPUT_L) 		val |= SNES_TL_MASK;
	if (joy & SAL_INPUT_R) 		val |= SNES_TR_MASK;

	return val;
}

bool8 S9xReadMousePosition (int /* which1 */, int &/* x */, int & /* y */,
		    uint32 & /* buttons */)
{
	S9xMessage (0,0,"read mouse");
	return (FALSE);
}

bool8 S9xReadSuperScopePosition (int & /* x */, int & /* y */,
				 uint32 & /* buttons */)
{
      S9xMessage (0,0,"read scope");
      return (FALSE);
}

const char *S9xGetFilenameInc (const char *e)
{
     S9xMessage (0,0,"get filename inc");
     return e;
}

void S9xSyncSpeed(void)
{
      //S9xMessage (0,0,"sync speed");
}

const char *S9xBasename (const char *f)
{
      const char *p;

      S9xMessage (0,0,"s9x base name");

      if ((p = strrchr (f, '/')) != NULL || (p = strrchr (f, '\\')) != NULL)
         return (p + 1);

      return (f);
}



void S9xSaveSRAM (int showWarning)
{
	if (CPU.SRAMModified)
	{
		if(Memory.SaveSRAM ((s8*)S9xGetFilename (".srm")))
		{
			sync();
		}
		else
		{
			MenuMessageBox("Saving SRAM...Failed","SRAM Not Saved!","",MENU_MESSAGE_BOX_MODE_PAUSE);
		}
	}
	else if(showWarning)
	{
		MenuMessageBox("Saving SRAM...Ignored!","No changes have been made to SRAM","So there is nothing to save!",MENU_MESSAGE_BOX_MODE_MSG);
	}
	sleep(1);
}



}

bool8_32 S9xOpenSoundDevice(int a, unsigned char b, int c)
{
	/*
	 * Stub: the actual audio device is opened by sal_AudioInit() (sal_sound.c),
	 * not here -- this snes9x hook only needs to report success so S9xInitSound()
	 * proceeds. It was left with an empty body, which for a non-void function is
	 * undefined behaviour: at -O2 on ARM execution runs off the end of the
	 * function into whatever follows, which manifested as a wild re-entry into
	 * mainEntry() (with the sample rate as a bogus argc) the moment S9xInitSound
	 * called it. Return a value.
	 */
	(void)a; (void)b; (void)c;
	return (1);
}

void S9xAutoSaveSRAM (void)
{
	//since I can't sync the data, there is no point in even writing the data
	//out at this point.  Instead I'm now saving the data as the users enter the menu.
	//Memory.SaveSRAM (S9xGetFilename (".srm"));
	//sync();  can't sync when emulator is running as it causes delays
}

void S9xLoadSRAM (void)
{
	Memory.LoadSRAM ((s8*)S9xGetFilename (".srm"));
}

static
int Run(int sound)
{
  	int skip=0, done=0, doneLast=0,aim=0,i;
	Settings.NextAPUEnabled = Settings.APUEnabled = sound;
	sal_TimerInit(Settings.FrameTime);
	done=sal_TimerRead()-1;

	if (sound) {
		/*
		Settings.SoundPlaybackRate = mMenuOptions.soundRate;
		Settings.Stereo = mMenuOptions.stereo ? TRUE : FALSE;
		*/
		Settings.SixteenBitSound=true;

		sal_AudioInit(mMenuOptions.soundRate, 16,
					mMenuOptions.stereo, Memory.ROMFramesPerSecond);

		S9xInitSound (mMenuOptions.soundRate,
					mMenuOptions.stereo, sal_AudioGetBufferSize());
		S9xSetPlaybackRate(mMenuOptions.soundRate);
		S9xSetSoundMute (FALSE);

	} else {
		S9xSetSoundMute (TRUE);
	}

  	while(!mEnterMenu) 
  	{
		for (i=0;i<10;i++)
		{
			aim=sal_TimerRead();
			if (done < aim)
			{
				done++;
				if (mMenuOptions.frameSkip == 0) //Auto
					IPPU.RenderThisFrame = (done >= aim);
				else if (IPPU.RenderThisFrame = (--skip <= 0))
					skip = mMenuOptions.frameSkip;

				//Run SNES for one glorious frame
				S9xMainLoop ();
				piko_frames++;
#ifdef PIKO_DYNAREC_WRAMSTAT
				/* Per-frame, because "code pages dirtied per frame" is the
				 * cost of invalidation, and it is only meaningful if the
				 * frame boundary is where the dirty set is cleared. */
				dyn_wram_frame();
#endif

				/* Checked per FRAME, not per 10-frame batch: a VERIFY build
				 * runs at about 1 FPS, so a batch granularity turned a 25s
				 * bound into minutes. This is the only thing that can stop the
				 * process on a device whose busybox has no kill. */
				if (piko_deadline && time(NULL) >= piko_deadline) {
					fprintf(stderr, "PIKO_MAX_SECONDS reached, exiting\n");
				piko_report_frames();
#ifdef PIKO_DYNAREC_EXEC
					if (dyn_exec_on) dyn_exec_report();
#endif
#ifdef PIKO_DYNAREC_VERIFY
					if (dyn_verify_on) dyn_verify_report();
#endif
#ifdef PIKO_DYNAREC_PROFILE
					if (dyn_profile_on) dyn_profile_dump(12, 12);
#endif
#ifdef PIKO_DYNAREC_WRAMSTAT
					if (dyn_wram_on) dyn_wram_report();
#endif
					fflush(stderr); fflush(stdout);
					exit(0);
				}

				if (sound) {
					S9xMixSamples((uint8 *) sal_GetCurrentAudioBuffer(),
								sal_AudioGetSampleCount());
					sal_SubmitSamples();
				}
//				HandleQuickStateRequests();
			}
			if (done>=aim) break; // Up to date now
			if (mEnterMenu) break;
		}
		done=aim; // Make sure up to date
		HandleQuickStateRequests();
		if (piko_deadline && time(NULL) >= piko_deadline) {
			/*
			 * exit(), not break. Breaking only leaves the emulation loop and
			 * drops back to the menu, so the process stays alive -- and with
			 * no kill on the device, "alive" means the screen stays dark until
			 * someone power-cycles the board. The whole point of this bound is
			 * that the process goes away by itself.
			 */
			fprintf(stderr, "PIKO_MAX_SECONDS reached, exiting\n");
				piko_report_frames();
#ifdef PIKO_DYNAREC_EXEC
			if (dyn_exec_on) dyn_exec_report();
#endif
#ifdef PIKO_DYNAREC_VERIFY
			if (dyn_verify_on) dyn_verify_report();
#endif
#ifdef PIKO_DYNAREC_PROFILE
			if (dyn_profile_on) dyn_profile_dump(12, 12);
#endif
#ifdef PIKO_DYNAREC_WRAMSTAT
			if (dyn_wram_on) dyn_wram_report();
#endif
			fflush(stderr);
			fflush(stdout);
			exit(0);
		}
  	}

	if (sound)
		sal_AudioClose();

	mEnterMenu=0;
	return mEnterMenu;

}

static inline int RunSound(void)
{
	return Run(1);
}

static inline int RunNoSound(void)
{
	return Run(0);
}

static 
int SnesRomLoad()
{
	char filename[SAL_MAX_PATH+1];
	int check;
	char text[256];
	FILE *stream=NULL;
  
    	MenuMessageBox("Loading Rom...",mRomName,"",MENU_MESSAGE_BOX_MODE_MSG);

	if (!Memory.LoadROM (mRomName))
	{
		MenuMessageBox("Loading Rom...",mRomName,"FAILED!!!!",MENU_MESSAGE_BOX_MODE_PAUSE);
		return SAL_ERROR;
	}
	
	MenuMessageBox("Loading Rom...OK!",mRomName,"",MENU_MESSAGE_BOX_MODE_MSG);

	S9xReset();
	S9xResetSound(1);
	S9xLoadSRAM();
	return SAL_OK;
}

int SnesInit()
{
	ZeroMemory (&Settings, sizeof (Settings));

	Settings.JoystickEnabled = FALSE;
	Settings.SoundPlaybackRate = 44100;
	Settings.Stereo = TRUE;
	Settings.SoundBufferSize = 0;
	Settings.CyclesPercentage = 100;
	Settings.DisableSoundEcho = FALSE;
	Settings.APUEnabled = TRUE;
	Settings.H_Max = SNES_CYCLES_PER_SCANLINE;
	Settings.SkipFrames = AUTO_FRAMERATE;
	Settings.Shutdown = Settings.ShutdownMaster = TRUE;
	Settings.FrameTimePAL = 20000;
	Settings.FrameTimeNTSC = 16667;
	Settings.FrameTime = Settings.FrameTimeNTSC;
	Settings.DisableSampleCaching = FALSE;
	Settings.DisableMasterVolume = TRUE;
	Settings.Mouse = FALSE;
	Settings.SuperScope = FALSE;
	Settings.MultiPlayer5 = FALSE;
	//	Settings.ControllerOption = SNES_MULTIPLAYER5;
	Settings.ControllerOption = 0;

	Settings.InterpolatedSound = TRUE;
	Settings.StarfoxHack = TRUE;
	
	Settings.ForceTransparency = FALSE;
	Settings.Transparency = TRUE;
	Settings.SixteenBit = TRUE;
	
	Settings.SupportHiRes = FALSE;
	Settings.NetPlay = FALSE;
	Settings.ServerName [0] = 0;
	Settings.AutoSaveDelay = 30;
	Settings.ApplyCheats = TRUE;
	Settings.TurboMode = FALSE;
	Settings.TurboSkipFrames = 15;
	Settings.ThreadSound = FALSE;
	Settings.SoundSync = FALSE;
	//Settings.NoPatch = true;		

	Settings.SuperFX = TRUE;
	Settings.DSP1Master = TRUE;
	Settings.SA1 = TRUE;
	Settings.C4 = TRUE;
	Settings.SDD1 = TRUE;

	GFX.Pitch = 320 * 2;
	GFX.RealPitch = 320 * 2;
	GFX.Screen = (uint8 *) sal_VideoGetBuffer();
	
	GFX.SubScreen = (uint8 *)malloc(GFX.RealPitch * 480 * 2); 
	GFX.ZBuffer =  (uint8 *)malloc(GFX.RealPitch * 480 * 2); 
	GFX.SubZBuffer = (uint8 *)malloc(GFX.RealPitch * 480 * 2);
	GFX.Delta = (GFX.SubScreen - GFX.Screen) >> 1;
	GFX.PPL = GFX.Pitch >> 1;
	GFX.PPLx2 = GFX.Pitch;
	GFX.ZPitch = GFX.Pitch >> 1;
	
	if (Settings.ForceNoTransparency)
         Settings.Transparency = FALSE;

	if (Settings.Transparency)
         Settings.SixteenBit = TRUE;

	Settings.HBlankStart = (256 * Settings.H_Max) / SNES_HCOUNTER_MAX;

	if (!Memory.Init () || !S9xInitAPU())
	{
		S9xMessage (0,0,"Failed to init memory");
		return SAL_ERROR;
	}

	//S9xInitSound ();
	
	//S9xSetRenderPixelFormat (RGB565);
	S9xSetSoundMute (TRUE);

	if (!S9xGraphicsInit ())
	{
         	S9xMessage (0,0,"Failed to init graphics");
		return SAL_ERROR;
	}

	return SAL_OK;
}

void _makepath (char *path, const char *, const char *dir,
	const char *fname, const char *ext)
{
	if (dir && *dir)
	{
		strcpy (path, dir);
		strcat (path, "/");
	}
	else
	*path = 0;
	strcat (path, fname);
	if (ext && *ext)
	{
		strcat (path, ".");
		strcat (path, ext);
	}
}

void _splitpath (const char *path, char *drive, char *dir, char *fname,
	char *ext)
{
	*drive = 0;

	char *slash = strrchr ((char*)path, '/');
	if (!slash)
		slash = strrchr ((char*)path, '\\');

	char *dot = strrchr ((char*)path, '.');

	if (dot && slash && dot < slash)
		dot = NULL;

	if (!slash)
	{
		strcpy (dir, "");
		strcpy (fname, path);
		if (dot)
		{
			*(fname + (dot - path)) = 0;
			strcpy (ext, dot + 1);
		}
		else
			strcpy (ext, "");
	}
	else
	{
		strcpy (dir, path);
		*(dir + (slash - path)) = 0;
		strcpy (fname, slash + 1);
		if (dot)
		{
			*(fname + (dot - slash) - 1) = 0;
			strcpy (ext, dot + 1);
		}
		else
			strcpy (ext, "");
	}
} 

extern "C"
{

int mainEntry(int argc, char* argv[])
{
	int ref = 0;

	s32 event=EVENT_NONE;

	/* Dynarec runtime-codegen self-test. Runs BEFORE sal_Init(), so it touches
	 * no framebuffer or input and is safe over a plain SSH shell:
	 *   PIKO_JIT_SELFTEST=1 ./PocketSNES
	 * Proves emit-and-run native code works on this PXA255 before we build the
	 * 65816 translator on top. Exits with the result; never falls through to
	 * the emulator. */
	if (getenv("PIKO_JIT_SELFTEST")) {
		int ok = S9xDynSelfTest();
		exit(ok ? 0 : 1);
	}
	/*
	 * PIKO_MAX_SECONDS: stop by ourselves after N seconds.
	 *
	 * This is not a convenience. There is no way to stop this process from the
	 * device's own shell: its busybox ships neither a `kill` applet nor a
	 * `timeout` one, and ash has no kill builtin either -- `kill -9 $pid` exits
	 * 127. A run started over ssh and left alone therefore cannot be ended
	 * except by dropping the connection and hoping SIGHUP lands, which is not
	 * reliable, or by rebooting the board. With matchbox-fbrun holding the
	 * graphical session down for the duration, that means the screen stays dark
	 * until someone power-cycles it.
	 *
	 * The bound is on TIME rather than frames on purpose: if the emulated
	 * machine wedges and stops producing frames, a frame count never reaches
	 * its limit and the bound silently does not exist.
	 */
	/*
	 * Build stamp. Several hardware runs today were diagnosed against a binary
	 * that had failed to copy, and the only thing that ever caught it was an
	 * opcode count that happened to change. Say which build this is, always.
	 */
	fprintf(stderr, "PIKO build %s %s\n", __DATE__, __TIME__);
	piko_start = time(NULL);
	{
		const char *ms = getenv("PIKO_MAX_SECONDS");
		if (ms && *ms)
			piko_deadline = time(NULL) + (time_t)strtoul(ms, NULL, 0);
	}
#ifdef PIKO_DYNAREC_PROFILE
	/* PROFILE build: PIKO_DYN_PROFILE=1 turns on the observation-only opcode/
	 * hot-block probe in the CPU dispatch; it dumps a summary to stderr
	 * periodically (every ~1M instructions). */
	if (getenv("PIKO_DYN_PROFILE")) {
		dyn_cache_init();
		dyn_profile_on = 1;
	}
#endif
#ifdef PIKO_DYNAREC_VERIFY
	/* VERIFY build: PIKO_DYN_VERIFY=1 diffs each translatable opcode's generated
	 * block against the interpreter on live game code. Interpreter stays
	 * authoritative; the generated code only touches a scratch copy. */
	if (getenv("PIKO_DYN_VERIFY")) {
		dyn_verify_init();
		dyn_verify_on = 1;
	}
#endif
#ifdef PIKO_DYNAREC_EXEC
	/* EXEC build: PIKO_DYN_EXEC=1 lets translated blocks drive the CPU. */
	if (getenv("PIKO_DYN_EXEC")) {
		/* Before dyn_exec_init: it prints which mode it armed in. */
		if (getenv("PIKO_DYN_WRAM_BLOCKS")) dyn_exec_wram_blocks = 1;
		dyn_exec_init();
		dyn_exec_on = 1;
	}
#endif
#ifdef PIKO_DYNAREC_WRAMSTAT
	/* WRAMSTAT build: PIKO_DYN_WRAM=1 measures writes onto WRAM pages that hold
	 * code. Implies EXEC, which the Makefile arranges; arm both here so a run
	 * cannot end up with the write hook live and no code-page map to test it
	 * against -- that combination reports zero and looks like good news. */
	if (getenv("PIKO_DYN_WRAM")) {
		dyn_exec_init();
		dyn_exec_on = 1;
		dyn_wram_init();
		dyn_wram_on = 1;
		/* The measurement counts writes onto code pages even when WRAM blocks
		 * are not being cached -- that is the whole point of it -- so the write
		 * hook has to be live regardless of dyn_exec_wram_blocks. */
		dyn_wram_tracking = 1;
	}
#endif

	sal_Init();
	sal_VideoInit(16,SAL_RGB(0,0,0),Memory.ROMFramesPerSecond);

	mRomName[0]=0;
	if (argc >= 2)
 		strcpy(mRomName, argv[1]); // Record ROM name

	MenuInit(sal_DirectoryGetHome(), &mMenuOptions);


	if(SnesInit() == SAL_ERROR)
	{
		sal_Reset();
		return 0;
	}

	while(1)
	{
		mInMenu=1;
		event=MenuRun(mRomName);
		mInMenu=0;

		if(event==EVENT_LOAD_ROM)
		{
			if(mTempState) free(mTempState);
			mTempState=NULL;
			if(SnesRomLoad() == SAL_ERROR)
			{
				MenuMessageBox("Failed to load rom",mRomName,"Press any button to continue", MENU_MESSAGE_BOX_MODE_PAUSE);
				sal_Reset();
		    		return 0;
			}
			else
			{
				event=EVENT_RUN_ROM;
		  	}
		}

		if(event==EVENT_RESET_ROM)
		{
			S9xReset();
			event=EVENT_RUN_ROM;
		}

		if(event==EVENT_RUN_ROM)
		{
			if(mMenuOptions.fullScreen)
			{
				sal_VideoSetScaling(SNES_WIDTH,SNES_HEIGHT);
			}

			if(mMenuOptions.transparency)	Settings.Transparency = TRUE;
			else Settings.Transparency = FALSE;

			sal_AudioSetVolume(mMenuOptions.volume,mMenuOptions.volume);
			sal_CpuSpeedSet(mMenuOptions.cpuSpeed);	
			sal_VideoClear(0);
			sal_VideoFlip(1);
			sal_VideoClear(0);
			sal_VideoFlip(1);
			if(mMenuOptions.soundEnabled) 	
				RunSound();
			else	RunNoSound();

			event=EVENT_NONE;
		}

		if(event==EVENT_EXIT_APP) break;	
	}

	if(mTempState) free(mTempState);
	mTempState=NULL;
	
	S9xGraphicsDeinit();
	S9xDeinitAPU();
	Memory.Deinit();

	free(GFX.SubZBuffer);
	free(GFX.ZBuffer);
	free(GFX.SubScreen);
	GFX.SubZBuffer=NULL;
	GFX.ZBuffer=NULL;
	GFX.SubScreen=NULL;

	sal_Reset();
  	return 0;
}

}






