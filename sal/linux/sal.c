
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <dirent.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/kd.h>
#include <SDL/SDL.h>       /* only for SDL_Init/SDL_Quit -- see sal_Init() */
#include "sal.h"

/*
 * Zaurus SL-C860 (piko) video + input backend.
 *
 * WHY THIS REPLACES THE SDL VIDEO PATH: PocketSNES upstream drives video
 * through SDL 1.2's fbcon backend with
 *   SDL_SetVideoMode(320, 240, 16, SDL_HWSURFACE | SDL_DOUBLEBUF)
 * On this board that does two things that are fatal here:
 *
 *   1. It asks the fixed 640x480 w100 panel to switch to a 320x240 MODE.
 *      w100fb can't do a real mode change and the FBIOPUT_VSCREENINFO for
 *      320x240 takes the kernel down (confirmed: the crash is inside
 *      SDL_SetVideoMode, before the first flip, and reboots the device).
 *   2. SDL_DOUBLEBUF page-flips via FBIOPAN_DISPLAY every frame -- the
 *      w100 pan/vsync path that piko's docs/DEADLETTER-W100-VSYNC.md flags
 *      as fragile.
 *
 * The sibling otQuake/otRetro framebuffer builds already solved tear-free
 * output on this exact panel (see ~/Code/otQuake/src/vid_fb.c): never
 * mode-switch -- keep the panel at its native 640x480 -- and page-flip via
 * a DOUBLED virtual framebuffer + FBIOPAN_DISPLAY, which piko's w100fb now
 * supports (ypanstep, added 2026-07-27) with an atomic vblank-edge flip.
 * This file ports that approach into PocketSNES's SAL:
 *
 *   - The emulator core still renders into a plain 320x240x16bpp buffer
 *     (sal_VideoGetBuffer() -- GFX.Screen, GFX.Pitch=320*2 in main.cpp are
 *     unchanged, and every sal_common.c menu/text helper draws into it too).
 *   - sal_VideoFlip() 2x pixel-doubles that 320x240 buffer into the
 *     currently-invisible fb page (320*2==640, 240*2==480, an exact fit)
 *     and flips to it with FBIOPAN_DISPLAY. No mode switch ever happens.
 *
 * Input likewise moves off SDL (SDL 1.2 fbcon delivers keyboard events
 * through the very video subsystem we're no longer using) to reading
 * /dev/input/event* evdev directly, exactly as otQuake does. matchbox-fbrun
 * stops Xfbdev -- which holds an EVIOCGRAB on the keyboard/touchscreen --
 * before exec'ing us, so the devices are ours to open.
 *
 * SDL stays linked only for sal_sound.c (audio, itself a graceful no-op
 * since piko's libSDL is built --disable-audio); sal_timer.c is already
 * pure gettimeofday.
 */

#define PALETTE_BUFFER_LENGTH	256*2*4

static u32 mPaletteBuffer[PALETTE_BUFFER_LENGTH];
static u32 *mPaletteCurr=(u32*)&mPaletteBuffer[0];
static u32 *mPaletteEnd=(u32*)&mPaletteBuffer[PALETTE_BUFFER_LENGTH];

s32 mCpuSpeedLookup[1]={0};

#include <sal_common.h>

/* ---- framebuffer state ------------------------------------------------ */
static int   fb_fd            = -1;
static unsigned char *fb_mem  = MAP_FAILED;
static int   fb_width         = 0;
static int   fb_height        = 0;
static int   fb_bpp           = 0;
static int   fb_line_len      = 0;
static int   fb_page_bytes    = 0;
static int   fb_mem_size      = 0;
static int   fb_pageflip      = 0;
static int   fb_back_page     = 1;

/* the 320x240x16 buffer the emulator + menu render into */
static u16  *mFrameBuf        = NULL;
/* one doubled fb row, built in cached memory then burst-copied out */
static unsigned int *blit_row = NULL;

/*
 * Per-page shadow of what each fb page currently displays, in the source's
 * own 320x240x16 form. sal_VideoFlip compares each source scanline against
 * it and skips rows the page already shows (and narrows changed rows to just
 * the differing span), so a mostly-static screen -- an RPG field/menu -- only
 * pays for the pixels that actually moved instead of a full 640x480 uncached
 * blit every frame.
 *
 * It has to be PER PAGE, not a single last-frame copy: with page flipping the
 * page we draw into now was last painted two frames ago, so "what changed
 * since last frame" is the wrong question -- "what does THIS page already
 * show" is the right one, and only a per-page shadow answers it. Index 0 is
 * also the single-buffer case. blit_shadow_valid gates the first paint of
 * each page (nothing to compare against yet). */
static u16  *blit_shadow[2]   = { NULL, NULL };
static int   blit_shadow_valid[2] = { 0, 0 };

/* VT ownership so fbcon's text cursor stops drawing over us */
static int   tty_fd           = -1;
static int   tty_in_graphics  = 0;

/* ---- evdev input state ------------------------------------------------ */
#define MAX_INPUT_FDS 8
static int input_fds[MAX_INPUT_FDS];
static int input_nfds = 0;
static int input_grab = 1;       /* EVIOCGRAB by default; non-fatal if it fails */
static u32 inputHeld  = 0;

/* ---- evdev keycode -> SAL_INPUT_* bit --------------------------------- *
 * Mirrors the logical mapping the SDL path used (LCTRL=A, LALT=B, SPACE=X,
 * LSHIFT=Y, TAB=L, BACKSPACE=R, RETURN=START, ESCAPE=SELECT, arrows=dpad).
 * Physical Zaurus button assignment can be refined later; this keeps the
 * same intent as upstream. */
static u32 evkey_to_sal(unsigned int code)
{
	switch (code) {
	case KEY_LEFTCTRL:
	case KEY_RIGHTCTRL:  return SAL_INPUT_A;
	case KEY_LEFTALT:
	case KEY_RIGHTALT:   return SAL_INPUT_B;
	case KEY_SPACE:      return SAL_INPUT_X;
	case KEY_LEFTSHIFT:
	case KEY_RIGHTSHIFT: return SAL_INPUT_Y;
	case KEY_TAB:        return SAL_INPUT_L;
	case KEY_BACKSPACE:  return SAL_INPUT_R;
	case KEY_ENTER:
	case KEY_KPENTER:    return SAL_INPUT_START;
	case KEY_ESC:        return SAL_INPUT_SELECT;
	case KEY_UP:         return SAL_INPUT_UP;
	case KEY_DOWN:       return SAL_INPUT_DOWN;
	case KEY_LEFT:       return SAL_INPUT_LEFT;
	case KEY_RIGHT:      return SAL_INPUT_RIGHT;
	default:             return 0;
	}
}

/* ---- VT graphics-mode handling (from otQuake vid_fb.c) ---------------- */
static void tty_graphics_mode(void)
{
	tty_fd = open("/dev/tty0", O_RDWR);
	if (tty_fd < 0)
		tty_fd = open("/dev/console", O_RDWR);
	if (tty_fd < 0) {
		fprintf(stderr, "sal: no console tty; fbcon cursor may show\n");
		return;
	}
	if (ioctl(tty_fd, KDSETMODE, KD_GRAPHICS) < 0) {
		fprintf(stderr, "sal: KDSETMODE KD_GRAPHICS failed (%s)\n", strerror(errno));
		close(tty_fd);
		tty_fd = -1;
		return;
	}
	tty_in_graphics = 1;
}

static void tty_text_mode(void)
{
	if (tty_fd >= 0) {
		if (tty_in_graphics)
			ioctl(tty_fd, KDSETMODE, KD_TEXT);
		close(tty_fd);
		tty_fd = -1;
	}
	tty_in_graphics = 0;
}

/* ---- teardown --------------------------------------------------------- */
static void sal_VideoCleanup(void)
{
	int i;
	tty_text_mode();
	if (fb_mem != MAP_FAILED) {
		memset(fb_mem, 0, (size_t)fb_mem_size);
		munmap(fb_mem, (size_t)fb_mem_size);
		fb_mem = MAP_FAILED;
	}
	if (fb_fd >= 0) { close(fb_fd); fb_fd = -1; }
	free(blit_shadow[0]); blit_shadow[0] = NULL;
	free(blit_shadow[1]); blit_shadow[1] = NULL;
	blit_shadow_valid[0] = blit_shadow_valid[1] = 0;
	for (i = 0; i < input_nfds; i++) {
		if (input_grab)
			ioctl(input_fds[i], EVIOCGRAB, 0);
		close(input_fds[i]);
	}
	input_nfds = 0;
}

static void sal_SigHandler(int sig)
{
	(void)sig;
	sal_VideoCleanup();     /* ungrab input + KD_TEXT: don't leave the console wedged */
	fflush(stdout);
	_exit(0);
}

/* ---- input ------------------------------------------------------------ *
 * repeat==0 : held mode  -- return the persistent held bitmask (gameplay).
 * repeat!=0 : menu mode  -- edge on first press, then a slow software
 *             auto-repeat, using the shared mInputRepeatTimer[] table.
 * (SDL_EnableKeyRepeat is gone; we generate repeats ourselves.) */
static u32 sal_Input(int repeat)
{
	struct input_event ev;
	int i;

	for (i = 0; i < input_nfds; i++) {
		while (read(input_fds[i], &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
			u32 bit;
			if (ev.type != EV_KEY)
				continue;
			if (ev.value == 2)      /* kernel autorepeat: ignore, we do our own */
				continue;
			bit = evkey_to_sal(ev.code);
			if (!bit)
				continue;
			if (ev.value)
				inputHeld |= bit;
			else
				inputHeld &= ~bit;
		}
	}

	if (!repeat)
		return inputHeld;

	/* menu auto-repeat */
	{
		u32 timer = sal_TimerRead();
		mInputRepeat = 0;
		for (i = 0; i < 32; i++) {
			u32 mask = (u32)1 << i;
			if (inputHeld & mask) {
				if (mInputRepeatTimer[i] == 0) {
					/* first press: fire now, long delay before repeat */
					mInputRepeat |= mask;
					mInputRepeatTimer[i] = timer + 30;
				} else if (mInputRepeatTimer[i] < timer) {
					mInputRepeat |= mask;
					mInputRepeatTimer[i] = timer + 8;
				}
			} else {
				mInputRepeatTimer[i] = 0;
			}
		}
		return mInputRepeat;
	}
}

u32 sal_InputPollRepeat()
{
	return sal_Input(1);
}

u32 sal_InputPoll()
{
	return sal_Input(0);
}

/* ---- CPU speed (no-ops on this board, kept for the SAL contract) ------ */
void sal_CpuSpeedSet(u32 mhz) { (void)mhz; }

u32 sal_CpuSpeedNext(u32 currSpeed)
{
	u32 newSpeed=currSpeed+1;
	if(newSpeed > 500) newSpeed = 500;
	return newSpeed;
}

u32 sal_CpuSpeedPrevious(u32 currSpeed)
{
	u32 newSpeed=currSpeed-1;
	if(newSpeed > 500) newSpeed = 0;
	return newSpeed;
}

u32 sal_CpuSpeedNextFast(u32 currSpeed)
{
	u32 newSpeed=currSpeed+10;
	if(newSpeed > 500) newSpeed = 500;
	return newSpeed;
}

u32 sal_CpuSpeedPreviousFast(u32 currSpeed)
{
	u32 newSpeed=currSpeed-10;
	if(newSpeed > 500) newSpeed = 0;
	return newSpeed;
}

/* ---- init ------------------------------------------------------------- */
s32 sal_Init(void)
{
	char path[32];
	int i, fd;

	/* SDL only for sal_sound.c's audio path (a graceful no-op on piko's
	 * --disable-audio libSDL). Non-fatal: video and input are ours now. */
	SDL_Init(SDL_INIT_TIMER);

	sal_TimerInit(60);
	memset(mInputRepeatTimer, 0, sizeof(mInputRepeatTimer));

	signal(SIGINT,  sal_SigHandler);
	signal(SIGTERM, sal_SigHandler);

	/* open evdev input devices (Xfbdev's grab is already gone: matchbox-fbrun
	 * stopped the X session before exec'ing us). */
	for (i = 0; i < 16 && input_nfds < MAX_INPUT_FDS; i++) {
		snprintf(path, sizeof(path), "/dev/input/event%d", i);
		fd = open(path, O_RDONLY | O_NONBLOCK);
		if (fd < 0)
			continue;
		if (input_grab && ioctl(fd, EVIOCGRAB, 1) < 0) {
			/* non-fatal: without the grab, keys also reach the console,
			 * but the emulator still gets them. */
			fprintf(stderr, "sal: EVIOCGRAB %s failed (%s)\n", path, strerror(errno));
		}
		input_fds[input_nfds++] = fd;
	}
	if (input_nfds == 0)
		fprintf(stderr, "sal: warning: no /dev/input/event* devices found\n");

	return SAL_OK;
}

/* ---- video ------------------------------------------------------------ */
u32 sal_VideoInit(u32 bpp, u32 color, u32 refreshRate)
{
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;

	mBpp = bpp;
	mRefreshRate = refreshRate;

	if (fb_fd >= 0)     /* already set up (bpp is always 16 here) */
		return SAL_OK;

	fb_fd = open("/dev/fb0", O_RDWR);
	if (fb_fd < 0)
		fb_fd = open("/dev/fb/0", O_RDWR);
	if (fb_fd < 0) {
		sal_LastErrorSet("open /dev/fb0 failed");
		return SAL_ERROR;
	}

	if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo) < 0 ||
	    ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo) < 0) {
		sal_LastErrorSet("fb FBIOGET_*SCREENINFO failed");
		close(fb_fd); fb_fd = -1;
		return SAL_ERROR;
	}

	/* NOTE: we deliberately do NOT FBIOPUT a new xres/yres -- keeping the
	 * panel's native mode is the whole point (see file header). */
	fb_width      = (int)vinfo.xres;
	fb_height     = (int)vinfo.yres;
	fb_bpp        = (int)vinfo.bits_per_pixel;
	fb_line_len   = (int)finfo.line_length;
	fb_page_bytes = fb_line_len * fb_height;
	fb_mem_size   = fb_page_bytes;

	/* Doubled virtual fb for tear-free page flipping -- only if the driver
	 * advertises panning (ypanstep>0); otherwise fall back to single-buffer. */
	if (finfo.ypanstep > 0) {
		struct fb_var_screeninfo req = vinfo;
		req.yres_virtual = vinfo.yres * 2;
		req.xoffset = 0;
		req.yoffset = 0;
		if (ioctl(fb_fd, FBIOPUT_VSCREENINFO, &req) == 0 &&
		    req.yres_virtual >= vinfo.yres * 2) {
			fb_mem_size = fb_page_bytes * 2;
			fb_pageflip = 1;
			fb_back_page = 1;
		} else {
			fprintf(stderr, "sal: doubled virtual fb unavailable, single-buffer\n");
		}
	}

	fb_mem = mmap(NULL, (size_t)fb_mem_size, PROT_READ | PROT_WRITE,
	              MAP_SHARED, fb_fd, 0);
	if (fb_mem == MAP_FAILED) {
		sal_LastErrorSet("mmap /dev/fb0 failed");
		close(fb_fd); fb_fd = -1;
		return SAL_ERROR;
	}
	memset(fb_mem, 0, (size_t)fb_mem_size);

	/*
	 * The emulator/menu render target. The DISPLAYED area is 320x240 (what
	 * sal_VideoFlip 2x-scales out), but the snes9x core renders into this
	 * same buffer (GFX.Screen) at up to 640px stride x ~478 lines in hi-res
	 * interlace mode (gfx.cpp: RenderedScreenWidth=512, GFX.Pitch=RealPitch*2,
	 * height<<1) BEFORE it downscales to the display size. Upstream got away
	 * with a nominally-320x240 SDL surface because on fbcon it was backed by
	 * the full 640x480(x2) framebuffer, so those over-stride writes landed in
	 * slack. Our buffer is a plain heap allocation, so it must be sized for
	 * that worst-case render or the core smashes the heap (observed as a wild
	 * mainEntry() re-entry with a garbage argc). 640x512 covers 640px stride
	 * x 478 interlaced lines with margin. */
	#define SAL_FB_ALLOC_W 640
	#define SAL_FB_ALLOC_H 512
	mFrameBuf = (u16 *)malloc((size_t)SAL_FB_ALLOC_W * SAL_FB_ALLOC_H * sizeof(u16));
	/* one doubled fb row of cached scratch (fb_width pixels = fb_width/2 words
	 * for the 2x path; size to fb_width words to also cover a 1:1 fallback). */
	blit_row = (unsigned int *)malloc((size_t)fb_width * sizeof(unsigned int));
	/* dirty-row shadows: one per fb page (only [0] used when single-buffered) */
	blit_shadow[0] = (u16 *)malloc((size_t)SAL_SCREEN_WIDTH * SAL_SCREEN_HEIGHT * sizeof(u16));
	blit_shadow[1] = (u16 *)malloc((size_t)SAL_SCREEN_WIDTH * SAL_SCREEN_HEIGHT * sizeof(u16));
	blit_shadow_valid[0] = blit_shadow_valid[1] = 0;
	if (!mFrameBuf || !blit_row || !blit_shadow[0] || !blit_shadow[1]) {
		sal_LastErrorSet("out of memory for video buffers");
		return SAL_ERROR;
	}
	memset(mFrameBuf, 0, (size_t)SAL_FB_ALLOC_W * SAL_FB_ALLOC_H * sizeof(u16));

	tty_graphics_mode();

	fprintf(stderr, "sal: fb %dx%d %dbpp line=%d %s\n",
	        fb_width, fb_height, fb_bpp, fb_line_len,
	        fb_pageflip ? "(page-flip)" : "(single-buffer)");

	sal_VideoClear(color);
	sal_VideoFlip(1);
	return SAL_OK;
}

void sal_VideoFlip(s32 vsync)
{
	unsigned char *dst_base;
	int y;
	(void)vsync;

	if (fb_mem == MAP_FAILED || !mFrameBuf || fb_bpp != 16)
		return;

	dst_base = fb_pageflip ? fb_mem + (fb_back_page * fb_page_bytes) : fb_mem;

	if (blit_row &&
	    SAL_SCREEN_WIDTH * 2 == fb_width &&
	    SAL_SCREEN_HEIGHT * 2 == fb_height) {
		/* exact 2x: one source pixel -> one 32-bit word (two identical
		 * RGB565 pixels); each source row -> two adjacent fb rows.
		 * Building the row in cached memory then memcpy'ing lets libc burst
		 * it out with LDM/STM instead of single uncached stores.
		 *
		 * Dirty-row skipping (see blit_shadow): the page already shows every
		 * row that matches the shadow, so unchanged rows cost only a cached
		 * 640-byte memcmp and no uncached writes; changed rows are narrowed to
		 * the differing [x0..x1] span. */
		int page   = fb_pageflip ? fb_back_page : 0;
		u16 *shadow = blit_shadow[page];
		int valid  = shadow && blit_shadow_valid[page];
		for (y = 0; y < SAL_SCREEN_HEIGHT; y++) {
			const u16 *srow = mFrameBuf + (size_t)y * SAL_SCREEN_WIDTH;
			u16 *shrow = shadow ? shadow + (size_t)y * SAL_SCREEN_WIDTH : NULL;
			int x0 = 0, x1 = SAL_SCREEN_WIDTH - 1, span, k;
			unsigned char *d0, *d1;

			if (valid) {
				if (!memcmp(srow, shrow, (size_t)SAL_SCREEN_WIDTH * sizeof(u16)))
					continue;               /* page already shows this row */
				/* memcmp said they differ, so these two scans meet */
				while (srow[x0] == shrow[x0]) x0++;
				while (srow[x1] == shrow[x1]) x1--;
			}
			span = x1 - x0 + 1;

			for (k = 0; k < span; k++) {
				unsigned int cc = srow[x0 + k];
				blit_row[k] = cc | (cc << 16);
			}
			d0 = dst_base + (size_t)(y * 2) * fb_line_len
			     + (size_t)x0 * sizeof(unsigned int);
			d1 = d0 + fb_line_len;
			memcpy(d0, blit_row, (size_t)span * sizeof(unsigned int));
			memcpy(d1, blit_row, (size_t)span * sizeof(unsigned int));

			if (shrow)
				memcpy(shrow + x0, srow + x0, (size_t)span * sizeof(u16));
		}
		if (shadow)
			blit_shadow_valid[page] = 1;
	} else {
		/* fallback: 1:1 top-left blit, black borders (unexpected geometry) */
		int bw = SAL_SCREEN_WIDTH  < fb_width  ? SAL_SCREEN_WIDTH  : fb_width;
		int bh = SAL_SCREEN_HEIGHT < fb_height ? SAL_SCREEN_HEIGHT : fb_height;
		int stride_px = fb_line_len / 2;
		for (y = 0; y < fb_height; y++) {
			u16 *drow = (u16 *)dst_base + (size_t)y * stride_px;
			if (y < bh) {
				const u16 *srow = mFrameBuf + (size_t)y * SAL_SCREEN_WIDTH;
				int x;
				for (x = 0; x < bw; x++)      drow[x] = srow[x];
				for (; x < fb_width; x++)     drow[x] = 0;
			} else {
				memset(drow, 0, (size_t)fb_width * sizeof(u16));
			}
		}
	}

	if (fb_pageflip) {
		struct fb_var_screeninfo pan;
		memset(&pan, 0, sizeof(pan));
		if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &pan) < 0) {
			fprintf(stderr, "sal: FBIOGET_VSCREENINFO failed, page-flip off\n");
			fb_pageflip = 0;
			return;
		}
		pan.xoffset = 0;
		pan.yoffset = fb_back_page * fb_height;
		if (ioctl(fb_fd, FBIOPAN_DISPLAY, &pan) < 0) {
			fprintf(stderr, "sal: FBIOPAN_DISPLAY failed (%s), page-flip off\n",
			        strerror(errno));
			fb_pageflip = 0;
			return;
		}
		fb_back_page = 1 - fb_back_page;
	}
}

void *sal_VideoGetBuffer()
{
	return (void*)mFrameBuf;
}

u32 sal_VideoSetScaling(s32 width, s32 height)
{
	(void)width; (void)height;
	return SAL_ERROR;
}

void sal_VideoPaletteSync()
{
}

void sal_VideoPaletteSet(u32 index, u32 color)
{
	*mPaletteCurr++=index;
	*mPaletteCurr++=color;
	if(mPaletteCurr>mPaletteEnd) mPaletteCurr=&mPaletteBuffer[0];
}

void sal_Reset(void)
{
	sal_AudioClose();
	sal_VideoCleanup();
	SDL_Quit();
}

int mainEntry(int argc, char *argv[]);

int main(int argc, char *argv[])
{
	return mainEntry(argc, argv);
}
