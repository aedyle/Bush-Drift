/* Bush Drift on iOS (session 06): see bd_ios.h. */
#ifndef GLES_SILENCE_DEPRECATION
#define GLES_SILENCE_DEPRECATION
#endif
#include <OpenGLES/ES3/gl.h>
#include <SDL3/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "bd_ios.h"

/* from raylib (not included whole: its names clash with nothing here, but keep this file to SDL and GL) */
int GetScreenWidth(void);
int GetScreenHeight(void);
void SetWindowSize(int width, int height);
void rlViewport(int x, int y, int width, int height);
void rlMatrixMode(int mode);
void rlLoadIdentity(void);
void rlOrtho(double left, double right, double bottom, double top, double znear, double zfar);
#define RL_MODELVIEW 0x1700
#define RL_PROJECTION 0x1701

static volatile int in_background;
static double shown_mean, shown_worst, worst_all, last_ms;   /* frame times for the on-screen readout */
static int slow;
static int watching;

static bool SDLCALL app_events(void *userdata, SDL_Event *e)
{
    (void)userdata;
    /* sent from inside UIKit's own callbacks, so the flag is set before iOS stops the app */
    if (e->type == SDL_EVENT_WILL_ENTER_BACKGROUND) { glFinish(); in_background = 1; }
    else if (e->type == SDL_EVENT_DID_ENTER_FOREGROUND) in_background = 0;
    return true;
}

static char *join(const char *dir, const char *name)
{
    size_t n = strlen(dir) + strlen(name) + 1;
    char *s = malloc(n);
    snprintf(s, n, "%s%s", dir, name);
    return s;
}

void bd_ios_setup(const char **records_path, const char **tuning_path)
{
    SDL_SetHint(SDL_HINT_ORIENTATIONS, "LandscapeLeft LandscapeRight");
    SDL_SetHint(SDL_HINT_IOS_HIDE_HOME_INDICATOR, "2");   /* the home bar dims, and needs a second swipe while driving */
    const char *base = SDL_GetBasePath();                 /* the app bundle */
    if (base && chdir(base) != 0) fprintf(stderr, "ios: could not open %s\n", base);
    char *pref = SDL_GetPrefPath("aedyle", "Bush Drift"); /* Library/Application Support/..., kept across updates */
    if (pref) {
        /* the game's and raylib's console output, for reading back from the Mac (a fresh file each launch) */
        char *log = join(pref, "console.log");
        if (freopen(log, "w", stdout)) { setvbuf(stdout, NULL, _IOLBF, 0); dup2(fileno(stdout), 2); }
        free(log);
        *records_path = join(pref, "records-bush-drift.txt");
        *tuning_path = join(pref, "tuning-live.txt");
        SDL_free(pref);
    }
}

void bd_ios_wait_while_background(void)
{
    if (!watching) { SDL_AddEventWatch(app_events, NULL); watching = 1; }
    static Uint64 last;
    if (in_background) { while (in_background) { SDL_PumpEvents(); SDL_Delay(50); } last = 0; }   /* the time away is not a frame */
    /* the play test: a line every 120 frames with the frame times (on a phone the game's readout starts hidden) */
    static int frames; static double worst, sum;
    last_ms = 0;
    Uint64 now = SDL_GetPerformanceCounter();
    if (last) { double ms = (double)(now - last) * 1000.0 / (double)SDL_GetPerformanceFrequency(); sum += ms; if (ms > worst) worst = ms; last_ms = ms; }
    last = now;
    if (last_ms > 20.0 && frames > 120) slow++;
    if (last_ms > worst_all && frames > 120) worst_all = last_ms;   /* the first two seconds load the road */
    if (++frames % 120 == 0) {
        printf("ios: frames %d-%d: mean %.2f ms (%.0f fps), worst %.1f ms, gl error 0x%x\n", frames - 119, frames, sum / 120, 120000.0 / sum, worst, frames <= 600 ? glGetError() : 0);
        shown_mean = sum / 120; shown_worst = worst;
        sum = worst = 0;
    }
}

void bd_ios_frame_stats(float *mean_ms, float *worst_ms, float *worst_all_ms, int *slow_frames)
{
    *mean_ms = (float)shown_mean; *worst_ms = (float)shown_worst; *worst_all_ms = (float)worst_all; *slow_frames = slow;
}

void bd_ios_fit_screen(void)
{
    SDL_Window *w = SDL_GL_GetCurrentWindow();
    int ww = 0, wh = 0;
    if (w && SDL_GetWindowSize(w, &ww, &wh) && ww > 0) SetWindowSize(ww, wh);
    int pw = 0, ph = 0;
    if (w) SDL_GetWindowSizeInPixels(w, &pw, &ph);
    printf("ios: window %d x %d points, %d x %d pixels, screen %d x %d, view framebuffer %lld\n", ww, wh, pw, ph,
           GetScreenWidth(), GetScreenHeight(), w ? (long long)SDL_GetNumberProperty(SDL_GetWindowProperties(w), SDL_PROP_WINDOW_UIKIT_OPENGL_FRAMEBUFFER_NUMBER, -1) : -2);
}

void bd_ios_safe_area(float *l, float *r, float *t, float *b)
{
    SDL_Window *w = SDL_GL_GetCurrentWindow();
    SDL_Rect a; int ww, wh;
    if (!w || !SDL_GetWindowSafeArea(w, &a) || !SDL_GetWindowSize(w, &ww, &wh) || ww <= 0 || wh <= 0) { *l = *r = *t = *b = 0; return; }
    float sx = (float)GetScreenWidth() / (float)ww, sy = (float)GetScreenHeight() / (float)wh;
    *l = a.x * sx; *t = a.y * sy;
    *r = (ww - a.x - a.w) * sx; *b = (wh - a.y - a.h) * sy;
}

void bd_ios_screen_viewport(void)
{
    SDL_Window *w = SDL_GL_GetCurrentWindow();
    int pw = 0, ph = 0;
    if (!w || !SDL_GetWindowSizeInPixels(w, &pw, &ph) || pw <= 0) return;
    rlViewport(0, 0, pw, ph);
    rlMatrixMode(RL_PROJECTION); rlLoadIdentity();
    rlOrtho(0, GetScreenWidth(), GetScreenHeight(), 0, 0.0, 1.0);
    rlMatrixMode(RL_MODELVIEW); rlLoadIdentity();
}

/* raylib binds framebuffer 0 to draw to the screen; on iOS the screen is SDL's view, which has its own framebuffer
 * (ios/include/GLES3/gl3.h sends raylib's calls here) */
void bd_ios_bind_framebuffer(GLenum target, GLuint fb)
{
    SDL_Window *w = fb == 0 ? SDL_GL_GetCurrentWindow() : NULL;
    if (w) {
        SDL_PropertiesID p = SDL_GetWindowProperties(w);
        fb = (GLuint)SDL_GetNumberProperty(p, SDL_PROP_WINDOW_UIKIT_OPENGL_FRAMEBUFFER_NUMBER, 0);
        glBindFramebuffer(target, fb);
        /* SDL presents whichever renderbuffer is bound, and leaves rebinding the view's to the caller;
         * raylib binds its own (the render textures' depth buffers), so rebind the view's here */
        glBindRenderbuffer(GL_RENDERBUFFER, (GLuint)SDL_GetNumberProperty(p, SDL_PROP_WINDOW_UIKIT_OPENGL_RENDERBUFFER_NUMBER, 0));
        return;
    }
    glBindFramebuffer(target, fb);
}

/* debug only (BD_IOS_GLCHECK, ios/include/bd_glcheck.h): the first GL errors, with the raylib line that made them */
void bd_ios_glcheck(const char *fn, int line)
{
    static int reported;
    GLenum e = glGetError();
    if (e != GL_NO_ERROR && reported < 30) { reported++; printf("ios: GL error 0x%x after %s (rlgl.h line %d)\n", e, fn, line); }
}
