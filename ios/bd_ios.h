/* Bush Drift on iOS (session 06): the few things the iPhone needs beyond the shared phone code. See ios/README.md. */
#ifndef BD_IOS_H
#define BD_IOS_H

/* before InitWindow: SDL hints (landscape, home bar), the working folder (the app bundle, where the game's files
 * are) and the writable paths for the records and the live tuning file */
void bd_ios_setup(const char **records_path, const char **tuning_path);

/* after InitWindow: raylib's SDL backend leaves the screen size at 0 x 0 for "the whole display"; take SDL's
 * window size (points) instead */
void bd_ios_fit_screen(void);

/* at the top of each frame: if the app is in the background, wait there until it comes back (no drawing) */
void bd_ios_wait_while_background(void);

/* the screen edges kept clear of buttons (notch, rounded corners, home bar), in raylib screen units */
void bd_ios_safe_area(float *l, float *r, float *t, float *b);

/* after BeginDrawing: draw in screen units over the full pixel resolution (raylib's SDL backend sets the
 * viewport in points, which would fill only part of a 3x screen) */
void bd_ios_screen_viewport(void);

/* after InitAudioDevice: the Ambient audio session (the silent switch mutes the game, other music keeps playing) */
void bd_ios_audio_ambient(void);

#endif
