/* iOS: raylib includes <GLES3/gl3.h>; Apple's OpenGL ES 3 headers live elsewhere. */
#ifndef BD_IOS_GLES3_SHIM
#define BD_IOS_GLES3_SHIM
#ifndef GLES_SILENCE_DEPRECATION
#define GLES_SILENCE_DEPRECATION
#endif
#include <OpenGLES/ES3/gl.h>
#include <OpenGLES/ES3/glext.h>
/* framebuffer 0 is not the screen on iOS (see bd_ios_bind_framebuffer in ios/bd_ios.c) */
void bd_ios_bind_framebuffer(GLenum target, GLuint fb);
#define glBindFramebuffer bd_ios_bind_framebuffer
#ifdef BD_IOS_GLCHECK
#include "../bd_glcheck.h"
#endif
#endif
