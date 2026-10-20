/* iOS: raylib includes <GLES2/gl2ext.h> after gl3.h; Apple's ES 3 extension header is already in (GLES3/gl3.h).
 * A few extension names raylib uses are not in Apple's headers; on ES 3 they are the core values below. */
#include <GLES3/gl3.h>
#ifndef GL_DEPTH_COMPONENT24_OES
#define GL_DEPTH_COMPONENT24_OES 0x81A6
#endif
#ifndef GL_DEPTH_COMPONENT32_OES
#define GL_DEPTH_COMPONENT32_OES 0x81A7
#endif
#ifndef GL_RED_EXT
#define GL_RED_EXT 0x1903
#endif
#ifndef GL_R32F_EXT
#define GL_R32F_EXT 0x822E
#endif
#ifndef GL_RGB32F_EXT
#define GL_RGB32F_EXT 0x8815
#endif
#ifndef GL_RGBA32F_EXT
#define GL_RGBA32F_EXT 0x8814
#endif
