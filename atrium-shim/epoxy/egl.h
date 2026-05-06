/*
 * Atrium shim: minimal epoxy/egl.h for hosts where libepoxy was built
 * without EGL support (notably macOS, where Apple has no native EGL).
 * Provides just the type declarations so QEMU's ui/egl-helpers.h
 * compiles. Runtime gating (qemu_egl_display == NULL) keeps every
 * EGL code path inert; venus output flows through virglrenderer's
 * render-server process and reaches the cocoa display via the
 * standard 2D scanout path.
 */
#ifndef ATRIUM_SHIM_EPOXY_EGL_H
#define ATRIUM_SHIM_EPOXY_EGL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void *EGLDisplay;
typedef void *EGLConfig;
typedef void *EGLContext;
typedef void *EGLSurface;
typedef void *EGLImage;
typedef void *EGLImageKHR;
typedef void *EGLSync;
typedef void *EGLSyncKHR;
typedef void *EGLClientBuffer;
typedef void *EGLNativeDisplayType;
typedef void *EGLNativeWindowType;
typedef void *EGLNativePixmapType;
typedef void *EGLDeviceEXT;

typedef int  EGLint;
typedef unsigned int EGLBoolean;
typedef unsigned int EGLenum;
typedef intptr_t EGLAttrib;
typedef uint64_t EGLuint64KHR;

#define EGL_NO_DISPLAY        ((EGLDisplay)0)
#define EGL_NO_CONTEXT        ((EGLContext)0)
#define EGL_NO_SURFACE        ((EGLSurface)0)
#define EGL_NO_IMAGE          ((EGLImage)0)
#define EGL_NO_IMAGE_KHR      ((EGLImageKHR)0)
#define EGL_NO_SYNC           ((EGLSync)0)
#define EGL_NO_SYNC_KHR       ((EGLSyncKHR)0)
#define EGL_FALSE             0
#define EGL_TRUE              1
#define EGL_SUCCESS           0x3000
#define EGL_DEFAULT_DISPLAY   ((EGLNativeDisplayType)0)

typedef void (*__eglMustCastToProperFunctionPointerType)(void);
typedef int EGLNativeFenceFDANDROID;

#ifdef __cplusplus
}
#endif

#endif
