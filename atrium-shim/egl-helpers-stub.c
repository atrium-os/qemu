/*
 * Atrium shim: macOS replacement for ui/egl-helpers.c.
 *
 * Real EGL doesn't exist on macOS. Every symbol that ui/egl-helpers.h
 * declares as `extern` is provided here as a NULL/false global so the
 * runtime checks in virtio-gpu-virgl.c (e.g. `if (qemu_egl_display)`)
 * route into the EGL-free venus path. Function entry points that
 * should never run on this host abort loudly so any regression in the
 * runtime gating is visible.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "ui/egl-helpers.h"

EGLDisplay   *qemu_egl_display     = NULL;
EGLConfig     qemu_egl_config      = NULL;
DisplayGLMode qemu_egl_mode        = DISPLAY_GL_MODE_OFF;
bool          qemu_egl_angle_d3d   = false;
EGLContext    qemu_egl_rn_ctx      = NULL;

const char *qemu_egl_get_error_string(void)
{
    return "atrium-shim: EGL not available on this host";
}

static void egl_unreachable(const char *fn)
{
    error_report("atrium-shim: %s called but no EGL on this host; "
                 "code path should be gated on qemu_egl_display != NULL", fn);
    abort();
}

/* Tolerant stubs for cleanup paths that walk possibly-empty fbs. */
void egl_fb_destroy(egl_fb *fb)            { (void)fb; }
void egl_fb_setup_default(egl_fb *fb, int width, int height, int x, int y)
{ (void)fb; (void)width; (void)height; (void)x; (void)y; }

/* Anything else: trip an assertion if reached. */
void egl_fb_setup_for_tex(egl_fb *fb, int w, int h, GLuint t, bool d)
{ (void)fb; (void)w; (void)h; (void)t; (void)d; egl_unreachable(__func__); }
void egl_fb_setup_new_tex(egl_fb *fb, int w, int h)
{ (void)fb; (void)w; (void)h; egl_unreachable(__func__); }
void egl_fb_blit(egl_fb *d, egl_fb *s, bool f)
{ (void)d; (void)s; (void)f; egl_unreachable(__func__); }
void egl_fb_read(DisplaySurface *d, egl_fb *s)
{ (void)d; (void)s; egl_unreachable(__func__); }
void egl_fb_read_rect(DisplaySurface *d, egl_fb *s, int x, int y, int w, int h)
{ (void)d; (void)s; (void)x; (void)y; (void)w; (void)h; egl_unreachable(__func__); }
void egl_texture_blit(QemuGLShader *g, egl_fb *d, egl_fb *s, bool f)
{ (void)g; (void)d; (void)s; (void)f; egl_unreachable(__func__); }
void egl_texture_blend(QemuGLShader *g, egl_fb *d, egl_fb *s, bool f,
                       int x, int y, double sx, double sy)
{ (void)g; (void)d; (void)s; (void)f; (void)x; (void)y; (void)sx; (void)sy;
  egl_unreachable(__func__); }

EGLSurface qemu_egl_init_surface_x11(EGLContext c, EGLNativeWindowType w)
{ (void)c; (void)w; egl_unreachable(__func__); return NULL; }
EGLContext qemu_egl_init_ctx(void)
{ egl_unreachable(__func__); return NULL; }
bool qemu_egl_has_dmabuf(void) { return false; }
bool egl_init(const char *rendernode, DisplayGLMode mode, Error **errp)
{
    (void)rendernode; (void)mode;
    error_setg(errp, "atrium-shim: EGL not available on macOS host");
    return false;
}
