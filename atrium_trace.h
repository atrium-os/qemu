/*
 * atrium_trace.h — single-header cross-boundary tracing.
 *
 * Emits Chrome Trace Format (catapult) JSON events to a per-process file
 * named by the ATRIUM_TRACE_FILE environment variable. When the env var is
 * unset, all probes are no-ops. Designed to work in:
 *   - Userspace processes on macOS host (QEMU, virgl_render_server, MoltenVK)
 *   - Userspace processes on FreeBSD guest (frescod-vulkan-smoke)
 *   - The FreeBSD kernel module (atrium-virtio-gpu) — see ATRIUM_TRACE_KMOD below
 *
 * Cross-host/guest correlation uses CLOCK_REALTIME nanoseconds. HVF doesn't
 * dilate REALTIME meaningfully under macOS, so timelines align to within
 * a few tens of microseconds — enough to localize ms-scale phases.
 *
 * Concat all output files (host + guest) and open in https://ui.perfetto.dev
 * or chrome://tracing for visual analysis.
 *
 * Usage in userspace:
 *   ATRIUM_TRACE_INIT();                          // once at process start
 *   ATRIUM_TRACE_BEGIN("vkQueueSubmit");          // matched pair
 *   ...work...
 *   ATRIUM_TRACE_END("vkQueueSubmit");
 *   ATRIUM_TRACE_INSTANT("fence_emit");           // single point
 *
 * In the FreeBSD kmod:
 *   #define ATRIUM_TRACE_KMOD
 *   #include "atrium_trace.h"
 *   atrium_trace_kmod_log("irq_entry");           // sysctl-buffer-backed
 */

#ifndef ATRIUM_TRACE_H
#define ATRIUM_TRACE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ATRIUM_TRACE_KMOD
/* ---------- FreeBSD kernel-module flavor ---------- */
/* Ring-buffer of (label, nanouptime ns) entries, drained via sysctl. */
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/time.h>

#ifndef ATRIUM_TRACE_KMOD_RING_SIZE
#define ATRIUM_TRACE_KMOD_RING_SIZE 4096
#endif

struct atrium_trace_kmod_entry {
	char     label[32];
	uint64_t ns_realtime;
	uint64_t id;          /* correlation id (fence_id, etc.); 0 = unset */
	uint32_t cpu;
	uint32_t pad;
};

extern struct atrium_trace_kmod_entry atrium_trace_kmod_ring[];
extern volatile uint32_t               atrium_trace_kmod_head;
extern volatile int                    atrium_trace_kmod_enabled;

static inline uint64_t atrium_trace_kmod_now_ns(void) {
	struct timespec ts;
	/* nanotime() reads the hardware clock for sub-µs precision.
	 * getnanotime() is cheaper but has 1/HZ (~1 ms) granularity,
	 * which collapses our submit_enter→woke interval to 0. */
	nanotime(&ts);  /* CLOCK_REALTIME via nanotime, hi-res */
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static inline void atrium_trace_kmod_log_id(const char *label, uint64_t id) {
	uint32_t idx;
	struct atrium_trace_kmod_entry *e;
	if (!atrium_trace_kmod_enabled) return;
	idx = atomic_fetchadd_32(&atrium_trace_kmod_head, 1) % ATRIUM_TRACE_KMOD_RING_SIZE;
	e = &atrium_trace_kmod_ring[idx];
	strlcpy(e->label, label, sizeof(e->label));
	e->ns_realtime = atrium_trace_kmod_now_ns();
	e->id = id;
	e->cpu = curcpu;
}

static inline void atrium_trace_kmod_log(const char *label) {
	atrium_trace_kmod_log_id(label, 0);
}

#else /* !ATRIUM_TRACE_KMOD — userspace flavor */
/* ---------- Userspace flavor (host + guest userspace) ---------- */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#ifdef __APPLE__
#  include <sys/syscall.h>
#  include <pthread.h>
#elif defined(__FreeBSD__)
#  include <pthread_np.h>
#  include <sys/thr.h>
#endif

/* One global FILE per process. Lazy-init at first ATRIUM_TRACE_BEGIN/INSTANT. */
extern FILE              *atrium_trace_file;
extern int                atrium_trace_inited;
extern int                atrium_trace_disabled;
extern pthread_mutex_t    atrium_trace_mtx;

static inline uint64_t atrium_trace_now_us(void) {
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static inline uint64_t atrium_trace_tid(void) {
#ifdef __APPLE__
	uint64_t tid = 0;
	pthread_threadid_np(NULL, &tid);
	return tid;
#elif defined(__FreeBSD__)
	long tid = 0;
	thr_self(&tid);
	return (uint64_t)tid;
#else
	return (uint64_t)pthread_self();
#endif
}

void atrium_trace_init(void);

static inline void atrium_trace_emit_id(const char *name, char ph, uint64_t id) {
	if (atrium_trace_disabled) return;
	if (!atrium_trace_inited) atrium_trace_init();
	if (atrium_trace_disabled) return;  /* init may have failed (env unset) */
	uint64_t ts = atrium_trace_now_us();
	uint64_t tid = atrium_trace_tid();
	pid_t pid = getpid();
	pthread_mutex_lock(&atrium_trace_mtx);
	if (id) {
		fprintf(atrium_trace_file,
		        "{\"name\":\"%s\",\"ph\":\"%c\",\"ts\":%llu,\"pid\":%d,\"tid\":%llu,\"args\":{\"id\":%llu}},\n",
		        name, ph, (unsigned long long)ts, (int)pid, (unsigned long long)tid,
		        (unsigned long long)id);
	} else {
		fprintf(atrium_trace_file,
		        "{\"name\":\"%s\",\"ph\":\"%c\",\"ts\":%llu,\"pid\":%d,\"tid\":%llu},\n",
		        name, ph, (unsigned long long)ts, (int)pid, (unsigned long long)tid);
	}
	pthread_mutex_unlock(&atrium_trace_mtx);
}

static inline void atrium_trace_emit(const char *name, char ph) {
	atrium_trace_emit_id(name, ph, 0);
}

#define ATRIUM_TRACE_BEGIN(name)        atrium_trace_emit((name), 'B')
#define ATRIUM_TRACE_END(name)          atrium_trace_emit((name), 'E')
#define ATRIUM_TRACE_INSTANT(name)      atrium_trace_emit((name), 'i')
#define ATRIUM_TRACE_BEGIN_ID(name,id)  atrium_trace_emit_id((name), 'B', (uint64_t)(id))
#define ATRIUM_TRACE_END_ID(name,id)    atrium_trace_emit_id((name), 'E', (uint64_t)(id))
#define ATRIUM_TRACE_INSTANT_ID(name,id) atrium_trace_emit_id((name), 'i', (uint64_t)(id))

/* RAII-ish scope macro for C: emits BEGIN at decl, END at scope exit
 * via __attribute__((cleanup)). Use ATRIUM_TRACE_SCOPE("label"); inside a block. */
static inline void atrium_trace_scope_end_(const char **lbl) {
	atrium_trace_emit(*lbl, 'E');
}
#define ATRIUM_TRACE_SCOPE(name) \
	const char *atrium_trace_scope_lbl_ __attribute__((cleanup(atrium_trace_scope_end_))) = (name); \
	atrium_trace_emit((name), 'B')

/* Place this in EXACTLY ONE translation unit per process to define globals. */
#define ATRIUM_TRACE_DEFINE_GLOBALS() \
	FILE              *atrium_trace_file = NULL; \
	int                atrium_trace_inited = 0; \
	int                atrium_trace_disabled = 0; \
	pthread_mutex_t    atrium_trace_mtx = PTHREAD_MUTEX_INITIALIZER; \
	static void atrium_trace_flush_(void) { \
		if (atrium_trace_file) { \
			fprintf(atrium_trace_file, "{\"name\":\"_end\",\"ph\":\"i\",\"ts\":%llu,\"pid\":%d,\"tid\":0}\n]\n", \
			        (unsigned long long)atrium_trace_now_us(), (int)getpid()); \
			fclose(atrium_trace_file); \
			atrium_trace_file = NULL; \
		} \
	} \
	void atrium_trace_init(void) { \
		const char *env = getenv("ATRIUM_TRACE_FILE"); \
		if (!env || !*env) { atrium_trace_disabled = 1; atrium_trace_inited = 1; return; } \
		char path[1024]; \
		snprintf(path, sizeof(path), "%s.%d", env, (int)getpid()); \
		atrium_trace_file = fopen(path, "w"); \
		if (!atrium_trace_file) { atrium_trace_disabled = 1; atrium_trace_inited = 1; return; } \
		setvbuf(atrium_trace_file, NULL, _IOLBF, 0); \
		fprintf(atrium_trace_file, "[\n"); \
		atrium_trace_inited = 1; \
		atexit(atrium_trace_flush_); \
	}

#endif /* ATRIUM_TRACE_KMOD */

#ifdef __cplusplus
}
#endif

#endif /* ATRIUM_TRACE_H */
