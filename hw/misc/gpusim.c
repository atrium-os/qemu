/*
 * gpusim — thin OUT-OF-PROCESS QEMU PCI device.
 *
 * Pure C: presents the modeled AMD config/BAR/MSI-X surface to the guest and
 * forwards BAR/MMIO accesses to the gpusim Rust model running as a SEPARATE
 * process, over a Unix socket. NO Rust is linked into QEMU — this is what makes
 * it work on macOS (linking the Rust runtime into QEMU hangs it in dyld at
 * startup; verified). Crash-isolated and clean. CLEAN-ROOM: forwarding only.
 *
 * Run order: start `gpusim-server <socket>` first, then QEMU with
 *   -device gpusim,socket=/tmp/gpusim.sock
 *
 * Wire format (LE): request 20B [op,region,write,len,offset:u64,data:u64];
 *                   response 16B [data:u64, irq_valid, irq_vector, pad].
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msix.h"
#include "hw/core/qdev-properties.h"
#include "qom/object.h"
#include "ui/console.h"
#include "qemu/timer.h"
#include "qemu/thread.h"

#include <sys/socket.h>
#include <sys/un.h>

#define TYPE_GPUSIM "gpusim"
OBJECT_DECLARE_SIMPLE_TYPE(GpusimState, GPUSIM)

/* BAR / region indices (match engine/src/vfio_user.rs). */
#define GPUSIM_BAR_VRAM 0
#define GPUSIM_BAR_DOORBELL 2
#define GPUSIM_BAR_REGS 5
#define GPUSIM_VENDOR_ID 0x1002
#define GPUSIM_DEVICE_ID 0x7550

#define GPUSIM_VRAM_BAR_SIZE (256ull * 1024 * 1024)
/* The doorbell BAR holds one page per queue: GFX at page 0, COMPUTE at page 1
 * (offset 0x1000), and a page per user-mode queue beyond that. It MUST be large
 * enough that a write to any queue's doorbell page stays in-BAR — a 0x1000
 * (single-page) BAR let the COMPUTE doorbell at 0x1000 overflow into the
 * adjacent MSI-X table, corrupting the vector-0 message address so every
 * interrupt after the first was misrouted. 4 MiB matches the model's advertised
 * BAR2 size (engine/src/device_adapter.rs) and covers 1024 doorbell pages. */
#define GPUSIM_DOORBELL_BAR_SIZE (4ull * 1024 * 1024)
/* Cover all register apertures incl. APER_SCHED (0x4_0000) and APER_PGATE
 * (0x5_0000); 512 KiB, power-of-two for the PCI BAR. */
#define GPUSIM_REGS_BAR_SIZE 0x80000ull
#define GPUSIM_MSIX_BAR 4
#define GPUSIM_NUM_MSIX 8

/* Display block: its aperture in the REGS BAR + the live-tier vblank tick. The
 * model (engine/src/display.rs) owns the deterministic timing; this host timer
 * just pulses VBLANK_TICK at ~60 Hz so the in-VM driver runs against a real-ish
 * refresh (a no-op before SET_MODE). */
#define GPUSIM_APER_DISP 0x30000ull        /* matches device::regs::APER_DISP */
#define GPUSIM_DISP_VBLANK_TICK 0x30ull    /* matches display::regs::VBLANK_TICK */
#define GPUSIM_VBLANK_PERIOD_NS 16666667ull /* ~60 Hz live refresh */

/* framed protocol (see server/src/lib.rs) */
#define C_REGION 0x01
#define C_DMA_REPLY 0x05
#define C_CONFIG_WRITE 0x07
#define C_SCANOUT 0x08
#define C_DELIVER_EOP 0x09   /* host -> model: deliver the oldest deferred EOP */
#define C_MES_RUN 0x0a       /* host -> model: MES timer fired, run ready queues */
#define S_DMA_READ 0x81
#define S_DMA_WRITE 0x82
#define S_FINAL 0x83
#define S_DEFER 0x84         /* model -> host: defer this EOP by delay_ns */
#define S_MES_PENDING 0x85   /* model -> host: a doorbell enqueued; arm the MES tick */

/* Decoupled-MES coalescing window: re-armed on each doorbell, so a burst of
 * submissions accumulates and the MES tick runs them once the burst goes idle. */
#define GPUSIM_MES_WINDOW_NS 50000000ull /* 50 ms idle → schedule (reliably coalesces a submit burst) */

struct GpusimState {
    PCIDevice parent_obj;
    MemoryRegion bar_vram;
    MemoryRegion bar_doorbell;
    MemoryRegion bar_regs;
    char *socket_path;
    int fd;
    QemuConsole *con;        /* display console — makes the modeled scanout visible */
    DisplaySurface *surface; /* current presented surface */
    int cur_w, cur_h;        /* its dims, to re-create on a mode change */
    QEMUTimer *vblank_timer; /* live-tier display refresh; pulses VBLANK_TICK */
    QemuMutex sock_lock;     /* serialize whole socket transactions: a vCPU MMIO
                              * C_REGION must not interleave with the main-loop
                              * gfx_update's C_SCANOUT (a 1 MiB+ reply) — they run
                              * on different threads and would desync the framing. */
    /* Shaping (gpu-device-model.md): EOP completions deferred by the modeled
     * render time. eop_due is a FIFO of fire-at timestamps (ns, QEMU_CLOCK_VIRTUAL);
     * render_busy_until chains them onto the serial engine tail; eop_timer fires
     * the head, delivering it via C_DELIVER_EOP. */
    QEMUTimer *eop_timer;
    GQueue *eop_due;             /* malloc'd uint64_t fire-at times, FIFO */
    uint64_t render_busy_until_ns;
    /* Decoupled MES: a doorbell enqueued work (S_MES_PENDING); the mes_timer,
     * re-armed on each doorbell, runs the ready queues once the burst goes idle. */
    QEMUTimer *mes_timer;
    bool mes_pending;
};

static void put_le64(uint8_t *p, uint64_t v)
{
    for (int i = 0; i < 8; i++) {
        p[i] = (v >> (8 * i)) & 0xff;
    }
}
static uint64_t get_le64(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) {
        v |= ((uint64_t)p[i]) << (8 * i);
    }
    return v;
}
static void put_le32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) {
        p[i] = (v >> (8 * i)) & 0xff;
    }
}
static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool io_all(int fd, void *buf, size_t n, bool writing)
{
    uint8_t *p = buf;
    size_t done = 0;
    while (done < n) {
        ssize_t r = writing ? write(fd, p + done, n - done)
                            : read(fd, p + done, n - done);
        if (r <= 0) {
            if (r < 0 && errno == EINTR) {
                continue;
            }
            return false;
        }
        done += r;
    }
    return true;
}

/* Shaping: queue a deferred EOP to fire `delay_ns` of modeled render time after
 * the serial engine reaches it. render_busy_until chains successive EOPs (the
 * engine is one FIFO), so fire times are monotonic; the timer tracks the head.
 * Called from the drain loop (under sock_lock). */
static void gpusim_eop_enqueue(GpusimState *s, uint64_t delay_ns)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t base = s->render_busy_until_ns > now ? s->render_busy_until_ns : now;
    uint64_t fire_at = base + delay_ns;
    s->render_busy_until_ns = fire_at;
    bool was_empty = g_queue_is_empty(s->eop_due);
    uint64_t *slot = g_new(uint64_t, 1);
    *slot = fire_at;
    g_queue_push_tail(s->eop_due, slot);
    if (was_empty) {
        timer_mod(s->eop_timer, fire_at);
    }
}

/* Service the model's DMA round-trips + shaping deferrals until the FINAL frame.
 * Returns read data; sets the irq/vec out-params from FINAL. Shared by C_REGION
 * and C_DELIVER_EOP (whichever request the caller already sent). */
static uint64_t gpusim_drain(GpusimState *s, bool *irq, uint8_t *vec)
{
    for (;;) {
        uint8_t tag;
        if (!io_all(s->fd, &tag, 1, false)) {
            return 0;
        }
        if (tag == S_DMA_READ) {
            uint8_t hdr[12];
            if (!io_all(s->fd, hdr, sizeof(hdr), false)) {
                return 0;
            }
            uint64_t gpa = get_le64(hdr);
            uint32_t l = get_le32(hdr + 8);
            uint8_t *buf = g_malloc(l);
            pci_dma_read(PCI_DEVICE(s), gpa, buf, l);
            uint8_t rep[6];
            rep[0] = C_DMA_REPLY;
            rep[1] = 0;
            put_le32(rep + 2, l);
            io_all(s->fd, rep, sizeof(rep), true);
            io_all(s->fd, buf, l, true);
            g_free(buf);
        } else if (tag == S_DMA_WRITE) {
            uint8_t hdr[12];
            if (!io_all(s->fd, hdr, sizeof(hdr), false)) {
                return 0;
            }
            uint64_t gpa = get_le64(hdr);
            uint32_t l = get_le32(hdr + 8);
            uint8_t *buf = g_malloc(l);
            if (!io_all(s->fd, buf, l, false)) {
                g_free(buf);
                return 0;
            }
            pci_dma_write(PCI_DEVICE(s), gpa, buf, l);
            g_free(buf);
            uint8_t rep[6] = { C_DMA_REPLY, 0, 0, 0, 0, 0 };
            io_all(s->fd, rep, sizeof(rep), true);
        } else if (tag == S_DEFER) {
            uint8_t d[8];
            if (!io_all(s->fd, d, sizeof(d), false)) {
                return 0;
            }
            gpusim_eop_enqueue(s, get_le64(d));
        } else if (tag == S_MES_PENDING) {
            s->mes_pending = true; /* do_write re-arms the MES tick after this */
        } else if (tag == S_FINAL) {
            uint8_t fin[10];
            if (!io_all(s->fd, fin, sizeof(fin), false)) {
                return 0;
            }
            if (irq && fin[8]) {
                *irq = true;
                if (vec) {
                    *vec = fin[9];
                }
            }
            return get_le64(fin);
        } else {
            return 0;
        }
    }
}

/* Send a REGION access, then service the model's requests until FINAL. Returns
 * read data; sets the irq/vec out-params if the model raised an interrupt. */
static uint64_t gpusim_region(GpusimState *s, uint8_t region, bool write_acc,
                              uint8_t len, uint64_t offset, uint64_t data,
                              bool *irq, uint8_t *vec)
{
    if (irq) {
        *irq = false;
    }
    if (s->fd < 0) {
        return 0;
    }
    uint8_t req[20];
    req[0] = C_REGION;
    req[1] = region;
    req[2] = write_acc ? 1 : 0;
    req[3] = len;
    put_le64(req + 4, offset);
    put_le64(req + 12, data);
    if (!io_all(s->fd, req, sizeof(req), true)) {
        return 0;
    }
    return gpusim_drain(s, irq, vec);
}

/* Forward a config-space dword write to the model (offset:u64 value:u32), then
 * consume the FINAL ack. Lets the model's referee see COMMAND enable bits. */
static void gpusim_config_forward(GpusimState *s, uint64_t offset, uint32_t value)
{
    if (s->fd < 0) {
        return;
    }
    uint8_t req[13];
    req[0] = C_CONFIG_WRITE;
    put_le64(req + 1, offset);
    put_le32(req + 9, value);
    if (!io_all(s->fd, req, sizeof(req), true)) {
        return;
    }
    uint8_t tag;
    if (io_all(s->fd, &tag, 1, false) && tag == S_FINAL) {
        uint8_t fin[10];
        io_all(s->fd, fin, sizeof(fin), false); /* ack */
    }
}

static uint64_t gpusim_do_read(void *opaque, hwaddr addr, unsigned size, uint8_t region)
{
    GpusimState *s = opaque;
    bool irq = false;
    uint8_t vec = 0;
    uint64_t v;
    qemu_mutex_lock(&s->sock_lock);
    v = gpusim_region(s, region, false, size, addr, 0, &irq, &vec);
    qemu_mutex_unlock(&s->sock_lock);
    if (irq) {
        msix_notify(PCI_DEVICE(s), vec);
    }
    return v;
}

static void gpusim_do_write(void *opaque, hwaddr addr, uint64_t val, unsigned size, uint8_t region)
{
    GpusimState *s = opaque;
    bool irq = false;
    uint8_t vec = 0;
    qemu_mutex_lock(&s->sock_lock);
    gpusim_region(s, region, true, size, addr, val, &irq, &vec);
    bool mes = s->mes_pending;
    s->mes_pending = false;
    qemu_mutex_unlock(&s->sock_lock);
    if (irq) {
        msix_notify(PCI_DEVICE(s), vec);
    }
    /* Decoupled MES: a doorbell enqueued work — (re)arm the MES tick. Re-arming on
     * each doorbell coalesces a submission burst; it fires once the burst is idle. */
    if (mes) {
        timer_mod(s->mes_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GPUSIM_MES_WINDOW_NS);
    }
}

static uint64_t vram_read(void *o, hwaddr a, unsigned s) { return gpusim_do_read(o, a, s, GPUSIM_BAR_VRAM); }
static void vram_write(void *o, hwaddr a, uint64_t v, unsigned s) { gpusim_do_write(o, a, v, s, GPUSIM_BAR_VRAM); }
static uint64_t doorbell_read(void *o, hwaddr a, unsigned s) { return gpusim_do_read(o, a, s, GPUSIM_BAR_DOORBELL); }
static void doorbell_write(void *o, hwaddr a, uint64_t v, unsigned s) { gpusim_do_write(o, a, v, s, GPUSIM_BAR_DOORBELL); }
static uint64_t regs_read(void *o, hwaddr a, unsigned s) { return gpusim_do_read(o, a, s, GPUSIM_BAR_REGS); }
static void regs_write(void *o, hwaddr a, uint64_t v, unsigned s) { gpusim_do_write(o, a, v, s, GPUSIM_BAR_REGS); }

static const MemoryRegionOps vram_ops = {
    .read = vram_read, .write = vram_write, .endianness = DEVICE_LITTLE_ENDIAN,
};
static const MemoryRegionOps doorbell_ops = {
    .read = doorbell_read, .write = doorbell_write, .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 8 },
};
static const MemoryRegionOps regs_ops = {
    .read = regs_read, .write = regs_write, .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* Display refresh hook: pull the current scanout FB from the model in one bulk
 * C_SCANOUT transfer and blit it to the console surface. THIS is what makes the
 * modeled display visible — without it the gpusim display path is readback-only.
 * The C_SCANOUT reply is large (a full framebuffer); the whole socket transaction
 * is taken under sock_lock so a concurrent vCPU MMIO C_REGION can't interleave
 * with it on the wire (gfx_update runs on the main loop, MMIO on the vCPU thread —
 * they are NOT mutually exclusive under HVF, and an interleave desyncs the framing,
 * silently dropping register writes). */
static void gpusim_gfx_update(void *opaque)
{
    GpusimState *s = opaque;
    if (s->fd < 0) {
        return;
    }

    uint32_t w = 0, h = 0, len = 0;
    uint8_t *fb = NULL;

    /* Socket transaction (request + full reply) under the lock. */
    qemu_mutex_lock(&s->sock_lock);
    uint8_t op = C_SCANOUT;
    if (io_all(s->fd, &op, 1, true)) {
        uint8_t present = 0;
        if (io_all(s->fd, &present, 1, false) && present) {
            uint8_t hdr[12];
            if (io_all(s->fd, hdr, sizeof(hdr), false)) {
                w = get_le32(hdr);
                h = get_le32(hdr + 4);
                len = get_le32(hdr + 8);
                if (w != 0 && h != 0 && len != 0) {
                    fb = g_malloc(len);
                    if (!io_all(s->fd, fb, len, false)) {
                        g_free(fb);
                        fb = NULL;
                    }
                }
            }
        }
    }
    qemu_mutex_unlock(&s->sock_lock);

    if (fb == NULL) {
        return; /* no FB programmed / transfer failed — leave the surface as-is */
    }

    /* (Re)create the surface when the mode changes. */
    if (!s->surface || s->cur_w != (int)w || s->cur_h != (int)h) {
        s->surface = qemu_create_displaysurface(w, h);
        dpy_gfx_replace_surface(s->con, s->surface);
        s->cur_w = (int)w;
        s->cur_h = (int)h;
    }

    /* Blit. The QEMU surface is x8r8g8b8 (B,G,R,X in LE memory); the scanout BO
     * is assumed to already be in that byte order (the efifb BGRA convention).
     * If colours come out swapped at first light, swizzle R<->B here. */
    uint8_t *dst = surface_data(s->surface);
    size_t fb_bytes = (size_t)w * h * 4;
    size_t n = len < fb_bytes ? len : fb_bytes;
    memcpy(dst, fb, n);
    g_free(fb);

    dpy_gfx_update_full(s->con);
}

static const GraphicHwOps gpusim_gfx_ops = {
    .gfx_update = gpusim_gfx_update,
};

/* Live-tier display refresh: pulse the model's VBLANK_TICK at ~60 Hz, then
 * re-arm. The model advances one frame (latching a pending flip); before
 * SET_MODE it is a no-op. The deterministic timing lives in the engine tests —
 * this only makes the in-VM driver run against a real-ish refresh. */
static void gpusim_vblank_tick(void *opaque)
{
    GpusimState *s = opaque;
    bool irq = false;
    uint8_t vec = 0;

    if (s->fd < 0) {
        return;
    }
    qemu_mutex_lock(&s->sock_lock);
    gpusim_region(s, GPUSIM_BAR_REGS, true, 4,
                  GPUSIM_APER_DISP + GPUSIM_DISP_VBLANK_TICK, 1, &irq, &vec);
    qemu_mutex_unlock(&s->sock_lock);
    if (irq) {
        msix_notify(PCI_DEVICE(s), vec);
    }
    /* Drive the console refresh each vblank: a static scanout (a compositor that
     * drew once and is now idle) produces no display updates, so -display cocoa
     * wouldn't repaint it until forced (e.g. a console switch). graphic_hw_update
     * re-runs gpusim_gfx_update (re-blit + full-dirty) so the latched frame stays
     * on screen at ~60 Hz with no nudge. It takes sock_lock internally, so it must
     * be OUTSIDE the lock above. */
    graphic_hw_update(s->con);
    timer_mod(s->vblank_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GPUSIM_VBLANK_PERIOD_NS);
}

/* Ask the model to deliver the oldest deferred EOP — it writes the IH cookie via
 * DMA + raises MSI-X (reported in FINAL). Caller holds sock_lock. */
static uint64_t gpusim_deliver_eop(GpusimState *s, bool *irq, uint8_t *vec)
{
    if (irq) {
        *irq = false;
    }
    if (s->fd < 0) {
        return 0;
    }
    uint8_t req = C_DELIVER_EOP;
    if (!io_all(s->fd, &req, 1, true)) {
        return 0;
    }
    return gpusim_drain(s, irq, vec);
}

/* A deferral timer expired: deliver the head EOP + msix_notify, then re-arm for
 * the next queued EOP (FIFO; fire times are monotonic, so the head is earliest). */
static void gpusim_eop_fire(void *opaque)
{
    GpusimState *s = opaque;
    bool irq = false;
    uint8_t vec = 0;

    if (s->fd < 0) {
        return;
    }
    qemu_mutex_lock(&s->sock_lock);
    gpusim_deliver_eop(s, &irq, &vec);
    qemu_mutex_unlock(&s->sock_lock);
    if (irq) {
        msix_notify(PCI_DEVICE(s), vec);
    }
    uint64_t *done = g_queue_pop_head(s->eop_due);
    g_free(done);
    if (!g_queue_is_empty(s->eop_due)) {
        uint64_t *next = g_queue_peek_head(s->eop_due);
        timer_mod(s->eop_timer, *next);
    }
}

/* Ask the model to run its ready queues (decoupled MES tick). The drain handles
 * the resulting fence DMAs + shaping deferrals (S_DEFER) + FINAL irq. Caller holds
 * sock_lock. */
static uint64_t gpusim_mes_run(GpusimState *s, bool *irq, uint8_t *vec)
{
    if (irq) {
        *irq = false;
    }
    if (s->fd < 0) {
        return 0;
    }
    uint8_t req = C_MES_RUN;
    if (!io_all(s->fd, &req, 1, true)) {
        return 0;
    }
    return gpusim_drain(s, irq, vec);
}

/* The MES timer fired: run the accumulated submissions (the model schedules them
 * earliest-deadline-first). Any immediate EOP IRQ is delivered now; shaping EOPs
 * were queued on the eop_timer by the drain's S_DEFER handling. */
static void gpusim_mes_fire(void *opaque)
{
    GpusimState *s = opaque;
    bool irq = false;
    uint8_t vec = 0;

    if (s->fd < 0) {
        return;
    }
    qemu_mutex_lock(&s->sock_lock);
    gpusim_mes_run(s, &irq, &vec);
    qemu_mutex_unlock(&s->sock_lock);
    if (irq) {
        msix_notify(PCI_DEVICE(s), vec);
    }
}

static void gpusim_realize(PCIDevice *pdev, Error **errp)
{
    GpusimState *s = GPUSIM(pdev);

    qemu_mutex_init(&s->sock_lock);
    s->eop_due = g_queue_new();
    s->render_busy_until_ns = 0;
    s->mes_pending = false;
    s->fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (s->fd < 0) {
        error_setg_errno(errp, errno, "gpusim: socket()");
        return;
    }
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    g_strlcpy(addr.sun_path, s->socket_path, sizeof(addr.sun_path));
    if (connect(s->fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        error_setg_errno(errp, errno, "gpusim: connect %s (is gpusim-server running?)",
                         s->socket_path);
        close(s->fd);
        s->fd = -1;
        return;
    }

    memory_region_init_io(&s->bar_vram, OBJECT(s), &vram_ops, s, "gpusim-vram",
                          GPUSIM_VRAM_BAR_SIZE);
    pci_register_bar(pdev, 0,
                     PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH,
                     &s->bar_vram);
    memory_region_init_io(&s->bar_doorbell, OBJECT(s), &doorbell_ops, s,
                          "gpusim-doorbell", GPUSIM_DOORBELL_BAR_SIZE);
    pci_register_bar(pdev, 2, PCI_BASE_ADDRESS_MEM_TYPE_64, &s->bar_doorbell);
    memory_region_init_io(&s->bar_regs, OBJECT(s), &regs_ops, s, "gpusim-regs",
                          GPUSIM_REGS_BAR_SIZE);
    pci_register_bar(pdev, 5, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar_regs);

    if (msix_init_exclusive_bar(pdev, GPUSIM_NUM_MSIX, GPUSIM_MSIX_BAR, errp) < 0) {
        close(s->fd);
        s->fd = -1;
        return;
    }
    for (int i = 0; i < GPUSIM_NUM_MSIX; i++) {
        msix_vector_use(pdev, i);
    }

    /* Register a display console so the modeled scanout is actually presented
     * (pulled from VRAM each refresh by gpusim_gfx_update). With -display cocoa
     * this is the visible window; headless runs simply never call gfx_update. */
    s->con = graphic_console_init(DEVICE(pdev), 0, &gpusim_gfx_ops, s);

    /* Arm the live-tier vblank: pulse the model's VBLANK_TICK at ~60 Hz so
     * page-flips latch + vblank counters advance in-VM (the engine tier keeps
     * the deterministic timing). */
    s->vblank_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gpusim_vblank_tick, s);
    timer_mod(s->vblank_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GPUSIM_VBLANK_PERIOD_NS);

    /* Shaping deferral timer (armed on demand by gpusim_eop_enqueue). */
    s->eop_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gpusim_eop_fire, s);
    /* Decoupled-MES tick (armed on demand by a doorbell in MES mode). */
    s->mes_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gpusim_mes_fire, s);
}

static void gpusim_exit(PCIDevice *pdev)
{
    GpusimState *s = GPUSIM(pdev);
    if (s->vblank_timer) {
        timer_free(s->vblank_timer);
        s->vblank_timer = NULL;
    }
    msix_uninit_exclusive_bar(pdev);
    if (s->fd >= 0) {
        close(s->fd);
        s->fd = -1;
    }
}

/* QEMU stays the config-space authority (COMMAND, BARs, MSI-X); we additionally
 * mirror the COMMAND register to the model so its referee can enforce the PCI
 * bring-up invariants (Bus Master / Memory Space enable). */
static void gpusim_write_config(PCIDevice *pdev, uint32_t addr, uint32_t val, int len)
{
    GpusimState *s = GPUSIM(pdev);
    pci_default_write_config(pdev, addr, val, len);
    if (addr <= PCI_COMMAND && addr + len > PCI_COMMAND) {
        gpusim_config_forward(s, PCI_COMMAND, pci_get_word(pdev->config + PCI_COMMAND));
    }
}

static const Property gpusim_properties[] = {
    DEFINE_PROP_STRING("socket", GpusimState, socket_path),
};

static void gpusim_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = gpusim_realize;
    k->exit = gpusim_exit;
    k->config_write = gpusim_write_config;
    k->vendor_id = GPUSIM_VENDOR_ID;
    k->device_id = GPUSIM_DEVICE_ID;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_DISPLAY_VGA;

    dc->desc = "gpusim RDNA functional model (out-of-process, clean-room)";
    device_class_set_props(dc, gpusim_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo gpusim_types[] = {
    {
        .name = TYPE_GPUSIM,
        .parent = TYPE_PCI_DEVICE,
        .instance_size = sizeof(GpusimState),
        .class_init = gpusim_class_init,
        .interfaces = (const InterfaceInfo[]){
            { INTERFACE_CONVENTIONAL_PCI_DEVICE },
            { },
        },
    }
};

DEFINE_TYPES(gpusim_types)
