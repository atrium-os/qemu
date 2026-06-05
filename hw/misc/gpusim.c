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
#define GPUSIM_DOORBELL_BAR_SIZE 0x1000ull
#define GPUSIM_REGS_BAR_SIZE 0x40000ull
#define GPUSIM_MSIX_BAR 4
#define GPUSIM_NUM_MSIX 8

/* framed protocol (see server/src/lib.rs) */
#define C_REGION 0x01
#define C_DMA_REPLY 0x05
#define C_CONFIG_WRITE 0x07
#define S_DMA_READ 0x81
#define S_DMA_WRITE 0x82
#define S_FINAL 0x83

struct GpusimState {
    PCIDevice parent_obj;
    MemoryRegion bar_vram;
    MemoryRegion bar_doorbell;
    MemoryRegion bar_regs;
    char *socket_path;
    int fd;
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

/* Send a REGION access, then service the model's DMA requests (translating them
 * to guest-memory access via pci_dma) until the FINAL frame. Returns read data;
 * sets the irq/vec out-params if the model raised an interrupt. */
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
    uint64_t v = gpusim_region(s, region, false, size, addr, 0, &irq, &vec);
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
    gpusim_region(s, region, true, size, addr, val, &irq, &vec);
    if (irq) {
        msix_notify(PCI_DEVICE(s), vec);
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

static void gpusim_realize(PCIDevice *pdev, Error **errp)
{
    GpusimState *s = GPUSIM(pdev);

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
}

static void gpusim_exit(PCIDevice *pdev)
{
    GpusimState *s = GPUSIM(pdev);
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
