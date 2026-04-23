/*
 * TMS380 Token Ring Adapter - QEMU PCI Device Model
 *
 * Emulates a Compaq 4/16 Token Ring PCI adapter using the TMS380
 * SIF register interface. The Linux tmspci + tms380tr drivers
 * communicate through these registers via DIO (Direct I/O).
 *
 * Copyright (c) 2026 Dan Berkowitz
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "tms380.h"
#include <dlfcn.h>

/* --- DIO (Direct I/O) helpers --- */

/* Read a 16-bit word from adapter SRAM at the current DIO address */
static uint16_t dio_read16(TMS380PCIState *s)
{
    uint32_t addr = ((uint32_t)s->dio_page << 16) | s->dio_addr;
    if (addr + 1 < TMS380_SRAM_SIZE) {
        return (s->sram[addr] << 8) | s->sram[addr + 1];
    }
    return 0xFFFF;
}

/* Read 16-bit from SRAM and auto-increment the DIO address by 2 */
static uint16_t dio_read16_inc(TMS380PCIState *s)
{
    uint16_t val = dio_read16(s);
    s->dio_addr += 2;
    return val;
}

/* Write a 16-bit word to adapter SRAM at the current DIO address */
static void dio_write16(TMS380PCIState *s, uint16_t val)
{
    uint32_t addr = ((uint32_t)s->dio_page << 16) | s->dio_addr;
    if (addr + 1 < TMS380_SRAM_SIZE) {
        s->sram[addr] = (val >> 8) & 0xFF;
        s->sram[addr + 1] = val & 0xFF;
    }
}

/* Write 16-bit to SRAM and auto-increment */
static void dio_write16_inc(TMS380PCIState *s, uint16_t val)
{
    dio_write16(s, val);
    s->dio_addr += 2;
}

/* --- Backend loading --- */

static bool tms380_load_backend(TMS380PCIState *s)
{
    if (!s->backend_lib_path || !s->backend_lib_path[0]) {
        return false;
    }

    s->backend_lib = dlopen(s->backend_lib_path, RTLD_NOW);
    if (!s->backend_lib) {
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: dlopen(%s): %s\n",
                      s->backend_lib_path, dlerror());
        return false;
    }

    s->fn_create = dlsym(s->backend_lib, "tr_backend_create");
    s->fn_insert = dlsym(s->backend_lib, "tr_backend_insert");
    s->fn_send = dlsym(s->backend_lib, "tr_backend_send");
    s->fn_get_recv_fd = dlsym(s->backend_lib, "tr_backend_get_recv_fd");
    s->fn_recv = dlsym(s->backend_lib, "tr_backend_recv");
    s->fn_destroy = dlsym(s->backend_lib, "tr_backend_destroy");

    if (!s->fn_create || !s->fn_insert || !s->fn_send ||
        !s->fn_get_recv_fd || !s->fn_recv || !s->fn_destroy) {
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: missing symbols in %s\n",
                      s->backend_lib_path);
        dlclose(s->backend_lib);
        s->backend_lib = NULL;
        return false;
    }

    qemu_log_mask(LOG_GUEST_ERROR, "tms380: backend loaded from %s\n",
                  s->backend_lib_path);
    return true;
}

/* --- Reset and initialization --- */

static void tms380_reset_done(void *opaque)
{
    TMS380PCIState *s = opaque;

    /* Transition from RESET → BUD.
     * The driver expects to see STS_INITIALIZE in SIFSTS after BUD completes. */
    s->dev_state = TMS_STATE_BUD;

    /* BUD complete — set STS_INITIALIZE to tell driver we're ready for IPB */
    s->sifsts = STS_INITIALIZE;

    /* Clear reset/halt bits in ACL */
    s->sifacl &= ~(ACL_ARESET | ACL_CPHALT);

    qemu_log_mask(LOG_GUEST_ERROR, "tms380: BUD complete, awaiting init\n");
}

static void tms380_do_reset(TMS380PCIState *s)
{
    s->dev_state = TMS_STATE_RESET;
    s->sifsts = STS_TEST;  /* Self-test in progress */
    s->sifcmd = 0;
    s->dio_addr = 0;
    s->dio_page = 0;

    /* Place MAC address at SRAM address 0x0000 (big-endian, one byte per word).
     * The tmspci driver reads it as: SIFADX=0, SIFADR=0, then 6x SIFINC >> 8 */
    for (int i = 0; i < 6; i++) {
        s->sram[i * 2] = s->mac[i];      /* High byte of each word */
        s->sram[i * 2 + 1] = 0x00;       /* Low byte (ignored by driver) */
    }

    /* Simulate BUD delay — 50ms virtual time */
    timer_mod(s->reset_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 50 * SCALE_MS);
}

/* Handle the initialization command (after driver writes IPB and sends CMD_EXECUTE) */
static void tms380_handle_init(TMS380PCIState *s)
{
    /* The driver wrote an IPB (Initialization Parameter Block) to SRAM.
     * We don't need to parse most of it — just transition to READY.
     * Clear all status bits to signal init complete. */
    s->sifsts = 0;
    s->dev_state = TMS_STATE_READY;
    qemu_log_mask(LOG_GUEST_ERROR, "tms380: initialization complete, ready\n");
}

/* --- SRB command handling --- */

static void tms380_raise_irq(TMS380PCIState *s, uint16_t irq_type)
{
    s->sifsts = (s->sifsts & ~STS_IRQ_MASK) | (irq_type & STS_IRQ_MASK);
    pci_irq_assert(&s->parent_obj);
}

static void tms380_handle_srb(TMS380PCIState *s, uint16_t srb_addr)
{
    uint8_t cmd = s->sram[srb_addr];

    qemu_log_mask(LOG_GUEST_ERROR, "tms380: SRB command 0x%02x at 0x%04x\n",
                  cmd, srb_addr);

    switch (cmd) {
    case SRB_CMD_OPEN:
        if (s->fn_create && s->mau_path && s->mau_path[0]) {
            s->backend = s->fn_create(s->mau_path, s->mac);
            if (s->backend && s->fn_insert(s->backend) == 0) {
                s->dev_state = TMS_STATE_OPEN;
                s->sram[srb_addr + 2] = 0x00; /* success */
                qemu_log_mask(LOG_GUEST_ERROR,
                              "tms380: OPEN success, on ring\n");
            } else {
                if (s->backend) {
                    s->fn_destroy(s->backend);
                    s->backend = NULL;
                }
                s->sram[srb_addr + 2] = 0x07;
                qemu_log_mask(LOG_GUEST_ERROR, "tms380: OPEN failed\n");
            }
        } else {
            /* No backend — stub success for testing */
            s->dev_state = TMS_STATE_OPEN;
            s->sram[srb_addr + 2] = 0x00;
            qemu_log_mask(LOG_GUEST_ERROR, "tms380: OPEN (stub)\n");
        }
        break;

    case SRB_CMD_CLOSE:
        if (s->backend && s->fn_destroy) {
            s->fn_destroy(s->backend);
            s->backend = NULL;
        }
        s->dev_state = TMS_STATE_READY;
        s->sram[srb_addr + 2] = 0x00;
        break;

    case SRB_CMD_SET_FUNCT_ADDR:
    case SRB_CMD_SET_GROUP_ADDR:
    case SRB_CMD_READ_ERROR_LOG:
        s->sram[srb_addr + 2] = 0x00; /* success */
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "tms380: unknown SRB 0x%02x\n", cmd);
        s->sram[srb_addr + 2] = 0x04;
        break;
    }

    /* Signal command completion */
    tms380_raise_irq(s, STS_IRQ_COMMAND_STATUS);
}

/* --- SIF register I/O --- */

static uint64_t tms380_io_read(void *opaque, hwaddr addr, unsigned size)
{
    TMS380PCIState *s = opaque;
    uint16_t val = 0;

    switch (addr) {
    case SIF_DAT:   /* 0x00 — read SRAM at current DIO address */
        val = dio_read16(s);
        break;

    case SIF_INC:   /* 0x02 — read SRAM + auto-increment */
        val = dio_read16_inc(s);
        break;

    case SIF_ADR:   /* 0x04 — current DIO address */
        val = s->dio_addr;
        break;

    case SIF_CMD:   /* 0x06 — read returns status register */
        val = s->sifsts;
        break;

    case SIF_ACL:   /* 0x08 — adapter control */
        val = s->sifacl;
        break;

    case SIF_ADD:   /* 0x0a — DIO address (alternate) */
        val = s->dio_addr;
        break;

    case SIF_ADX:   /* 0x0c — DIO extended/page address */
        val = s->dio_page;
        break;

    case SIF_DMALEN: /* 0x0e — DMA length (not used) */
        val = 0;
        break;

    default:
        break;
    }

    return val;
}

static void tms380_io_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    TMS380PCIState *s = opaque;
    uint16_t v = (uint16_t)val;

    switch (addr) {
    case SIF_DAT:   /* 0x00 — write SRAM at current DIO address */
        dio_write16(s, v);
        break;

    case SIF_INC:   /* 0x02 — write SRAM + auto-increment */
        dio_write16_inc(s, v);
        break;

    case SIF_ADR:   /* 0x04 — set DIO address */
        s->dio_addr = v;
        break;

    case SIF_CMD:   /* 0x06 — command register */
        s->sifcmd = v;

        /* Handle command bits */
        if (v & CMD_EXECUTE) {
            if (s->dev_state == TMS_STATE_BUD) {
                /* Driver sent EXEC_SOFT_RESET (0xFF00) or CMD_EXECUTE during init.
                 * If we're in BUD state with STS_INITIALIZE set, this is the
                 * init command after IPB write. */
                if (s->sifsts & STS_INITIALIZE) {
                    /* Still in BUD waiting for IPB — this is the init execute */
                    tms380_handle_init(s);
                }
            } else if (s->dev_state == TMS_STATE_READY ||
                       s->dev_state == TMS_STATE_OPEN) {
                /* SCB request — the driver wrote a command to the SCB area.
                 * Read the SRB address from the SCB and process it. */
                if (v & CMD_SCB_REQUEST) {
                    /* The SCB contains the SRB address at a known offset.
                     * For simplicity, use the DIO address as the SRB location. */
                    uint16_t srb_addr = s->dio_addr;
                    tms380_handle_srb(s, srb_addr);
                }
            }
        }

        /* Clear system interrupt when requested */
        if (v & 0x00FF) {
            pci_irq_deassert(&s->parent_obj);
            s->sifsts &= ~STS_IRQ_MASK;
        }
        break;

    case SIF_ACL:   /* 0x08 — adapter control */
        s->sifacl = v;
        if (v & ACL_ARESET) {
            tms380_do_reset(s);
        }
        break;

    case SIF_ADD:   /* 0x0a — DIO address (alternate set) */
        s->dio_addr = v;
        break;

    case SIF_ADX:   /* 0x0c — DIO page/extended address */
        s->dio_page = v;
        break;

    case SIF_DMALEN: /* 0x0e — DMA length (ignored) */
        break;

    default:
        break;
    }
}

static const MemoryRegionOps tms380_io_ops = {
    .read = tms380_io_read,
    .write = tms380_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 2,
    },
};

/* --- PCI device lifecycle --- */

static void tms380_pci_realize(PCIDevice *pci_dev, Error **errp)
{
    TMS380PCIState *s = TMS380_PCI(pci_dev);

    /* Set PCI interrupt pin A */
    pci_dev->config[PCI_INTERRUPT_PIN] = 1;

    /* Allocate adapter SRAM */
    s->sram = g_malloc0(TMS380_SRAM_SIZE);

    /* Register I/O BAR */
    memory_region_init_io(&s->io_bar, OBJECT(pci_dev), &tms380_io_ops, s,
                          "tms380-sif", TMS380_IO_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->io_bar);

    /* Create reset timer */
    s->reset_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tms380_reset_done, s);

    /* Load backend if configured */
    tms380_load_backend(s);

    /* Start in reset */
    tms380_do_reset(s);

    qemu_log_mask(LOG_GUEST_ERROR,
                  "tms380: PCI adapter MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                  s->mac[0], s->mac[1], s->mac[2],
                  s->mac[3], s->mac[4], s->mac[5]);
}

static void tms380_pci_exit(PCIDevice *pci_dev)
{
    TMS380PCIState *s = TMS380_PCI(pci_dev);

    timer_free(s->reset_timer);

    if (s->backend && s->fn_destroy) {
        s->fn_destroy(s->backend);
        s->backend = NULL;
    }
    if (s->backend_lib) {
        dlclose(s->backend_lib);
        s->backend_lib = NULL;
    }

    g_free(s->sram);
}

static const Property tms380_pci_properties[] = {
    DEFINE_PROP_STRING("mau_path", TMS380PCIState, mau_path),
    DEFINE_PROP_STRING("backend_lib", TMS380PCIState, backend_lib_path),
    DEFINE_PROP_UINT8("mac0", TMS380PCIState, mac[0], 0x00),
    DEFINE_PROP_UINT8("mac1", TMS380PCIState, mac[1], 0x00),
    DEFINE_PROP_UINT8("mac2", TMS380PCIState, mac[2], 0xF8),
    DEFINE_PROP_UINT8("mac3", TMS380PCIState, mac[3], 0x00),
    DEFINE_PROP_UINT8("mac4", TMS380PCIState, mac[4], 0x00),
    DEFINE_PROP_UINT8("mac5", TMS380PCIState, mac[5], 0x01),
};

static void tms380_pci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = tms380_pci_realize;
    k->exit = tms380_pci_exit;
    k->vendor_id = TMS380_PCI_VENDOR_ID;
    k->device_id = TMS380_PCI_DEVICE_ID;
    k->class_id = PCI_CLASS_NETWORK_OTHER;
    k->revision = 0x01;
    device_class_set_props(dc, tms380_pci_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "TMS380 Token Ring 16/4 PCI Adapter (Compaq)";
}

static const TypeInfo tms380_pci_type_info = {
    .name          = TYPE_TMS380_PCI,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(TMS380PCIState),
    .class_init    = tms380_pci_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void tms380_pci_register_types(void)
{
    type_register_static(&tms380_pci_type_info);
}

type_init(tms380_pci_register_types)
