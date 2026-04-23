/*
 * TMS380 Token Ring Adapter - QEMU PCI Device Model
 *
 * Emulates a Compaq 4/16 Token Ring PCI adapter using the TMS380
 * SIF register interface. The Linux tmspci + tms380tr drivers
 * communicate through these registers.
 *
 * The adapter backend connects to vmau via libtr_backend.so (Rust FFI).
 *
 * Copyright (c) 2026 Dan Berkowitz
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "tms380.h"
#include <dlfcn.h>

/* --- Backend loading --- */

static bool tms380_load_backend(TMS380PCIState *s)
{
    if (!s->backend_lib_path || !s->backend_lib_path[0]) {
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: no backend_lib specified\n");
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

/* --- Initialization data sequence --- */

static uint16_t tms380_init_data(TMS380PCIState *s, int index)
{
    switch (index) {
    case 0: return 0x0000;  /* Bring-up code: success */
    case 1: return (s->mac[0] << 8) | s->mac[1];
    case 2: return (s->mac[2] << 8) | s->mac[3];
    case 3: return (s->mac[4] << 8) | s->mac[5];
    case 4: return 0x0040;  /* Adapter RAM: 64KB */
    case 5: return 0x0010;  /* Ring speed: 16 Mbps */
    default: return 0xFFFF;
    }
}

/* --- Reset handling --- */

static void tms380_reset_done(void *opaque)
{
    TMS380PCIState *s = opaque;
    s->dev_state = TMS_STATE_READY;
    s->sifacl &= ~SIFACL_INIT;
    s->init_read_index = 0;
    qemu_log_mask(LOG_GUEST_ERROR, "tms380: reset complete, adapter ready\n");
}

static void tms380_do_reset(TMS380PCIState *s)
{
    s->dev_state = TMS_STATE_RESET;
    s->sifacl |= SIFACL_INIT;
    s->sifdat = 0;
    s->sifcmd = 0;
    s->sifadr = 0;
    s->init_read_index = 0;
    memset(s->sram, 0, TMS380_SRAM_SIZE);

    if (s->backend && s->fn_destroy) {
        s->fn_destroy(s->backend);
        s->backend = NULL;
    }

    timer_mod(s->reset_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10 * SCALE_MS);
}

/* --- SRB command handling --- */

static void tms380_raise_interrupt(TMS380PCIState *s)
{
    s->sifcmd |= SIFCMD_ADAP_INT;
    pci_irq_assert(&s->parent_obj);
}

static void tms380_handle_srb(TMS380PCIState *s)
{
    uint8_t cmd = s->sram[SRB_CMD_OFFSET];

    qemu_log_mask(LOG_GUEST_ERROR, "tms380: SRB command 0x%02x\n", cmd);

    switch (cmd) {
    case SRB_CMD_OPEN:
        if (s->fn_create && s->mau_path && s->mau_path[0]) {
            s->backend = s->fn_create(s->mau_path, s->mac);
            if (s->backend) {
                int rc = s->fn_insert(s->backend);
                if (rc == 0) {
                    s->dev_state = TMS_STATE_OPEN;
                    s->sram[SRB_RETCODE_OFFSET] = SRB_SUCCESS;
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "tms380: OPEN success, inserted into ring\n");
                } else {
                    s->fn_destroy(s->backend);
                    s->backend = NULL;
                    s->sram[SRB_RETCODE_OFFSET] = 0x07; /* open error */
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "tms380: OPEN failed: insert returned %d\n", rc);
                }
            } else {
                s->sram[SRB_RETCODE_OFFSET] = 0x07;
                qemu_log_mask(LOG_GUEST_ERROR,
                              "tms380: OPEN failed: create returned NULL\n");
            }
        } else {
            /* No backend — still return success for probe testing */
            s->dev_state = TMS_STATE_OPEN;
            s->sram[SRB_RETCODE_OFFSET] = SRB_SUCCESS;
            qemu_log_mask(LOG_GUEST_ERROR,
                          "tms380: OPEN (no backend, stub success)\n");
        }
        break;

    case SRB_CMD_CLOSE:
        if (s->backend && s->fn_destroy) {
            s->fn_destroy(s->backend);
            s->backend = NULL;
        }
        s->dev_state = TMS_STATE_READY;
        s->sram[SRB_RETCODE_OFFSET] = SRB_SUCCESS;
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: CLOSE\n");
        break;

    case SRB_CMD_SET_FUNCT_ADDR:
        s->sram[SRB_RETCODE_OFFSET] = SRB_SUCCESS;
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: SET_FUNCT_ADDR (stub)\n");
        break;

    case SRB_CMD_SET_GROUP_ADDR:
        s->sram[SRB_RETCODE_OFFSET] = SRB_SUCCESS;
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: SET_GROUP_ADDR (stub)\n");
        break;

    case SRB_CMD_READ_ERROR_LOG:
        /* Zero out the error counters area */
        memset(&s->sram[4], 0, 14);
        s->sram[SRB_RETCODE_OFFSET] = SRB_SUCCESS;
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: READ_ERROR_LOG (stub)\n");
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "tms380: unknown SRB command 0x%02x\n", cmd);
        s->sram[SRB_RETCODE_OFFSET] = 0x04; /* invalid command */
        break;
    }

    /* Raise interrupt to signal SRB completion */
    tms380_raise_interrupt(s);
}

/* --- SIF register I/O --- */

static uint64_t tms380_io_read(void *opaque, hwaddr addr, unsigned size)
{
    TMS380PCIState *s = opaque;
    uint16_t val = 0;

    switch (addr) {
    case SIF_DAT:
        if (s->dev_state == TMS_STATE_RESET) {
            val = 0;
        } else if (s->init_read_index < 6) {
            /* During init, return adapter info sequence */
            val = tms380_init_data(s, s->init_read_index);
            s->init_read_index++;
        } else {
            /* Normal operation: read from SRAM at current address */
            uint16_t addr16 = s->sifadr & 0xFF;
            if (addr16 + 1 < TMS380_SRAM_SIZE) {
                val = (s->sram[addr16] << 8) | s->sram[addr16 + 1];
                s->sifadr += 2;
            }
        }
        break;
    case SIF_CMD:
        val = s->sifcmd;
        break;
    case SIF_ADR:
        val = s->sifadr;
        break;
    case SIF_ACL:
        val = s->sifacl;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "tms380: read from unknown offset 0x%02" HWADDR_PRIx "\n",
                      addr);
        break;
    }

    return val;
}

static void tms380_io_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    TMS380PCIState *s = opaque;

    switch (addr) {
    case SIF_DAT:
        s->sifdat = val;
        /* Write to SRAM at current address */
        {
            uint16_t addr16 = s->sifadr & 0xFF;
            if (addr16 + 1 < TMS380_SRAM_SIZE) {
                s->sram[addr16] = (val >> 8) & 0xFF;
                s->sram[addr16 + 1] = val & 0xFF;
                s->sifadr += 2;
            }
        }
        break;
    case SIF_CMD:
        s->sifcmd = val;
        if (val & SIFCMD_INTRESET) {
            s->sifcmd &= ~SIFCMD_ADAP_INT;
            pci_irq_deassert(&s->parent_obj);
        }
        if (val & SIFCMD_EXECUTE) {
            tms380_handle_srb(s);
        }
        break;
    case SIF_ADR:
        s->sifadr = val;
        break;
    case SIF_ACL:
        s->sifacl = val;
        if (val & SIFACL_ARESET) {
            tms380_do_reset(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "tms380: write to unknown offset 0x%02" HWADDR_PRIx
                      " = 0x%04" PRIx64 "\n", addr, val);
        break;
    }
}

static const MemoryRegionOps tms380_io_ops = {
    .read = tms380_io_read,
    .write = tms380_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 2,
    },
};

/* --- PCI device lifecycle --- */

static void tms380_pci_realize(PCIDevice *pci_dev, Error **errp)
{
    TMS380PCIState *s = TMS380_PCI(pci_dev);

    /* Set PCI interrupt pin A — required for IRQ assignment */
    pci_dev->config[PCI_INTERRUPT_PIN] = 1;

    /* Register I/O BAR for SIF registers */
    memory_region_init_io(&s->io_bar, OBJECT(pci_dev), &tms380_io_ops, s,
                          "tms380-sif", TMS380_IO_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->io_bar);

    /* Create reset timer */
    s->reset_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tms380_reset_done, s);

    /* Try to load the backend library */
    tms380_load_backend(s);

    /* Start in reset, auto-complete to ready */
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
