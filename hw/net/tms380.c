/*
 * TMS380 Token Ring Adapter - QEMU Device Model
 *
 * Emulates the TI TMS380C26 (IBM Token Ring 16/4 ISA) at the
 * System Interface (SIF) register level. The Linux tms380tr driver
 * communicates through these registers.
 *
 * Phase 3b: Register skeleton for driver probe. No SRB commands yet.
 *
 * Copyright (c) 2026 Dan Berkowitz
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/isa/isa.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "tms380.h"

/* Initialization data returned via SIFDAT after reset.
 * The tms380tr driver reads this sequence to learn about the adapter. */
static uint16_t tms380_init_data(TMS380State *s, int index)
{
    switch (index) {
    case 0: return 0x0000;  /* Bring-up code: success */
    case 1: return (s->mac[0] << 8) | s->mac[1];  /* MAC word 0 */
    case 2: return (s->mac[2] << 8) | s->mac[3];  /* MAC word 1 */
    case 3: return (s->mac[4] << 8) | s->mac[5];  /* MAC word 2 */
    case 4: return 0x0040;  /* Adapter RAM: 64KB */
    case 5: return 0x0010;  /* Ring speed: 16 Mbps capable */
    default: return 0xFFFF;
    }
}

static void tms380_reset_done(void *opaque)
{
    TMS380State *s = opaque;
    s->dev_state = TMS_STATE_READY;
    s->sifacl &= ~SIFACL_INIT;
    s->init_read_index = 0;
    qemu_log_mask(LOG_GUEST_ERROR, "tms380: reset complete, adapter ready\n");
}

static void tms380_do_reset(TMS380State *s)
{
    s->dev_state = TMS_STATE_RESET;
    s->sifacl |= SIFACL_INIT;
    s->sifdat = 0;
    s->sifcmd = 0;
    s->sifadr = 0;
    s->init_read_index = 0;

    /* Simulate reset delay — timer fires after 10ms virtual time */
    timer_mod(s->reset_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10 * SCALE_MS);
}

static uint64_t tms380_io_read(void *opaque, hwaddr addr, unsigned size)
{
    TMS380State *s = opaque;
    uint16_t val = 0;

    switch (addr) {
    case SIF_DAT:
        if (s->dev_state == TMS_STATE_READY) {
            val = tms380_init_data(s, s->init_read_index);
            s->init_read_index++;
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
                      "tms380: read from unknown register 0x%02" HWADDR_PRIx "\n",
                      addr);
        break;
    }

    return val;
}

static void tms380_io_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    TMS380State *s = opaque;

    switch (addr) {
    case SIF_DAT:
        s->sifdat = val;
        break;
    case SIF_CMD:
        s->sifcmd = val;
        /* If interrupt reset bit is set, clear interrupt */
        if (val & SIFCMD_INTRESET) {
            qemu_irq_lower(s->irq);
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
                      "tms380: write to unknown register 0x%02" HWADDR_PRIx
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

static void tms380_realizefn(DeviceState *dev, Error **errp)
{
    ISADevice *isadev = ISA_DEVICE(dev);
    TMS380State *s = TMS380(dev);

    /* Set up I/O region for the SIF registers */
    memory_region_init_io(&s->io, OBJECT(dev), &tms380_io_ops, s,
                          "tms380-sif", TMS380_IO_SIZE);
    isa_register_ioport(isadev, &s->io, s->iobase);

    /* Get the ISA IRQ line */
    s->irq = isa_get_irq(isadev, s->irq_num);

    /* Create the reset timer */
    s->reset_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tms380_reset_done, s);

    /* Start in reset state, then auto-complete */
    tms380_do_reset(s);

    qemu_log_mask(LOG_GUEST_ERROR,
                  "tms380: adapter at I/O 0x%x IRQ %d MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                  s->iobase, s->irq_num,
                  s->mac[0], s->mac[1], s->mac[2],
                  s->mac[3], s->mac[4], s->mac[5]);
}

static const Property tms380_properties[] = {
    DEFINE_PROP_UINT32("iobase", TMS380State, iobase, 0x0a20),
    DEFINE_PROP_UINT32("irq", TMS380State, irq_num, 9),
    /* MAC address as individual bytes — set via -device tms380,mac=... later.
     * For now use a default locally-administered address. */
    DEFINE_PROP_UINT8("mac0", TMS380State, mac[0], 0x00),
    DEFINE_PROP_UINT8("mac1", TMS380State, mac[1], 0x00),
    DEFINE_PROP_UINT8("mac2", TMS380State, mac[2], 0xF8),
    DEFINE_PROP_UINT8("mac3", TMS380State, mac[3], 0x00),
    DEFINE_PROP_UINT8("mac4", TMS380State, mac[4], 0x00),
    DEFINE_PROP_UINT8("mac5", TMS380State, mac[5], 0x01),
};

static void tms380_class_initfn(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tms380_realizefn;
    device_class_set_props(dc, tms380_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "TMS380 Token Ring 16/4 ISA Adapter";
}

static const TypeInfo tms380_type_info = {
    .name          = TYPE_TMS380,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(TMS380State),
    .class_init    = tms380_class_initfn,
};

static void tms380_register_types(void)
{
    type_register_static(&tms380_type_info);
}

type_init(tms380_register_types)
